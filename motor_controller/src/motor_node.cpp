#include "rclcpp/rclcpp.hpp"
#include "motor_controller/msg/motor_state.hpp"
#include "motor_controller/msg/motor_command.hpp"
#include "motor_controller/motor.h"
#include <libserialport.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <dirent.h>
#include <map>
#include <thread>
#include <atomic>

#define MAX_BUFFER_SIZE 1024
#define GEAR_RATIO 6.33f

struct MotorHandle {
    Motor motor;
    int fd;
    int motor_id;
    std::string port_name;
    rclcpp::Publisher<motor_controller::msg::MotorState>::SharedPtr pub;
    rclcpp::Subscription<motor_controller::msg::MotorCommand>::SharedPtr sub;
};

class MotorNode : public rclcpp::Node {
public:
    MotorNode() : Node("motor_node"), running_(true) {
        detect_and_open_motors();
        for (auto& [name, handle] : motors_) {
            setup_ros_interfaces(name, handle);
        }
        thread_ = std::thread([this]() { control_loop(); });
    }

    ~MotorNode() {
        running_ = false;
        if (thread_.joinable())
            thread_.join();
        for (auto& [_, handle] : motors_) {
            if (handle.fd >= 0) close(handle.fd);
        }
    }

private:
    std::map<std::string, MotorHandle> motors_;
    std::atomic<bool> running_;
    std::thread thread_;

    int open_serial(const std::string& port_name) {
        int fd = open(port_name.c_str(), O_RDWR | O_NOCTTY);
        if (fd < 0) return -1;

        struct termios tty;
        memset(&tty, 0, sizeof tty);
        if (tcgetattr(fd, &tty) != 0) {
            close(fd);
            return -1;
        }

        cfsetospeed(&tty, B4000000);
        cfsetispeed(&tty, B4000000);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);
        tty.c_lflag &= ~(ECHO | ICANON | ISIG);
        tty.c_oflag &= ~OPOST;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;

        if (tcsetattr(fd, TCSANOW, &tty) != 0) {
            close(fd);
            return -1;
        }

        return fd;
    }

    void detect_and_open_motors() {
        struct sp_port **ports;
        if (sp_list_ports(&ports) != SP_OK) {
            RCLCPP_ERROR(this->get_logger(), "Error listando puertos.");
            return;
        }

        std::map<std::string, std::string> serial_to_label = {
            {"FTA7NOUS", "FL"},
            {"FT8ISETK", "FR"},
            {"FT94W6WF", "RL"},
            {"FT89FBGO", "RR"}
        };

        std::map<std::string, std::pair<int, int>> motor_ids = {
            {"FL", {0, 2}},
            {"FR", {1, 0}},
            {"RL", {1, 0}},
            {"RR", {1, 0}}
        };

        for (int i = 0; ports[i] != nullptr; ++i) {
            struct sp_port* port = ports[i];
            const char* serial = sp_get_port_usb_serial(port);
            const char* name = sp_get_port_name(port);
            if (serial && name) {
                std::string serial_str(serial);
                std::string port_name(name);
                if (serial_to_label.count(serial_str)) {
                    std::string label = serial_to_label[serial_str];
                    int upper_id = motor_ids[label].first;
                    int lower_id = motor_ids[label].second;

                    int fd = open_serial(port_name);
                    if (fd < 0) {
                        RCLCPP_ERROR(this->get_logger(), "No se pudo abrir %s", port_name.c_str());
                        continue;
                    }

                    motors_[label + "_upper"] = {Motor(), fd, upper_id, port_name};
                    motors_[label + "_lower"] = {Motor(), fd, lower_id, port_name};
                }
            }
        }

        sp_free_port_list(ports);
    }

    void setup_ros_interfaces(const std::string& name, MotorHandle& handle) {
        handle.pub = this->create_publisher<motor_controller::msg::MotorState>("/motors/" + name + "/state", 10);

        handle.sub = this->create_subscription<motor_controller::msg::MotorCommand>(
            "/motors/" + name + "/cmd", 10,
            [this, name](const motor_controller::msg::MotorCommand::SharedPtr msg) {
                handle_command(name, msg);
            }
        );
    }

    void handle_command(const std::string& name, const motor_controller::msg::MotorCommand::SharedPtr msg) {
        auto& m = motors_[name];
        m.motor.setControlParams(
            msg->torque_desired / GEAR_RATIO,
            msg->speed_desired * GEAR_RATIO,
            msg->position_desired * GEAR_RATIO,
            msg->k_pos / (GEAR_RATIO * GEAR_RATIO),
            msg->k_spd / (GEAR_RATIO * GEAR_RATIO)
        );
    }

    void control_loop() {
        uint8_t recv_buf[MAX_BUFFER_SIZE];

        while (rclcpp::ok() && running_) {
            for (auto& [name, handle] : motors_) {
                auto packet = handle.motor.createControlPacket(handle.motor_id);
                write(handle.fd, &packet, sizeof(Motor::ControlData_t));

                ssize_t n = read(handle.fd, recv_buf, MAX_BUFFER_SIZE);
                if (n >= static_cast<ssize_t>(sizeof(Motor::RecvData_t))) {
                    Motor::RecvData_t* recv = reinterpret_cast<Motor::RecvData_t*>(recv_buf);
                    if (recv->head[0] == 0xFD && recv->head[1] == 0xEE) {
                        handle.motor.updateFeedback(*recv);
                    }
                }

                motor_controller::msg::MotorState msg;
                msg.torque = handle.motor.getTorque() * GEAR_RATIO;
                msg.speed = handle.motor.getSpeed() / GEAR_RATIO;
                msg.position = handle.motor.getPosition() / GEAR_RATIO;
                msg.temperature = handle.motor.getTemperature();
                msg.error = handle.motor.getError();

                handle.pub->publish(msg);
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

