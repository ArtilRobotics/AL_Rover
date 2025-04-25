#include <iostream>
#include <string>
#include <map>
#include <thread>
#include <vector>
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <csignal>
#include <atomic>
#include <libserialport.h>
#include <cerrno>
#include <cstring>
#include "motor.h"
#include <spnav.h>

#define MAX_BUFFER_SIZE 1024
#define GEAR_RATIO 6.33f

std::atomic<bool> g_running(true);
Motor g_motor;

std::map<std::string, std::string> logical_serials = {
    {"FR", "FT8ISETK"},  
    {"FL", "FT7WBEJZ"},
    {"RR", "FT89FBGO"},
    {"RL", "FT94W6WF"}
};

std::map<std::string, std::string> detect_ports() {
    std::map<std::string, std::string> serial_to_port;
    struct sp_port **ports;

    if (sp_list_ports(&ports) != SP_OK) return serial_to_port;

    for (int i = 0; ports[i] != nullptr; ++i) {
        const char* serial = sp_get_port_usb_serial(ports[i]);
        if (serial) serial_to_port[serial] = sp_get_port_name(ports[i]);
    }

    sp_free_port_list(ports);
    return serial_to_port;
}

int initialize_serial_port(const char* port_name) {
    int fd = open(port_name, O_RDWR | O_NOCTTY);
    if (fd < 0) return -1;

    struct termios tty {};
    if (tcgetattr(fd, &tty) != 0) { close(fd); return -1; }

    cfsetospeed(&tty, B4000000);
    cfsetispeed(&tty, B4000000);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        close(fd);
        return -1;
    }

    return fd;
}

void send_motor_cmd(const std::string& name, const std::string& port, int id, float speed_rad_s) {
    int fd = initialize_serial_port(port.c_str());
    if (fd < 0) return;

    float k_spd = 0.4f;
    g_motor.setControlParams(0.0f, speed_rad_s * GEAR_RATIO, 0.0f, 0.0f, k_spd / (GEAR_RATIO * GEAR_RATIO));

    Motor::ControlData_t packet = g_motor.createControlPacket(id);
    write(fd, &packet, sizeof(packet));
    close(fd);
}


void control_loop() {
    auto serial_to_port = detect_ports();

    if (spnav_open() == -1) {
        std::cerr << "No se pudo conectar al SpaceMouse\n";
        return;
    }

    const float scale_translation = 0.003f;
    const float scale_rotation = 0.002f;
    const int threshold = 100; // Umbral de sensibilidad mínima
    const float min_speed = 0.3f;
    const float max_speed = 0.8f;
    const float input_max = 350.0f; // Más o menos lo máximo que detecta el SpaceMouse

    std::cout << "Controlando con SpaceMouse...\n";
    std::cout << "CTRL+C para salir\n";

    spnav_event event;
    std::map<std::string, int> fds;

    // Abrir los puertos una sola vez
    for (const auto& pair : logical_serials) {
        const std::string& logical = pair.first;
        const std::string& serial = pair.second;
        const std::string& port = detect_ports()[serial];
        int fd = initialize_serial_port(port.c_str());
        if (fd >= 0) {
            fds[logical] = fd;
        }
    }

    while (g_running) {
        float Vx = 0, Vy = 0, omega = 0;

        if (spnav_poll_event(&event)) {
            if (event.type == SPNAV_EVENT_MOTION) {
                float raw_x = event.motion.x;
                float raw_y = event.motion.y;
                float raw_z = event.motion.z;
                float raw_rz = event.motion.ry;

                // Función de mapeo
                auto map_input = [&](float input) -> float {
                    if (std::abs(input) < threshold) return 0.0f;
                    float norm = std::min(std::abs(input) / input_max, 1.0f);
                    float scaled = min_speed + (max_speed - min_speed) * norm;
                    return (input > 0) ? scaled : -scaled;
                };

                Vx = map_input(raw_z);
                Vy = map_input(-raw_x);
                omega = map_input(raw_rz);

                std::cout << "Mapped Vx: " << Vx << " m/s, "
                          << "Vy: " << Vy << " m/s, "
                          << "Omega: " << omega << " rad/s\n";
            } else if (event.type == SPNAV_EVENT_BUTTON) {
                Vx = 0;
                Vy = 0;
                omega = 0;
            }
        }

        float r = 0.1;
        float L_plus_W = 1.0f;

        float w1 = (1.0f / r) * ( Vx - Vy - L_plus_W * omega);
        float w2 = (1.0f / r) * ( Vx + Vy + L_plus_W * omega);
        float w3 = (1.0f / r) * ( Vx + Vy - L_plus_W * omega);
        float w4 = (1.0f / r) * ( Vx - Vy + L_plus_W * omega);

        if (!fds.empty()) {
            send_motor_cmd("FL", detect_ports()[logical_serials["FL"]], 1, w1);
            send_motor_cmd("FR", detect_ports()[logical_serials["FR"]], 0, -w2);
            send_motor_cmd("RL", detect_ports()[logical_serials["RL"]], 0, w3);
            send_motor_cmd("RR", detect_ports()[logical_serials["RR"]], 0, -w4);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10)); // Más rápido que 50ms
    }

    // Cerrar puertos
    for (auto& fd_pair : fds) {
        close(fd_pair.second);
    }

    spnav_close();
    std::cout << "Saliendo...\n";
}


void signal_handler(int) {
    g_running = false;
}

int main() {
    signal(SIGINT, signal_handler);
    control_loop();
    return 0;
}
