#include <iostream>
#include <string>
#include <map>
#include <vector>
#include <thread>
#include <chrono>
#include <atomic>
#include <csignal>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <libserialport.h>
#include "motor.h"  // Asegúrate de tener esto bien definido

#define MAX_BUFFER_SIZE 1024
#define GEAR_RATIO 6.33f

struct MotorInfo {
    std::string port_name;
    int motor_id_1 = -1;
    int motor_id_2 = -1;
};

struct ChannelConfig {
    std::string label;
    std::string port_name;
    int motor_id_1;
    int motor_id_2;
};

std::map<std::string, std::array<Motor, 2>> g_motors;
std::atomic<bool> g_running(true);

// === Señal de salida ===
void signal_handler(int signum) {
    std::cout << "\nCtrl+C detectado. Cerrando el sistema...\n";
    g_running = false;
}

// === Inicialización del puerto serie ===
int initialize_serial_port(const char* port_name) {
    int fd = open(port_name, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::cerr << "Error abriendo " << port_name << ": " << strerror(errno) << std::endl;
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error desde tcgetattr: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    cfsetospeed(&tty, B4000000);
    cfsetispeed(&tty, B4000000);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::cerr << "Error desde tcsetattr: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    return fd;
}

// === Detectar puertos conectados ===
std::vector<ChannelConfig> detect_connected_channels() {
    // IDs de Motores, 1 up, 2 down
    std::map<std::string, MotorInfo> motors = {
        {"FL", {"", 0, 1}},
        {"FR", {"", 1, 0}},
        {"RL", {"", 1, 0}},
        {"RR", {"", 1, 0}}
    };
    // Serial Numbers U2D2 to identify the parts
    std::map<std::string, std::string> serial_to_label = {
        {"FT7WBEJZ", "FL"},
        {"FT8ISETK", "FR"},
        {"FT94W6WF", "RL"},
        {"FT89FBGO", "RR"}
    };
    // Signs to motor 1:+ and -1:-
    std::map<std::string, int> lift_direction_sign = {
        {"FL", -1},
        {"FR", 1},
        {"RL", -1},
        {"RR", 1}
    };
    

    struct sp_port **ports;
    if (sp_list_ports(&ports) != SP_OK) {
        std::cerr << "Error listando puertos." << std::endl;
        return {};
    }

    for (int i = 0; ports[i] != nullptr; i++) {
        struct sp_port *port = ports[i];
        const char* nombre = sp_get_port_name(port);
        const char* serie = sp_get_port_usb_serial(port);

        if (serie) {
            std::string serie_str(serie);
            if (serial_to_label.count(serie_str)) {
                std::string label = serial_to_label[serie_str];
                motors[label].port_name = nombre;
            }
        }
    }

    std::vector<ChannelConfig> configs;
    for (const auto& [label, info] : motors) {
        if (!info.port_name.empty()) {
            g_motors[label] = {Motor(), Motor()};
            configs.push_back({label, info.port_name, info.motor_id_1, info.motor_id_2});
        }
    }

    return configs;
}

// === Hilo por canal ===
void channel_thread(ChannelConfig cfg) {
    int fd = initialize_serial_port(cfg.port_name.c_str());
    if (fd < 0) {
        std::cerr << "No se pudo abrir el puerto " << cfg.port_name << " (" << cfg.label << ")\n";
        return;
    }

    uint8_t recv_buffer[MAX_BUFFER_SIZE];
    int ids[] = {cfg.motor_id_1, cfg.motor_id_2};
    int index = 0;

    while (g_running) {
        int motor_id = ids[index];
        g_motors[cfg.label][motor_id].setControlParams(0, 0, 0, 0, 0);
        Motor::ControlData_t packet = g_motors[cfg.label][motor_id].createControlPacket(motor_id);
        write(fd, reinterpret_cast<uint8_t*>(&packet), sizeof(Motor::ControlData_t));


        ssize_t bytes_read = read(fd, recv_buffer, MAX_BUFFER_SIZE);
        if (bytes_read >= static_cast<ssize_t>(sizeof(Motor::RecvData_t))) {
            auto* recv_packet = reinterpret_cast<Motor::RecvData_t*>(recv_buffer);
            if (recv_packet->head[0] == 0xFD && recv_packet->head[1] == 0xEE) {
                uint16_t crc = crc_ccitt(0x2cbb, reinterpret_cast<uint8_t*>(recv_packet), sizeof(Motor::RecvData_t) - 2);
                if (crc == recv_packet->CRC16) {
                    g_motors[cfg.label][recv_packet->mode.id].updateFeedback(*recv_packet);
                }
            }
        }

        index = (index + 1) % 2;
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    close(fd);
}

// === Estadísticas ===
void print_statistics() {
    std::cout << "\nLabel   | Motor | Enviados | Recibidos | Perdidos | Pérdida (%)\n";
    std::cout << "--------|--------|----------|-----------|----------|-------------\n";

    for (const auto& [label, motors] : g_motors) {
        for (int i = 0; i < 2; ++i) {
            uint64_t sent = motors[i].getSendCount();
            uint64_t received = motors[i].getReceiveCount();
            int64_t lost = sent > received ? sent - received : 0;
            double loss_rate = sent > 0 ? (double)lost / sent * 100 : 0;

            printf("%7s | %6d | %8lu | %9lu | %8ld | %10.2f%%\n",
                   label.c_str(), i, sent, received, lost, loss_rate);

            g_motors[label][i].resetStats();
        }
    }

    std::cout << std::endl;
}

void superior_positions(const std::vector<ChannelConfig>& channels) {
    std::cout << "\n=== Posiciones de los motores superiores ===\n";
    for (const auto& cfg : channels) {
        const auto& label = cfg.label;
        int superior_id = cfg.motor_id_1;

        if (g_motors.count(label)) {
            float pos = g_motors[label][superior_id].getPosition() / GEAR_RATIO;
            std::cout << label << " [" << superior_id << "] = " << pos << std::endl;
        }
    }
    std::cout << std::endl;


    std::cout << "\n=== Posiciones de los motores inferiores ===\n";
    for (const auto& cfg : channels) {
        const auto& label = cfg.label;
        int superior_id = cfg.motor_id_2;

        if (g_motors.count(label)) {
            float pos = g_motors[label][superior_id].getPosition() / GEAR_RATIO;
            std::cout << label << " [" << superior_id << "] = " << pos << std::endl;
        }
    }
    std::cout << std::endl;
}



// === Main principal ===
int main() {
    signal(SIGINT, signal_handler);

    auto channels = detect_connected_channels();
    if (channels.empty()) {
        std::cerr << "No se detectaron canales válidos. Abortando.\n";
        return 1;
    }

    std::vector<std::thread> threads;
    for (const auto& cfg : channels) {
        threads.emplace_back(channel_thread, cfg);
    }

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        for (const auto& [label, motors] : g_motors) {
            for (int i = 0; i < 2; ++i) {
                float tor = motors[i].getTorque() * GEAR_RATIO;
                float spd = motors[i].getSpeed() / GEAR_RATIO;
                float pos = motors[i].getPosition() / GEAR_RATIO;
                float temp = motors[i].getTemperature();
                uint16_t err = motors[i].getError();

                std::cout << "Motor " << label << " [" << i << "] - "
                          << "Torque: " << tor
                          << ", Velocidad: " << spd
                          << ", Posición: " << pos
                          << ", Temperatura: " << temp
                          << ", Error: " << err << std::endl;
            }
        }

        print_statistics();
        superior_positions(channels);

    }

    for (auto& thread : threads) {
        thread.join();
    }

    std::cout << "Sistema finalizado correctamente...\n";
    return 0;
}

