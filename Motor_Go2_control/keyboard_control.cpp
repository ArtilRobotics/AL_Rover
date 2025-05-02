// === INCLUDES ===
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <atomic>
#include <csignal>
#include <libserialport.h>
#include <map>
#include <cmath>
#include "motor.h"

#define MAX_BUFFER_SIZE 1024
#define GEAR_RATIO 6.33f

float k_pos = 5.0f;
const float dt = 0.5f;
const float velocidad = 0.05f;
const float paso_incremental = 0.1f;

// === Estructura Configuración Canal ===
struct ChannelConfig {
    std::string label;
    std::string port_name;
    int motor_id;
};


std::map<std::string, Motor> g_motors;
std::map<std::string, ChannelConfig> g_configs;
std::atomic<bool> g_running(true);

// Mapas globales para control por teclado
std::map<std::string, std::string> logical_serials = {
    {"FR", "FT8ISETK"},
    {"FL", "FT7WBEJZ"},
    {"RR", "FT89FBGO"},
    {"RL", "FT94W6WF"}
};
Motor g_motor;


// === Señal de salida ===
void signal_handler(int) {
    std::cout << "\nCtrl+C detectado. Apagando...\n";
    g_running = false;
}

// === Inicializa puerto serie ===
int initialize_serial_port(const char* port_name) {
    int fd = open(port_name, O_RDWR | O_NOCTTY);
    if (fd < 0) return -1;

    struct termios tty {};
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

// === Detecta puertos serie disponibles y motores ===
std::vector<ChannelConfig> detect_channels() {
    std::map<std::string, std::string> serial_to_label = {
        {"FT7WBEJZ", "FL"},
        {"FT8ISETK", "FR"},
        {"FT94W6WF", "RL"},
        {"FT89FBGO", "RR"}
    };

    std::map<std::string, int> label_to_motor_id = {
        {"FL", 0}, {"FR", 1}, {"RL", 1}, {"RR", 1}
    };

    struct sp_port **ports;
    std::vector<ChannelConfig> configs;

    if (sp_list_ports(&ports) != SP_OK) {
        std::cerr << "Error listando puertos.\n";
        return configs;
    }

    for (int i = 0; ports[i] != nullptr; ++i) {
        struct sp_port* port = ports[i];
        const char* serial = sp_get_port_usb_serial(port);
        if (serial) {
            std::string serial_str(serial);
            if (serial_to_label.count(serial_str)) {
                std::string label = serial_to_label[serial_str];
                int motor_id = label_to_motor_id[label];
                ChannelConfig cfg = {label, sp_get_port_name(port), motor_id};
                configs.push_back(cfg);
                g_configs[label] = cfg;
                g_motors[label] = Motor();
            }
        }
    }

    sp_free_port_list(ports);
    return configs;
}

// === Hilo por canal ===
void channel_thread(ChannelConfig cfg) {
    int fd = initialize_serial_port(cfg.port_name.c_str());
    if (fd < 0) {
        std::cerr << "No se pudo abrir el puerto " << cfg.port_name << "\n";
        return;
    }

    uint8_t recv_buffer[MAX_BUFFER_SIZE];

    while (g_running) {
        auto packet = g_motors[cfg.label].createControlPacket(cfg.motor_id);
        write(fd, reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
        g_motors[cfg.label].incrementSendCount();

        ssize_t bytes_read = read(fd, recv_buffer, MAX_BUFFER_SIZE);
        if (bytes_read >= sizeof(Motor::RecvData_t)) {
            auto* recv_packet = reinterpret_cast<Motor::RecvData_t*>(recv_buffer);
            if (recv_packet->head[0] == 0xFD && recv_packet->head[1] == 0xEE) {
                uint16_t crc = crc_ccitt(0x2cbb, reinterpret_cast<uint8_t*>(recv_packet), sizeof(Motor::RecvData_t) - 2);
                if (crc == recv_packet->CRC16) {
                    g_motors[cfg.label].updateFeedback(*recv_packet);
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    close(fd);
}

// === Control por teclado estilo W/A/S/D ===
char get_key() {
    struct termios oldt, newt;
    char ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

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

void send_motor_cmd(const std::string& name, const std::string& port, int id, float speed_rad_s) {
    int fd = initialize_serial_port(port.c_str());
    if (fd < 0) return;

    float k_spd = 0.4f;
    g_motor.setControlParams(0.0f, speed_rad_s * GEAR_RATIO, 0.0f, 0.0f, k_spd / (GEAR_RATIO * GEAR_RATIO));

    Motor::ControlData_t packet = g_motor.createControlPacket(id);
    write(fd, &packet, sizeof(packet));
    close(fd);
}


// === Menú Interactivo ===
void interactive_menu() {
    std::map<std::string, int> pos_sign = {
        {"FL", 1},
        {"FR", -1},
        {"RL", -1},
        {"RR", 1}
    };


    float Vx = 0, Vy = 0, omega = 0;
    std::cout << "Controles:\n";
    std::cout << "  W/A/S/D → Movimiento\n";
    std::cout << "  Flechas (↑↓←→) → Rotación\n";
    std::cout << "  M → Subir posición\n";
    std::cout << "  N → Bajar posición\n";
    std::cout << "  U → Detener motores\n";
    std::cout << "  Q para salir\n";

    std::map<std::string, float> pos_ref;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    for (const auto& [label, motor] : g_motors) {
        pos_ref[label] = motor.getPosition() / GEAR_RATIO;
    }

    std::cout << "=== Posiciones iniciales ===\n";
    for (const auto& [label, pos] : pos_ref) {
        std::cout << label << ": " << pos << " rad\n";
    }

    while (g_running) {
        char key = get_key();
        Vx = Vy = omega = 0;

        if (key == 'q') break;

        if (key == 'w') Vx = 0.5;
        else if (key == 's') Vx = -0.5;
        else if (key == 'a') Vy = 0.5;
        else if (key == 'd') Vy = -0.5;
        else if (key == 27 && get_key() == '[') {
            char arrow = get_key();
            if (arrow == 'A' || arrow == 'D') omega = 0.5;
            else if (arrow == 'B' || arrow == 'C') omega = -0.5;
        }

        float r = 0.1f;
        float L_plus_W = 1.0f;

        float w1 = (1 / r) * (Vx - Vy - L_plus_W * omega);
        float w2 = (1 / r) * (Vx + Vy + L_plus_W * omega);
        float w3 = (1 / r) * (Vx + Vy - L_plus_W * omega);
        float w4 = (1 / r) * (Vx - Vy + L_plus_W * omega);

        auto ports = detect_ports();

        send_motor_cmd("FL", ports[logical_serials["FL"]], 1, w1);
        send_motor_cmd("FR", ports[logical_serials["FR"]], 0, -w2);
        send_motor_cmd("RL", ports[logical_serials["RL"]], 0, w3);
        send_motor_cmd("RR", ports[logical_serials["RR"]], 0, -w4);

        if (key == 'm' || key == 'n') {
            float delta = (key == 'm') ? paso_incremental : -paso_incremental;
            for (auto& [label, motor] : g_motors) {
                float nueva_pos = pos_ref[label] + delta * pos_sign[label];
                if ((delta > 0) || (delta < 0 && nueva_pos >= pos_ref[label])) {
                    pos_ref[label] = nueva_pos;
                    motor.setControlParams(0, 0, nueva_pos * GEAR_RATIO, k_pos, 0);
                    std::cout << label << " → Posición objetivo actualizada: " << nueva_pos << " rad\n";
                }
            }
        }

        if (key == 'u') {
            for (auto& [label, motor] : g_motors) {
                float actual = motor.getPosition() / GEAR_RATIO;
                motor.setControlParams(0, 0, actual * GEAR_RATIO, k_pos, 0);
            }
            std::cout << "Motores detenidos.\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}


int main() {
    signal(SIGINT, signal_handler);

    auto configs = detect_channels();
    if (configs.empty()) {
        std::cerr << "No se detectaron motores válidos.\n";
        return 1;
    }

    std::vector<std::thread> threads;
    for (const auto& cfg : configs)
        threads.emplace_back(channel_thread, cfg);

    interactive_menu();

    for (auto& t : threads) t.join();

    std::cout << "Sistema apagado correctamente.\n";
    return 0;
}