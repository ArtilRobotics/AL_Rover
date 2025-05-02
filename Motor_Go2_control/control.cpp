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
#include <linux/joystick.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdint>
#include <cstring>


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
//Joystick Variables
std::atomic<float> Vx(0.0), Vy(0.0), omega(0.0), delta_altura(0.0);
std::atomic<bool> detener_motores(false);

// Variable de velocidades
std::atomic<float> velocidad_factor(0.5f);  // valor inicial


// Mapas globales para control por teclado
std::map<std::string, std::string> logical_serials = {
    {"FR", "FT8ISETK"},
    {"FL", "FT7WBEJZ"},
    {"RR", "FT89FBGO"},
    {"RL", "FT94W6WF"}
};
Motor g_motor;

uint16_t remapAxis(int16_t value) {
    return static_cast<uint16_t>(static_cast<int32_t>(value) + 32768);
}

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

// === Control para el Joystick ====
void joystick_thread() {
    const char* device = "/dev/input/js0";
    int js_fd = open(device, O_RDONLY);
    if (js_fd < 0) {
        std::cerr << "No se pudo abrir el joystick en " << device << std::endl;
        return;
    }

    struct js_event e;

    while (g_running) {
        ssize_t bytes = read(js_fd, &e, sizeof(e));
        if (bytes == sizeof(e)) {
            e.type &= ~JS_EVENT_INIT;

            if (e.type == JS_EVENT_AXIS) {
                float val = e.value / 32767.0f;

                switch (e.number) {
                    case 2: Vy = -val * velocidad_factor.load(); break;       // Eje lateral
                    case 1: Vx = -val * velocidad_factor.load(); break;      // Eje frontal (inverso)
                    case 0: omega = -val * velocidad_factor.load(); break;    // Rotación
                    //case 3: delta_altura = -val * paso_incremental; break; // Altura
                    case 4: {  // Gatillo izquierdo (subir)
                        float normalized = remapAxis(e.value) / 65535.0f;
                        delta_altura = normalized * paso_incremental;
                        break;
                    }
                    case 5: {  // Gatillo derecho (bajar)
                        float normalized = remapAxis(e.value) / 65535.0f;
                        delta_altura = -normalized * paso_incremental;
                        break;
                    }
                }
            }

            if (e.type == JS_EVENT_BUTTON && e.number == 3 && e.value) {
                detener_motores = true;
            }
            
            if (e.number == 10 && e.value) { // Restar velocidad
                float v = velocidad_factor.load();
                velocidad_factor.store(std::max(0.1f, v - 0.1f));
                std::cout << "[Botón 10] Reduciendo velocidad: " << velocidad_factor.load() << std::endl;
            }
            if (e.number == 11 && e.value) { // Aumentar velocidad
                float v = velocidad_factor.load();
                velocidad_factor.store(std::min(1.0f, v + 0.1f));
                std::cout << "[Botón 11] Aumentando velocidad: " << velocidad_factor.load() << std::endl;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    close(js_fd);
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


void joystick_loop() {
    std::map<std::string, float> pos_ref;
    std::map<std::string, int> pos_sign = {
        {"FL", 1},
        {"FR", -1},
        {"RL", -1},
        {"RR", 1}
    };

    auto ports = detect_ports();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    for (const auto& [label, motor] : g_motors) {
        pos_ref[label] = motor.getPosition() / GEAR_RATIO;
    }

    while (g_running) {
        float vx = Vx.load();
        float vy = Vy.load();
        float w = omega.load();

        float r = 0.1f;
        float L_plus_W = 1.0f;

        float w1 = (1 / r) * (vx - vy - L_plus_W * w);
        float w2 = (1 / r) * (vx + vy + L_plus_W * w);
        float w3 = (1 / r) * (vx + vy - L_plus_W * w);
        float w4 = (1 / r) * (vx - vy + L_plus_W * w);

        std::cout << "\n[Control] Vx: " << vx
          << ", Vy: " << vy
          << ", omega: " << w
          << "\n[Control] w1: " << w1 << " w2: " << w2
          << " w3: " << w3 << " w4: " << w4 << std::endl;


        send_motor_cmd("FL", ports[logical_serials["FL"]], 1, w1);
        send_motor_cmd("FR", ports[logical_serials["FR"]], 0, -w2);
        send_motor_cmd("RL", ports[logical_serials["RL"]], 0, w3);
        send_motor_cmd("RR", ports[logical_serials["RR"]], 0, -w4);

        float delta = delta_altura.load();
        if (std::abs(delta) > 1e-3) {
            for (auto& [label, motor] : g_motors) {
                float nueva_pos = pos_ref[label] + delta * pos_sign[label];
                pos_ref[label] = nueva_pos;
                motor.setControlParams(0, 0, nueva_pos * GEAR_RATIO, 60.0f / (GEAR_RATIO * GEAR_RATIO), 5.0f / (GEAR_RATIO * GEAR_RATIO));

                std::cout << "[Posición] " << label << " → Nueva pos: " << nueva_pos << " rad\n";
            }
        }

        if (detener_motores.exchange(false)) {
            std::cout << "[Botón] Detener motores activado." << std::endl;
            for (auto& [label, motor] : g_motors) {
                float actual = motor.getPosition() / GEAR_RATIO;
                motor.setControlParams(0, 0, actual * GEAR_RATIO, k_pos, 0);
            }
            std::cout << "Botón de parada presionado. Motores detenidos.\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
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

    std::thread js_thread(joystick_thread);   

    joystick_loop();

    for (auto& t : threads) t.join();
    js_thread.join();

    std::cout << "Sistema apagado correctamente.\n";
    return 0;
}