/**
 * Sistema de Control y Monitoreo de Motores Multicanal (Versión con I/O Bloqueante)
 */

#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <atomic>
#include <csignal>  // <--- PARA MANEJAR CTRL+C
#include "motor.h"

#define NUM_CHANNELS 1
#define MOTORS_PER_CHANNEL 1
#define MAX_BUFFER_SIZE 1024
#define GEAR_RATIO 6.33f

std::vector<std::vector<Motor>> g_motors(NUM_CHANNELS, std::vector<Motor>(MOTORS_PER_CHANNEL));
std::atomic<bool> g_running(true);

// Variables globales de control
float g_tor_des = 0.0f;
float g_spd_des = 0.0f;
float g_pos_des = 0.0f;
float g_k_pos = 0.0f;
float g_k_spd = 0.0f;

// === MANEJADOR DE SEÑAL SIGINT (CTRL+C) ===
void signal_handler(int signum) {
    std::cout << "\nCtrl+C detectado. Deteniendo los motores...\n";

    // Establecer todos los parámetros en modo stop
    g_tor_des = 0.0f;
    g_spd_des = 0.0f;
    g_pos_des = 0.0f;
    g_k_pos = 0.0f;
    g_k_spd = 0.0f;

    g_running = false;
}

void channel_thread(int channel);
void print_statistics();
int initialize_serial_port(const char* port_name);

int main(int argc, char* argv[]) {
    signal(SIGINT, signal_handler);  // <--- REGISTRO DEL MANEJADOR DE CTRL+C

    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <mode>\n";
        std::cerr << "Modes: stop, tor, speed, pos\n";
        return 1;
    }

    if (strcmp(argv[1], "stop") == 0) {
        g_tor_des = 0.0f;
        g_spd_des = 0.0f;
        g_pos_des = 0.0f;
        g_k_pos = 0.0f;
        g_k_spd = 0.0f;
    } else if (strcmp(argv[1], "tor") == 0) {
        g_tor_des = 0.25f;
        g_spd_des = 0.0f;
        g_pos_des = 0.0f;
        g_k_pos = 0.0f;
        g_k_spd = 0.0f;
    } else if (strcmp(argv[1], "speed") == 0) {
        g_tor_des = 0.0f;
        g_spd_des = 6.28f;
        g_pos_des = 0.0f;
        g_k_pos = 0.0f;
        g_k_spd = 0.4f;
    } else if (strcmp(argv[1], "pos") == 0) {
        g_tor_des = 0.0f;
        g_spd_des = 0.0f;
        g_pos_des = 0.5f;
        g_k_pos = 0.01f;
        g_k_spd = 0.01f;
    } else {
        std::cerr << "Invalid mode.\n";
        return 1;
    }

    std::vector<std::thread> threads;

    for (int i = 0; i < NUM_CHANNELS; ++i) {
        for (int j = 0; j < MOTORS_PER_CHANNEL; ++j) {
            g_motors[i][j].setControlParams(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        }
    }

    for (int i = 0; i < NUM_CHANNELS; ++i) {
        threads.emplace_back(channel_thread, i);
    }

    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));

        for (int i = 0; i < NUM_CHANNELS; ++i) {
            for (int j = 0; j < MOTORS_PER_CHANNEL; ++j) {
                g_motors[i][j].setControlParams(
                    g_tor_des / GEAR_RATIO,
                    g_spd_des * GEAR_RATIO,
                    g_pos_des * GEAR_RATIO,
                    g_k_pos / GEAR_RATIO / GEAR_RATIO,
                    g_k_spd / GEAR_RATIO / GEAR_RATIO
                );

                float tor = g_motors[i][j].getTorque() * GEAR_RATIO;
                float spd = g_motors[i][j].getSpeed() / GEAR_RATIO;
                float pos = g_motors[i][j].getPosition() / GEAR_RATIO;
                float temp = g_motors[i][j].getTemperature();
                uint16_t err = g_motors[i][j].getError();

                std::cout << "Canal " << i << ", Motor " << j
                          << " - Torque de salida: " << tor
                          << ", Velocidad de salida: " << spd
                          << ", Posición de salida: " << pos
                          << ", Temperatura: " << temp
                          << ", Error: " << err << std::endl;
            }
        }

        print_statistics();
    }

    // Esperar a que todos los hilos terminen
    for (auto& thread : threads) {
        thread.join();
    }

    std::cout << "Motores detenidos. Sistema apagado correctamente.\n";
    return 0;
}

int initialize_serial_port(const char* port_name) {
    int fd = open(port_name, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        std::cerr << "Error opening " << port_name << ": " << strerror(errno) << std::endl;
        return -1;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd, &tty) != 0) {
        std::cerr << "Error from tcgetattr: " << strerror(errno) << std::endl;
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
        std::cerr << "Error from tcsetattr: " << strerror(errno) << std::endl;
        close(fd);
        return -1;
    }

    return fd;
}

void channel_thread(int channel) {
    char port_name[20];
    snprintf(port_name, sizeof(port_name), "/dev/ttyUSB%d", channel);

    int fd = initialize_serial_port(port_name);
    if (fd < 0) {
        std::cerr << "Failed to initialize port " << port_name << std::endl;
        return;
    }

    uint8_t recv_buffer[MAX_BUFFER_SIZE];
    int current_motor = 0;

    while (g_running) {
        Motor::ControlData_t packet = g_motors[channel][current_motor].createControlPacket(current_motor);
        uint8_t* p = reinterpret_cast<uint8_t*>(&packet);
        std::cout << "Channel " << channel << ", Motor " << current_motor << " sending packet: ";
        for (size_t i = 0; i < sizeof(Motor::ControlData_t); ++i) {
            printf("%02X ", p[i]);
        }
        std::cout << std::endl;

        ssize_t bytes_written = write(fd, &packet, sizeof(Motor::ControlData_t));
        if (bytes_written == sizeof(Motor::ControlData_t)) {
            g_motors[channel][current_motor].incrementSendCount();
        } else {
            std::cerr << "Send failed: " << strerror(errno) << std::endl;
        }

        ssize_t bytes_read = read(fd, recv_buffer, MAX_BUFFER_SIZE);
        if (bytes_read >= sizeof(Motor::RecvData_t)) {
            Motor::RecvData_t* recv_packet = reinterpret_cast<Motor::RecvData_t*>(recv_buffer);

            if (recv_packet->head[0] == 0xFD && recv_packet->head[1] == 0xEE) {
                uint16_t crc = crc_ccitt(0x2cbb, reinterpret_cast<uint8_t*>(recv_packet), sizeof(Motor::RecvData_t) - 2);
                if (crc == recv_packet->CRC16) {
                    g_motors[channel][recv_packet->mode.id].updateFeedback(*recv_packet);
                    std::cout << "Received data for Motor ID: " << recv_packet->mode.id << std::endl;
                }
            }
        }

        current_motor = (current_motor + 1) % MOTORS_PER_CHANNEL;
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    close(fd);
}

void print_statistics() {
    std::cout << "Channel | Motor | Sent | Received | Lost | Loss Rate\n";
    std::cout << "--------|-------|------|----------|------|----------\n";

    for (int i = 0; i < NUM_CHANNELS; ++i) {
        for (int j = 0; j < MOTORS_PER_CHANNEL; ++j) {
            uint64_t sent = g_motors[i][j].getSendCount();
            uint64_t received = g_motors[i][j].getReceiveCount();
            int64_t lost = sent > received ? sent - received : 0;
            double loss_rate = sent > 0 ? (double)lost / sent * 100 : 0;

            printf("%7d | %5d | %4lu | %8lu | %4ld | %8.2f%%\n",
                   i, j, sent, received, lost, loss_rate);

            g_motors[i][j].resetStats();
        }
    }

    std::cout << std::endl;
}

