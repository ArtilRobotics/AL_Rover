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

#define MAX_BUFFER_SIZE 1024
#define GEAR_RATIO 6.33f

std::atomic<bool> g_running(true);
Motor g_motor;

// Mapea nombre lógico a número de serie USB
std::map<std::string, std::string> logical_serials = {
    {"FR", "FT8ISETK"},  
    {"FL", "FT7WBEJZ"},   // w1 y w2
    {"RR", "FT89FBGO"},   // w3
    {"RL", "FT94W6WF"}    // w4
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

    float Vx = 0, Vy = 0, omega = 0;
    char key;

    std::cout << "Controles:\n";
    std::cout << "  W/A/S/D → Movimiento\n";
    std::cout << "  Flechas (↑↓←→) → Rotación\n";
    std::cout << "  Q para salir\n";

    while (g_running) {
        key = get_key();

        Vx = Vy = omega = 0;

        if (key == 'q') {
            g_running = false;
            break;
        }

        if (key == 'w') Vx = 1;
        else if (key == 's') Vx = -1;
        else if (key == 'a') Vy = 1;
        else if (key == 'd') Vy = -1;
        else if (key == 27 && get_key() == '[') {
            char arrow = get_key();
            if (arrow == 'A' || arrow == 'D') omega = 1;
            else if (arrow == 'B' || arrow == 'C') omega = -1;
        }

        float r = 0.1;
        float L_plus_W = 1.0f;

        float w1 = (1.0f / r) * ( Vx - Vy - L_plus_W * omega);
        float w2 = (1.0f / r) * ( Vx + Vy + L_plus_W * omega);
        float w3 = (1.0f / r) * ( Vx + Vy - L_plus_W * omega);
        float w4 = (1.0f / r) * ( Vx - Vy + L_plus_W * omega);

        std::map<std::string, std::string> ports = {
            {"FF", serial_to_port[logical_serials["FF"]]},
            {"RR", serial_to_port[logical_serials["RR"]]},
            {"RL", serial_to_port[logical_serials["RL"]]}
        };

        // Mapeo: FF ID 1 = w1, FF ID 0 = w2, RR ID 0 = w3, RL ID 0 = w4
        send_motor_cmd("FF", ports["FF"], 0, w1);
        send_motor_cmd("FF", ports["FF"], 0, w2);
        send_motor_cmd("RR", ports["RR"], 0, w3);
        send_motor_cmd("RL", ports["RL"], 0, w4);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));


	if (key == 'e') {
	    // Detener todos los motores
	    send_motor_cmd("FF", ports["FF"], 0, 0);
	    send_motor_cmd("FF", ports["FF"], 0, 0);
	    send_motor_cmd("RR", ports["RR"], 0, 0);
	    send_motor_cmd("RL", ports["RL"], 0, 0);
	}

    }

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

