#include <iostream>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <atomic>
#include <vector>
#include <string>
#include <dirent.h>
#include "motor.h"  // Tu clase Motor

// Colores y emojis
#define RED "\033[1;31m"
#define GREEN "\033[1;32m"
#define YELLOW "\033[1;33m"
#define BLUE "\033[1;34m"
#define RESET "\033[0m"
#define TORQUE_EMOJI "⚙️"
#define SPEED_EMOJI "⚡"
#define POSITION_EMOJI "📍"
#define TEMP_EMOJI "🌡️"
#define ERROR_EMOJI "❌"
#define MENU_EMOJI "🔹"

#define MAX_BUFFER_SIZE 1024
#define GEAR_RATIO 6.33f

// Variables globales
Motor g_motor;
std::atomic<bool> g_running(true);

float g_tor_des = 0.0f;
float g_spd_des = 0.0f;
float g_pos_des = 0.0f;
float g_k_pos = 0.0f;
float g_k_spd = 0.0f;

int g_motor_id = 1;
std::string g_selected_port;

// Declaraciones
void motor_thread();
void interactive_menu();
int initialize_serial_port(const char* port_name);
std::vector<std::string> listar_puertos_serie();
void seleccionar_puerto_y_motor();
void imprimir_estado_actual();


int main() {
    seleccionar_puerto_y_motor();

    g_motor.setControlParams(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    std::thread motor_control_thread(motor_thread);
    interactive_menu();

    g_running = false;
    motor_control_thread.join();
    return 0;
}

/**
 * Lista los puertos serie del sistema (Linux, sin filesystem)
 */
std::vector<std::string> listar_puertos_serie() {
    std::vector<std::string> puertos;
    DIR* dir = opendir("/dev");
    if (!dir) return puertos;

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string name = entry->d_name;
        if (name.find("ttyUSB") != std::string::npos || name.find("ttyACM") != std::string::npos) {
            puertos.push_back("/dev/" + name);
        }
    }

    closedir(dir);
    return puertos;
}

void seleccionar_puerto_y_motor() {
    auto puertos = listar_puertos_serie();

    if (puertos.empty()) {
        std::cerr << RED << "No se encontraron puertos serie disponibles." << RESET << std::endl;
        exit(1);
    }

    std::cout << BLUE << "=== Selección de Puerto Serie ===\n" << RESET;
    for (size_t i = 0; i < puertos.size(); ++i) {
        std::cout << GREEN << i + 1 << ". " << RESET << puertos[i] << "\n";
    }

    int seleccion = 0;
    std::cout << YELLOW << "Seleccione el número del puerto: " << RESET;
    std::cin >> seleccion;

    if (seleccion < 1 || seleccion > puertos.size()) {
        std::cerr << RED << "Selección inválida." << RESET << std::endl;
        exit(1);
    }

    g_selected_port = puertos[seleccion - 1];

    std::cout << YELLOW << "Ingrese el ID del motor (ej. 1): " << RESET;
    std::cin >> g_motor_id;
}

int initialize_serial_port(const char* port_name) {
    int fd = open(port_name, O_RDWR | O_NOCTTY);
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

void motor_thread() {
    int fd = initialize_serial_port(g_selected_port.c_str());
    if (fd < 0) {
        std::cerr << RED << "Error al abrir el puerto " << g_selected_port << RESET << std::endl;
        return;
    }

    uint8_t recv_buffer[MAX_BUFFER_SIZE];

    while (g_running) {
        g_motor.setControlParams(
            g_tor_des / GEAR_RATIO,
            g_spd_des * GEAR_RATIO,
            g_pos_des * GEAR_RATIO,
            g_k_pos / (GEAR_RATIO * GEAR_RATIO),
            g_k_spd / (GEAR_RATIO * GEAR_RATIO)
        );

        Motor::ControlData_t packet = g_motor.createControlPacket(g_motor_id);
        write(fd, &packet, sizeof(Motor::ControlData_t));

        ssize_t bytes_read = read(fd, recv_buffer, MAX_BUFFER_SIZE);
        if (bytes_read >= sizeof(Motor::RecvData_t)) {
            Motor::RecvData_t* recv_packet = reinterpret_cast<Motor::RecvData_t*>(recv_buffer);
            if (recv_packet->head[0] == 0xFD && recv_packet->head[1] == 0xEE) {
                g_motor.updateFeedback(*recv_packet);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

}

void interactive_menu() {
    int option = 0;
    while (g_running) {
        std::cout << BLUE << "\n=== CONTROL DEL MOTOR " << MENU_EMOJI << " ===\n" << RESET;
        std::cout << GREEN << "1. " << RESET << "Detener motor 🛑\n";
        std::cout << GREEN << "2. " << RESET << "Modo Torque " << TORQUE_EMOJI << "\n";
        std::cout << GREEN << "3. " << RESET << "Modo Velocidad " << SPEED_EMOJI << "\n";
        std::cout << GREEN << "4. " << RESET << "Modo Posición " << POSITION_EMOJI << "\n";
        std::cout << GREEN << "5. " << RESET << "Salir 🚪\n";
        std::cout << GREEN << "6. " << RESET << "Leer estado actual del motor 🔍\n";
        
        std::cout << YELLOW << "Seleccione una opción: " << RESET;
        std::cin >> option;

        switch (option) {
            case 1:
                g_tor_des = g_spd_des = g_pos_des = g_k_pos = g_k_spd = 0.0f;
                std::cout << "🛑 Motor detenido\n";
                break;
            case 2:
                std::cout << "Ingrese el torque deseado (0 - 1.0): ";
                std::cin >> g_tor_des;
                g_spd_des = g_pos_des = g_k_pos = g_k_spd = 0.0f;
                break;
            case 3:
                std::cout << "Ingrese la velocidad deseada (rad/s): ";
                std::cin >> g_spd_des;
                g_tor_des = g_pos_des = 0.0f;
                g_k_pos = 0.0f;
                g_k_spd = 0.4f;
                break;
            case 4:
                std::cout << "Ingrese la posición deseada (rad): ";
                std::cin >> g_pos_des;
                g_tor_des = g_spd_des = 0.0f;
                g_k_pos = 60.0f;
                g_k_spd = 5.0f;
                break;
            case 5:
                std::cout << "🚪 Saliendo...\n";
                g_running = false;
                break;
	    case 6:
	    	imprimir_estado_actual();
		break;
            default:
                std::cout << ERROR_EMOJI << " Opción no válida\n";
                break;
        }
    }
}

void imprimir_estado_actual() {
    std::cout << BLUE << "\n=== ESTADO ACTUAL DEL MOTOR ===\n" << RESET;
    std::cout << TORQUE_EMOJI << " Torque:    " << g_motor.getTorque() * GEAR_RATIO << " Nm\n";
    std::cout << SPEED_EMOJI  << " Velocidad: " << g_motor.getSpeed() / GEAR_RATIO << " rad/s\n";
    std::cout << POSITION_EMOJI << " Posición: " << g_motor.getPosition() / GEAR_RATIO << " rad\n";
    std::cout << TEMP_EMOJI << " Temp:      " << g_motor.getTemperature() << " °C\n";
}


