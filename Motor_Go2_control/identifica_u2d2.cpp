#include <iostream>
#include <string>
#include <map>
#include <libserialport.h>

struct MotorInfo {
    std::string port_name;
    int superior_id = -1;
    int inferior_id = -1;
};

void listPorts() {
    struct sp_port **ports;

    if (sp_list_ports(&ports) != SP_OK) {
        std::cerr << "Error listando puertos." << std::endl;
        return;
    }

    std::map<std::string, MotorInfo> motors = {
        {"FL", {"", 1, 0}},  // Motor Frontal Izquierdo
        {"FR", {"", 0, 1}},  // Motor Frontal Derecho
        {"RL", {"", 1, 0}},  // Motor Trasero Izquierdo
        {"RR", {"", 1, 0}}   // Motor Trasero Derecho
    };

    // Mapeo de números de serie a etiquetas lógicas
    std::map<std::string, std::string> serial_to_label = {
        {"FT7WBEJZ", "FL"},
        {"FT8ISETK", "FR"},
        {"FT94W6WF", "RL"},
        {"FT89FBGO", "RR"}
    };

    for (int i = 0; ports[i] != nullptr; i++) {
        struct sp_port *port = ports[i];

        const char *nombre = sp_get_port_name(port);
        const char *serie = sp_get_port_usb_serial(port);

        if (serie) {
            std::string serie_str(serie);
            if (serial_to_label.count(serie_str)) {
                std::string label = serial_to_label[serie_str];
                motors[label].port_name = nombre;
            }
        }
    }

    // Mostrar información
    std::cout << "\nAsignación de dispositivos:\n";
    for (const auto& [label, info] : motors) {
        std::cout << "Motor " << label
                  << " -> " << (info.port_name.empty() ? "NO DETECTADO" : info.port_name)
                  << " | Superior ID: " << info.superior_id
                  << " | Inferior ID: " << info.inferior_id
                  << std::endl;
    }
}

int main() {
    listPorts();
    return 0;
}

