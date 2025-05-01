#include <iostream>
#include <libserialport.h>

void listar_puertos_detallado() {
    struct sp_port **ports;
    if (sp_list_ports(&ports) != SP_OK) {
        std::cerr << "Error al listar los puertos." << std::endl;
        return;
    }

    for (int i = 0; ports[i] != nullptr; ++i) {
        struct sp_port *port = ports[i];

        const char* name = sp_get_port_name(port);
        std::cout << "Puerto       : " << (name ? name : "Desconocido") << "\n";

        const char* description = sp_get_port_description(port);
        std::cout << "Descripción  : " << (description ? description : "Desconocido") << "\n";

        const char* manufacturer = sp_get_port_usb_manufacturer(port);
        std::cout << "Fabricante   : " << (manufacturer ? manufacturer : "Desconocido") << "\n";

        const char* product = sp_get_port_usb_product(port);
        std::cout << "Producto     : " << (product ? product : "Desconocido") << "\n";

        const char* serial_number = sp_get_port_usb_serial(port);
        std::cout << "Número serie : " << (serial_number ? serial_number : "Desconocido") << "\n";

        int vid = 0, pid = 0;
	if (sp_get_port_usb_vid_pid(port, &vid, &pid) == SP_OK) {
	    std::cout << "VID:PID      : " << std::hex << vid << ":" << pid << std::dec << "\n";
	} else {
	    std::cout << "VID:PID      : Desconocido\n";
	}

        std::cout << "VID:PID      : " << std::hex << vid << ":" << pid << std::dec << "\n";

        std::cout << "---------------------------------------------\n";
    }

    sp_free_port_list(ports);
}

int main() {
    listar_puertos_detallado();
    return 0;
}

