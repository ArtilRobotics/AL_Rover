#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <cstring>

int16_t remapAxis(int16_t value) {
    // Remapea de [-32767, 32767] a [0, 65534]
    return static_cast<int16_t>(value + 32767);
}

int main() {
    const char *device = "/dev/input/js0";  
    int js_fd = open(device, O_RDONLY);

    if (js_fd < 0) {
        std::cerr << "No se pudo abrir el joystick en " << device << std::endl;
        return 1;
    }

    std::cout << "Joystick detectado en " << device << std::endl;

    // Info del joystick
    char name[128];
    if (ioctl(js_fd, JSIOCGNAME(sizeof(name)), name) < 0)
        strncpy(name, "Desconocido", sizeof(name));
    std::cout << "Nombre: " << name << std::endl;

    // Leer eventos
    struct js_event e;
    while (true) {
        ssize_t bytes = read(js_fd, &e, sizeof(e));
        if (bytes == sizeof(e)) {
            e.type &= ~JS_EVENT_INIT;

            if (e.type == JS_EVENT_BUTTON) {
                std::cout << "Botón " << static_cast<int>(e.number)
                          << (e.value ? " presionado" : " liberado") << std::endl;
            } else if (e.type == JS_EVENT_AXIS) {
                if (e.number == 4 || e.number == 5) {
                    int16_t remapped = remapAxis(e.value);
                    std::cout << "Eje " << static_cast<int>(e.number)
                              << " remapeado: " << remapped << std::endl;
                } else {
                    std::cout << "Eje " << static_cast<int>(e.number)
                              << " valor: " << e.value << std::endl;
                }
            }
        }
    }

    close(js_fd);
    return 0;
}
