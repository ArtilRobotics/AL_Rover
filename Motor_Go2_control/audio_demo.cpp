#include <SFML/Audio.hpp>
#include <iostream>

int main() {
    // Cargar sonidos
    sf::SoundBuffer bufferX, bufferY;
    if (!bufferX.loadFromFile("audio/audio_x.wav") || !bufferY.loadFromFile("audio/audio_y.wav")) {
        std::cerr << "Error cargando los archivos de audio.\n";
        return -1;
    }

    sf::Sound soundX, soundY;
    soundX.setBuffer(bufferX);
    soundY.setBuffer(bufferY);

    // Simular entradas de movimiento
    bool moverX = true;
    bool moverY = true;

    // Reproducir sonidos dependiendo de movimiento
    if (moverX) soundX.play();
    if (moverY) soundY.play();

    // Esperar mientras los sonidos se reproducen
    while (soundX.getStatus() == sf::Sound::Playing || soundY.getStatus() == sf::Sound::Playing) {
        sf::sleep(sf::milliseconds(100));
    }

    return 0;
}
