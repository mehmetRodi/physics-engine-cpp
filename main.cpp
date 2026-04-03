#include <SFML/Graphics.hpp>

int main() {
    // Create a window 800x600 pixels
    sf::RenderWindow window(sf::VideoMode(800, 600), "3D Physics Engine - Debug Renderer");

    // This is the "Game Loop"
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear(sf::Color::Black); // Clear screen with black
        // Rendering logic for our shapes will go here later!
        window.display(); // Swap buffers
    }

    return 0;
}
