#include "math/Vec3.hpp"
#include "physics/RigidBody.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/Graphics/CircleShape.hpp>
#include <iostream>

int main() {
  // Create a window 800x600 pixels
  sf::RenderWindow window(sf::VideoMode(800, 600),
                          "3D Physics Engine - Debug Renderer");

  sf::CircleShape shape(20.f); // visual representation
  shape.setFillColor(sf::Color::Green);

  sf::Clock clock; // to track delta time dt

  RigidBody body(1.0f);
  body.position = Vec3(400, 100, 0);
  // This is the "Game Loop"
  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed)
        window.close();
    }
    float dt = clock.restart().asSeconds();

    // 1. Input/Forces
    body.applyForce(
        Vec3(0, 9.81f * 100.0f, 0)); // Fake gravity (scaled for pixels)

    // 2. Physics Update
    body.update(dt);
    if (body.position.y > 500.f) {
      body.position.y = 500.f;
      body.velocity.y *= -0.7f;
    }

    // 3. Sync Visuals
    shape.setPosition(body.position.x, body.position.y);

    window.clear(sf::Color::Black); // Clear screen with black
    // Rendering logic for our shapes will go here
    window.draw(shape);
    window.display(); // Swap buffers
  }

  Vec3 gravity(0.0f, -9.81f, 0.0f);
  Vec3 velocity(0.0f, 10.0f, 0.0f);

  Vec3 nextVelocity = velocity + gravity;

  std::cout << "Next velocity is: " << nextVelocity.y << std::endl;

  return 0;
}
