#include "math/Vec3.hpp"
#include "physics/World.hpp"
#include <SFML/Graphics.hpp>
#include <SFML/Graphics/CircleShape.hpp>
#include <iostream>

int main() {
  // Create a window 800x600 pixels
  sf::RenderWindow window(sf::VideoMode(800, 600),
                          "3D Physics Engine - Debug Renderer");

  sf::CircleShape shape(20.f); // visual representation
  shape.setFillColor(sf::Color::Green);

  sf::Clock clock; // to track frame time
  constexpr float fixedDt = 1.0f / 60.0f;
  float accumulator = 0.0f;

  World world(Vec3(0.f, 9.81f * 100.0f, 0.f));
  const World::BodyId body = world.createBody(1.0f, 20.0f);
  world.body(body).position = Vec3(400.f, 100.f, 0.f);

  // This is the "Game Loop"
  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed)
        window.close();
    }
    const float frameDt = clock.restart().asSeconds();
    accumulator += frameDt;

    // 1. Physics Update
    while (accumulator >= fixedDt) {
      world.step(fixedDt);
      if (world.body(body).position.y > 500.f) {
        world.body(body).position.y = 500.f;
        world.body(body).velocity.y *= -0.7f;
      }
      accumulator -= fixedDt;
    }

    // 2. Sync Visuals
    shape.setPosition(world.body(body).position.x, world.body(body).position.y);

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
