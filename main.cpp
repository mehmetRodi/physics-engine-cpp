#include "math/Vec3.hpp"
#include "physics/World.hpp"

#include <SFML/Graphics.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <sstream>
#include <string>
#include <vector>

namespace {
constexpr unsigned windowWidth = 960;
constexpr unsigned windowHeight = 640;
constexpr float fixedDt = 1.0f / 60.0f;
constexpr float arenaMargin = 36.0f;
constexpr std::size_t bodyCount = 30;

struct RenderBody {
  World::BodyId id;
  sf::Color color;
};

struct DemoScene {
  World world;
  std::vector<RenderBody> bodies;
  std::uint64_t steps = 0;
};

sf::FloatRect arenaBounds() {
  return sf::FloatRect(arenaMargin, arenaMargin,
                       static_cast<float>(windowWidth) - 2.0f * arenaMargin,
                       static_cast<float>(windowHeight) - 2.0f * arenaMargin);
}

sf::Color colorForBody(std::size_t index) {
  static const std::array<sf::Color, 6> palette = {
      sf::Color(80, 220, 170),  sf::Color(255, 205, 90),  sf::Color(110, 170, 255),
      sf::Color(240, 110, 130), sf::Color(180, 140, 255), sf::Color(125, 215, 245),
  };

  return palette[index % palette.size()];
}

DemoScene createScene() {
  DemoScene scene{World(Vec3(0.0f, 0.0f, 0.0f)), {}, 0};
  scene.world.reserveRigidBodies(bodyCount);
  scene.bodies.reserve(bodyCount);

  constexpr std::size_t columns = 6;
  constexpr float startX = 130.0f;
  constexpr float startY = 105.0f;
  constexpr float spacingX = 135.0f;
  constexpr float spacingY = 95.0f;

  for (std::size_t i = 0; i < bodyCount; ++i) {
    const float radius = 12.0f + static_cast<float>((i * 7) % 10);
    const float mass = 0.8f + static_cast<float>(i % 5) * 0.45f;
    const World::BodyId id = scene.world.createBody(mass, radius);

    RigidBody& body = scene.world.rigidBody(id);
    body.position = Vec3(startX + static_cast<float>(i % columns) * spacingX,
                         startY + static_cast<float>(i / columns) * spacingY, 0.0f);

    const float xDirection = (i % 2 == 0) ? 1.0f : -1.0f;
    const float yDirection = ((i / 2) % 2 == 0) ? 1.0f : -1.0f;
    body.velocity = Vec3(xDirection * (65.0f + static_cast<float>(i % 5) * 18.0f),
                         yDirection * (45.0f + static_cast<float>(i % 7) * 13.0f), 0.0f);
    body.material.restitution = 0.85f + static_cast<float>(i % 4) * 0.05f;

    scene.bodies.push_back({id, colorForBody(i)});
  }

  return scene;
}

void constrainToArena(World& world, const std::vector<RenderBody>& bodies) {
  const sf::FloatRect bounds = arenaBounds();
  const float left = bounds.left;
  const float right = bounds.left + bounds.width;
  const float top = bounds.top;
  const float bottom = bounds.top + bounds.height;

  for (const RenderBody& renderBody : bodies) {
    RigidBody& body = world.rigidBody(renderBody.id);
    const float restitution = body.material.restitution;

    if (body.position.x - body.radius < left) {
      body.position.x = left + body.radius;
      body.velocity.x = std::abs(body.velocity.x) * restitution;
    } else if (body.position.x + body.radius > right) {
      body.position.x = right - body.radius;
      body.velocity.x = -std::abs(body.velocity.x) * restitution;
    }

    if (body.position.y - body.radius < top) {
      body.position.y = top + body.radius;
      body.velocity.y = std::abs(body.velocity.y) * restitution;
    } else if (body.position.y + body.radius > bottom) {
      body.position.y = bottom - body.radius;
      body.velocity.y = -std::abs(body.velocity.y) * restitution;
    }
  }
}

bool loadDebugFont(sf::Font& font) {
  static constexpr std::array<const char*, 6> fontPaths = {
      "assets/Inter-Regular.ttf",
      "assets/Arial.ttf",
      "/System/Library/Fonts/Supplemental/Arial.ttf",
      "/System/Library/Fonts/Supplemental/Helvetica.ttf",
      "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
      "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
  };

  for (const char* path : fontPaths) {
    if (font.loadFromFile(path)) {
      return true;
    }
  }

  return false;
}

std::string debugString(bool paused, const DemoScene& scene) {
  std::ostringstream out;
  out << "Rigid-body sphere demo | bodies: " << scene.bodies.size() << " | fixed dt: 1/60s"
      << " | steps: " << scene.steps << " | " << (paused ? "paused" : "running")
      << " | Space pause, N step, R reset, Esc quit";
  return out.str();
}
} // namespace

int main() {
  sf::RenderWindow window(sf::VideoMode(windowWidth, windowHeight), "Rigid-body sphere demo");
  window.setVerticalSyncEnabled(true);

  DemoScene scene = createScene();
  bool paused = false;
  float accumulator = 0.0f;
  sf::Clock clock;

  sf::Font font;
  const bool hasDebugFont = loadDebugFont(font);
  sf::Text debugText;
  if (hasDebugFont) {
    debugText.setFont(font);
    debugText.setCharacterSize(15);
    debugText.setFillColor(sf::Color(225, 230, 235));
    debugText.setPosition(18.0f, 10.0f);
  }

  sf::RectangleShape arena;
  const sf::FloatRect bounds = arenaBounds();
  arena.setPosition(bounds.left, bounds.top);
  arena.setSize(sf::Vector2f(bounds.width, bounds.height));
  arena.setFillColor(sf::Color(18, 22, 28));
  arena.setOutlineColor(sf::Color(90, 105, 120));
  arena.setOutlineThickness(2.0f);

  sf::CircleShape shape;
  shape.setOutlineThickness(2.0f);
  shape.setOutlineColor(sf::Color(12, 15, 18));

  while (window.isOpen()) {
    sf::Event event;
    while (window.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        window.close();
      } else if (event.type == sf::Event::KeyPressed) {
        if (event.key.code == sf::Keyboard::Escape) {
          window.close();
        } else if (event.key.code == sf::Keyboard::Space) {
          paused = !paused;
        } else if (event.key.code == sf::Keyboard::R) {
          scene = createScene();
          accumulator = 0.0f;
          clock.restart();
        } else if (event.key.code == sf::Keyboard::N && paused) {
          scene.world.step(fixedDt);
          constrainToArena(scene.world, scene.bodies);
          ++scene.steps;
        }
      }
    }

    const float frameDt = std::min(clock.restart().asSeconds(), 0.25f);
    if (!paused) {
      accumulator += frameDt;
    }

    while (accumulator >= fixedDt) {
      scene.world.step(fixedDt);
      constrainToArena(scene.world, scene.bodies);
      accumulator -= fixedDt;
      ++scene.steps;
    }

    const std::string status = debugString(paused, scene);
    window.setTitle(status);
    if (hasDebugFont) {
      debugText.setString(status);
    }

    window.clear(sf::Color(8, 10, 14));
    window.draw(arena);

    for (const RenderBody& renderBody : scene.bodies) {
      const RigidBody& body = scene.world.body(renderBody.id);
      shape.setRadius(body.radius);
      shape.setOrigin(body.radius, body.radius);
      shape.setPosition(body.position.x, body.position.y);
      shape.setFillColor(renderBody.color);
      window.draw(shape);
    }

    if (hasDebugFont) {
      window.draw(debugText);
    }

    window.display();
  }

  return 0;
}
