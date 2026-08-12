#include "render.hpp"
#include "boid.hpp"

#include <SFML/Graphics.hpp>
#include <algorithm>
#include <cstddef>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace bs {

Assets::Assets()
{
  boid_shape_.setPointCount(3);
  boid_shape_.setPoint(0, sf::Vector2f(6.f, 0.f));
  boid_shape_.setPoint(1, sf::Vector2f(-8.f, -6.f));
  boid_shape_.setPoint(2, sf::Vector2f(-8.f, 6.f));
  boid_shape_.setFillColor(sf::Color::Red);

  hunter_shape_.setPointCount(3);
  hunter_shape_.setPoint(0, sf::Vector2f(10.f, 0.f));
  hunter_shape_.setPoint(1, sf::Vector2f(-12.f, -8.f));
  hunter_shape_.setPoint(2, sf::Vector2f(-12.f, 8.f));
  hunter_shape_.setFillColor(sf::Color::Cyan);

  if (!font_.loadFromFile("font/arial.ttf")) {
    throw std::runtime_error("Can't load font 'arial.ttf'");
  }

  bar_.setFillColor(sf::Color(70, 130, 180));

  text_.setFont(font_);
  text_.setCharacterSize(12);
  text_.setFillColor(sf::Color::White);

  label_.setFont(font_);
  label_.setCharacterSize(10);
  label_.setFillColor(sf::Color::White);
}

sf::ConvexShape& Assets::get_boid_shape()
{
  return boid_shape_;
}
sf::ConvexShape& Assets::get_hunter_shape()
{
  return hunter_shape_;
}
sf::RectangleShape& Assets::get_bar()
{
  return bar_;
}
sf::Text& Assets::get_text()
{
  return text_;
}
sf::Text& Assets::get_label()
{
  return label_;
}

void Assets::draw_entity(sf::RenderWindow& window, sf::ConvexShape& shape,
                         Boid const& entity)
{
  float x_pos{static_cast<float>(entity.get_position().x_)};
  float y_pos{static_cast<float>(entity.get_position().y_)};
  shape.setPosition(x_pos, y_pos);

  float angle{static_cast<float>(entity.get_velocity().angle())};
  shape.setRotation(angle);

  window.draw(shape);
}

void render_flock(sf::RenderWindow& window, std::vector<Boid> const& flock,
                  Assets& assets)
{
  for (auto const& boid : flock) {
    assets.draw_entity(window, assets.get_boid_shape(), boid);
  }
}

void render_kettle(sf::RenderWindow& window, std::vector<Boid> const& kettle,
                   Assets& assets)
{
  for (auto const& hunter : kettle) {
    assets.draw_entity(window, assets.get_hunter_shape(), hunter);
  }
}

void render_histogram(sf::RenderWindow& window, std::vector<int> const& bins,
                      sf::FloatRect const& bounds, double max_val,
                      Assets& assets)
{
  if (bins.empty()) {
    return;
  }

  int max_count{*std::max_element(bins.begin(), bins.end())};
  if (max_count == 0) {
    max_count = 1;
  }

  float bin_width{bounds.width / static_cast<float>(bins.size())};

  sf::RectangleShape& bar{assets.get_bar()};
  sf::Text& text{assets.get_text()};
  sf::Text& label{assets.get_label()};

  for (size_t i{0}; i < bins.size(); ++i) {
    float normalized_height{
        (static_cast<float>(bins[i]) / static_cast<float>(max_count))
        * bounds.height};
    float x_pos{bounds.left + static_cast<float>(i) * bin_width};
    float y_pos{bounds.top + bounds.height - normalized_height};

    bar.setSize(sf::Vector2f(bin_width - 1.f, normalized_height));
    bar.setPosition(x_pos, y_pos);
    window.draw(bar);

    if (bins[i] > 0) {
      text.setString(std::to_string(bins[i]));
      sf::FloatRect textRect{text.getLocalBounds()};
      text.setOrigin(textRect.left + textRect.width / 2.0f,
                     textRect.top + textRect.height / 2.0f);
      text.setPosition(x_pos + bin_width / 2.0f, y_pos - 10.f);
      window.draw(text);

      float v_min{(static_cast<float>(i) / static_cast<float>(bins.size()))
                  * static_cast<float>(max_val)};

      if (i % 5 == 0) {
        label.setString(std::to_string(static_cast<int>(v_min)));
        sf::FloatRect labelRect{label.getLocalBounds()};
        label.setOrigin(labelRect.left + labelRect.width / 2.0f,
                        labelRect.top + labelRect.height / 2.0f);
        label.setPosition(x_pos + bin_width / 2.0f,
                          bounds.top + bounds.height + 10.f);
        window.draw(label);
      }
    }
  }

  bar.setOutlineThickness(-1.f);
  bar.setOutlineColor(sf::Color(40, 40, 40));

  sf::RectangleShape xAxis{sf::Vector2f(bounds.width, 2.f)};
  xAxis.setPosition(bounds.left, bounds.top + bounds.height);
  xAxis.setFillColor(sf::Color::White);
  window.draw(xAxis);
}

void handle_events(sf::RenderWindow& window, sf::RenderWindow& statsWindow,
                   bool& is_bounded)
{
  sf::Event event;

  while (window.pollEvent(event)) {
    if (event.type == sf::Event::Closed) {
      window.close();
      if (statsWindow.isOpen())
        statsWindow.close();
    }

    if (event.type == sf::Event::KeyPressed) {
      if (event.key.code == sf::Keyboard::B) {
        is_bounded = !is_bounded;

        if (is_bounded) {
          std::cout << "Boundary: Limited Window.\n";
        } else {
          std::cout << "Boundary: Toroidal Space.\n";
        }
      }

      if (event.key.code == sf::Keyboard::S) {
        if (statsWindow.isOpen()) {
          statsWindow.close();
        } else {
          statsWindow.create(sf::VideoMode(600, 400), "Speed distribution");
          statsWindow.setFramerateLimit(60);
        }
      }
    }
  }

  if (statsWindow.isOpen()) {
    while (statsWindow.pollEvent(event)) {
      if (event.type == sf::Event::Closed) {
        statsWindow.close();
      }
      if (event.type == sf::Event::KeyPressed) {
        if (event.key.code == sf::Keyboard::S) {
          statsWindow.close();
        }
      }
    }
  }
}

} // namespace bs