#ifndef RENDER_HPP
#define RENDER_HPP

#include "boid.hpp"

#include <SFML/Graphics.hpp>
#include <vector>

namespace bs {

class Assets
{
 private:
  sf::ConvexShape boid_shape_;
  sf::ConvexShape hunter_shape_;
  sf::Font font_;
  sf::RectangleShape bar_;
  sf::Text text_;
  sf::Text label_;

  void draw_entity(sf::RenderWindow& window, sf::ConvexShape& shape,
                   Boid const& entity);

 public:
  Assets();

  void draw_boid(sf::RenderWindow& window, Boid const& boid);
  void draw_hunter(sf::RenderWindow& window, Boid const& hunter);

  void draw_histogram(sf::RenderWindow& window, std::vector<int> const& bins,
                      sf::FloatRect const& bounds, double max_val);
};

void render_flock(sf::RenderWindow& window, std::vector<Boid> const& flock,
                  Assets& assets);

void render_kettle(sf::RenderWindow& window, std::vector<Boid> const& kettle,
                   Assets& assets);

void handle_events(sf::RenderWindow& window, sf::RenderWindow& statsWindow,
                   bool& is_bounded);

} // namespace bs
#endif