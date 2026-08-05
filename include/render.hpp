#ifndef RENDER_HPP
#define RENDER_HPP

#include "boid.hpp"

#include <SFML/Graphics.hpp>
#include <vector>

namespace boids {

void render_flock(sf::RenderWindow& window,
                  std::vector<Boid> const& flock);

void render_kettle(sf::RenderWindow& window,
                   std::vector<Boid> const& kettle);

void render_histogram(sf::RenderWindow& window, std::vector<int> const& bins,
                      sf::FloatRect const& bounds, double max_val);

void handle_events(sf::RenderWindow& window, sf::RenderWindow& statsWindow,
                   bool& is_bounded);

} // namespace boids
#endif