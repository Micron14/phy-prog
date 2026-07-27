#include "boid.hpp"
#include "render.hpp"
#include "simconfig.hpp"
#include "simulation.hpp"
#include "statistics.hpp"
#include "vector2d.hpp"

#include <SFML/Graphics.hpp>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>

int main()
{
  try {
    boids::SimConfig config{boids::get_user_config()};
    boids::SimConfig h_config{boids::h_config_setter(config)};

    sf::RenderWindow window(
        sf::VideoMode(static_cast<unsigned int>(config.border_width),
                      static_cast<unsigned int>(config.border_height)),
        "Boids simulation");
    window.setFramerateLimit(60);

    sf::RenderWindow statsWindow;

    auto flock  = boids::entity_gen(config);
    auto kettle = boids::entity_gen(h_config);
    bool is_bounded{true};

    while (window.isOpen()) {
      handle_events(window, statsWindow, is_bounded);

      sf::Vector2i mouse_px{sf::Mouse::getPosition(window)};
      sf::Vector2f mouse_px_corrected =
          window.mapPixelToCoords(mouse_px); // correction to window scale
      phy::Vector2D mouse_pos{static_cast<double>(mouse_px_corrected.x),
                              static_cast<double>(mouse_px_corrected.y)};
      bool left_mouse{sf::Mouse::isButtonPressed(sf::Mouse::Left)};
      bool right_mouse{sf::Mouse::isButtonPressed(sf::Mouse::Right)};

      sim::update_physics(flock, kettle, config, h_config, left_mouse,
                          right_mouse, mouse_pos, is_bounded);

      std::vector<int> speed_histogram{
          stats::calculate_speed_histogram(flock, 25, 50.0)};

      window.clear(sf::Color::White);
      render_flock(window, flock);
      render_kettle(window, kettle);
      window.display();

      if (statsWindow.isOpen()) {
        statsWindow.clear(sf::Color(25, 25, 25));
        sf::FloatRect graph_area(50.f, 50.f, 500.f, 280.f);
        render_histogram(statsWindow, speed_histogram, graph_area, 50.0);
        statsWindow.display();
      }

      static int i{1};
      if (i % 60 == 0) {
        stats::StatResult speed{stats::avg_speed(flock)};
        stats::StatResult position{stats::avg_position(flock)};
        std::cout << "Mean speed: " << speed.mean_
                  << " | Std. Dev.: " << speed.std_dev_
                  << " | Mean position: " << position.mean_
                  << " | Std. Dev.: " << position.std_dev_ << '\n';
      }
      ++i;
    }

  } catch (std::exception const& e) {
    std::cerr << "Fatal Error: " << e.what() << '\n';
    return EXIT_FAILURE;
  } catch (...) {
    std::cerr << "Unknown exception occurred\n";
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}