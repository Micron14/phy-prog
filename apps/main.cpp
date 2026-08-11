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
    bs::SimConfig config{bs::get_user_config()};
    bs::SimConfig h_config{bs::h_config_setter(config)};

    sf::RenderWindow window(
        sf::VideoMode(static_cast<unsigned int>(config.border_width),
                      static_cast<unsigned int>(config.border_height)),
        "Boids simulation");
    window.setFramerateLimit(60);

    sf::RenderWindow statsWindow;
    bs::Assets assets;

    auto flock  = bs::entity_gen(config);
    auto kettle = bs::entity_gen(h_config);
    bool is_bounded{true};
    int tic{1};

    while (window.isOpen()) {
      bs::handle_events(window, statsWindow, is_bounded);

      sf::Vector2i mouse_px{sf::Mouse::getPosition(window)};
      sf::Vector2f mouse_px_corrected =
          window.mapPixelToCoords(mouse_px); // correction to window scale
      bs::Vector2D mouse_pos{static_cast<double>(mouse_px_corrected.x),
                             static_cast<double>(mouse_px_corrected.y)};
      bool left_mouse{sf::Mouse::isButtonPressed(sf::Mouse::Left)};
      bool right_mouse{sf::Mouse::isButtonPressed(sf::Mouse::Right)};

      bs::update_physics(flock, kettle, config, h_config, left_mouse,
                         right_mouse, mouse_pos, is_bounded);

      std::vector<int> speed_histogram{
          bs::calculate_speed_histogram(flock, 25, 50.0)};

      window.clear(sf::Color::White);
      bs::render_flock(window, flock, assets);
      bs::render_kettle(window, kettle, assets);
      window.display();

      if (statsWindow.isOpen()) {
        statsWindow.clear(sf::Color(25, 25, 25));
        sf::FloatRect graph_area(50.f, 50.f, 500.f, 280.f);
        bs::render_histogram(statsWindow, speed_histogram, graph_area, 50.0,
                             assets);
        statsWindow.display();
      }

      if (tic % 60 == 0) {
        bs::StatResult speed{bs::avg_speed(flock)};
        bs::StatResult position{bs::avg_position(flock)};
        std::cout << "Mean speed: " << speed.mean_
                  << " | Std. Dev.: " << speed.std_dev_
                  << " | Mean position: " << position.mean_
                  << " | Std. Dev.: " << position.std_dev_ << '\n';
      }
      ++tic;
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