#include "simconfig.hpp"

#include <ios>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace boids {

SimConfig get_user_config()
{
  SimConfig config;

  char choice;
  while (true) {
    std::cout << "Would you like to start the boid simulation with the default "
                 "parameters? (y/n): ";
    std::cin >> choice;
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    if (choice == 'y' || choice == 'Y') {
      std::cout << "Starting with the default parameters...\n";
      return config;
    } else if (choice == 'n' || choice == 'N') {
      config.is_custom = true;
      break;
    } else {
      std::cout << "Not valid.\n";
    }
  }

  std::cout << "\n--- Custom Parameter Input ---\n";

  std::cout << "Enter number of boids: ";
  if (!(std::cin >> config.n_entities) || config.n_entities <= 0) {
    throw std::runtime_error("Invalid input: The number of boids must be an "
                             "integer greater than zero.");
  }

  std::cout << "Enter the visual range (d): ";
  if (!(std::cin >> config.visual_range) || config.visual_range <= 0) {
    throw std::runtime_error("Invalid input: The visual range must be an "
                             "integer greater than zero.");
  }

  std::cout << "Enter the separation radius (ds): ";
  if (!(std::cin >> config.separation_radius)
      || config.separation_radius <= 0) {
    throw std::runtime_error("Invalid input: The separation radius must be an "
                             "integer greater than zero.");
  }

  std::cout << "Enter the separation factor (s): ";
  if (!(std::cin >> config.separation_factor) || config.separation_factor < 0) {
    throw std::runtime_error(
        "Invalid input: The separation strenght cannot be negative.");
  }

  std::cout << "Enter the alignment factor (a): ";
  if (!(std::cin >> config.alignment_factor) || config.alignment_factor < 0) {
    throw std::runtime_error(
        "Invalid input: The alignment factor cannot be negative.");
  }

  std::cout << "Enter the cohesion factor (c): ";
  if (!(std::cin >> config.cohesion_factor) || config.cohesion_factor < 0) {
    throw std::runtime_error(
        "Invalid input: The cohesion factor cannot be negative.");
  }

  return config;
}

SimConfig h_config_setter(SimConfig const& config)
{
  SimConfig h_config{config};
  if (config.is_custom) {
    std::cout << "Enter number of hunters: ";
    if (!(std::cin >> h_config.n_entities) || h_config.n_entities < 0) {
      throw std::runtime_error(
          "Invalid input: The number of hunters cannot be negative");
    }
  } else {
    h_config.n_entities = 3;
  }
  h_config.is_hunter = true;
  h_config.separation_factor *= 0.5;
  h_config.separation_radius *= 3;
  h_config.alignment_factor = 0;
  h_config.cohesion_factor  = 0;
  h_config.min_vel          = 40.0;
  h_config.max_vel          = 80.0;
  return h_config;
}

} // namespace boids