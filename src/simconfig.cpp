#include "simconfig.hpp"

#include <ios>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>

namespace bs {

void clear_cin_buffer() // static 
{
  std::cin.clear();
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

template<typename T, typename Predicate>
T read_param(std::string const& prompt, std::string const& error_msg,
             Predicate is_valid) // static 
{
  T value{};
  std::cout << prompt;
  if (!(std::cin >> value) || !is_valid(value)) {
    clear_cin_buffer();
    throw std::runtime_error(error_msg);
  }
  return value;
}

SimConfig get_user_config()
{
  SimConfig config;

  char choice;
  while (true) {
    std::cout << "Would you like to start the boid simulation with the default "
                 "parameters? (y/n): ";
    if (std::cin >> choice) {
      clear_cin_buffer();
      if (choice == 'y' || choice == 'Y') {
        std::cout << "Starting with the default parameters...\n";
        return config;
      }
      if (choice == 'n' || choice == 'N') {
        config.is_custom = true;
        break;
      }
      std::cout << "Not valid. Please enter 'y' or 'n'.\n";
    } else {
      clear_cin_buffer();
      std::cout << "Input error. Please try again.\n";
    }
  }

  std::cout << "\n--- Custom Parameter Input ---\n";

  config.n_entities = read_param<int>("Enter number of boids: ",
                                      "Invalid input: The number of boids must "
                                      "be an integer greater than zero.",
                                      [](int v) { return v > 0; });

  config.visual_range = read_param<double>(
      "Enter the visual range (d): ",
      "Invalid input: The visual range must be a positive number.",
      [](double v) { return v > 0.0; });

  config.separation_radius = read_param<double>(
      "Enter the separation radius (ds): ",
      "Invalid input: The separation radius must be a positive number.",
      [](double v) { return v > 0.0; });

  if (config.separation_radius > config.visual_range) {
    throw std::runtime_error("Invalid input: The separation radius cannot be "
                             "greater than the visual range.");
  }

  config.separation_factor = read_param<double>(
      "Enter the separation factor (s): ",
      "Invalid input: The separation strength cannot be negative.",
      [](double v) { return v >= 0.0; });

  config.alignment_factor = read_param<double>(
      "Enter the alignment factor (a): ",
      "Invalid input: The alignment factor cannot be negative.",
      [](double v) { return v >= 0.0; });

  config.cohesion_factor = read_param<double>(
      "Enter the cohesion factor (c): ",
      "Invalid input: The cohesion factor cannot be negative.",
      [](double v) { return v >= 0.0; });

  return config;
}

SimConfig create_hunter_config(SimConfig h_config)
{
  if (h_config.is_custom) {
    h_config.n_entities = read_param<int>(
        "Enter number of hunters: ",
        "Invalid input: The number of hunters cannot be negative.",
        [](int v) { return v >= 0; });
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

} // namespace bs