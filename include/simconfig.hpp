#ifndef SIM_CONFIG_HPP
#define SIM_CONFIG_HPP

namespace boids {

struct SimConfig
{
  bool is_custom{false};
  int n_entities{300};
  bool is_hunter{false};
  double border_width{1200.0};
  double border_height{900.0};
  double dt{0.35};
  double border_turnfactor{5.0};
  double margin{100.0};
  double visual_range{60.0};
  double separation_radius{20.0};
  double separation_factor{0.8};
  double alignment_factor{0.5};
  double cohesion_factor{0.01};
  double influence_radius{200.0};
  double interaction_strength{8.0};
  double min_vel{20.0};
  double max_vel{35.0};
};

SimConfig get_user_config();
SimConfig h_config_setter(SimConfig const& config);

} // namespace boids
#endif