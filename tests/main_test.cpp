#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN

#include <cmath>
#include <stdexcept>

#include "boid.hpp"
#include "doctest.h"
#include "simulation.hpp"
#include "statistics.hpp"
#include "vector2d.hpp"

TEST_CASE("Testing Vector2D struct")
{
  bs::Vector2D v1{1.0, 2.0};
  bs::Vector2D v2{3.0, 4.0};

  CHECK((v1 + v2).x_ == doctest::Approx(4.0));
  CHECK((v1 + v2).y_ == doctest::Approx(6.0));

  CHECK((v1 - v2).x_ == doctest::Approx(-2.0));
  CHECK((v1 - v2).y_ == doctest::Approx(-2.0));

  CHECK((v1 * 2.0).x_ == doctest::Approx(2.0));
  CHECK((v1 * 2.0).y_ == doctest::Approx(4.0));

  CHECK((2.0 * v1).x_ == doctest::Approx(2.0));
  CHECK((2.0 * v1).y_ == doctest::Approx(4.0));

  CHECK((v1 / 2.0).x_ == doctest::Approx(0.5));
  CHECK((v1 / 2.0).y_ == doctest::Approx(1.0));

  bs::Vector2D v{10.0, -5.0};
  CHECK_THROWS_AS(v /= 0.0, std::runtime_error);
  CHECK_THROWS_AS(v / 0.0, std::runtime_error);

  bs::Vector2D zero{0.0, 0.0};
  bs::Vector2D v0{5.0, 5.0};

  CHECK(v0.distance(v0) == doctest::Approx(0.0));
  CHECK(zero.norm() == doctest::Approx(0.0));
  CHECK(zero.norm2() == doctest::Approx(0.0));
  CHECK(zero.angle() == doctest::Approx(0.0)); // atan2(0,0), which returns 0

  bs::Vector2D v_right{1.0, 0.0};
  bs::Vector2D v_up{0.0, 1.0};
  bs::Vector2D v_left{-1.0, 0.0};
  bs::Vector2D v_down{0.0, -1.0};

  CHECK(v_right.angle() == doctest::Approx(0.));
  CHECK(v_up.angle() == doctest::Approx(90));
  CHECK(v_left.angle() == doctest::Approx(180));
  CHECK(v_down.angle() == doctest::Approx(-90));
}

TEST_CASE("Testing Boid class")
{
  bs::Boid b({0.0, 0.0}, {0.0, 0.0});
  b.set_max_vel(40.0);
  b.set_min_vel(10.0);
  b.limit_velocity();
  CHECK(b.get_velocity().norm() == doctest::Approx(0.0));

  bs::Boid over_max({0.0, 0.0}, {100.0, 100.0});
  over_max.set_max_vel(40.0);
  over_max.limit_velocity();
  CHECK(over_max.get_velocity().norm() == doctest::Approx(40.0));

  bs::Boid under_min({0.0, 0.0}, {3.0, 4.0});
  under_min.set_min_vel(10.0);
  under_min.limit_velocity();
  CHECK(under_min.get_velocity().norm() == doctest::Approx(10.0));

  bs::Boid b0({100.0, 100.0}, {10.0, 10.0});
  std::vector<bs::Boid> empty_flock;
  std::vector<bs::Boid> single_flock = {b};

  bs::SimConfig test_config;
  test_config.separation_radius = 50.0;
  test_config.separation_factor = 1.0;

  CHECK(b0.compute_separation(empty_flock, test_config).norm()
        == doctest::Approx(0.0));
  CHECK(b0.compute_alignment(empty_flock, test_config).norm()
        == doctest::Approx(0.0));
  CHECK(b0.compute_cohesion(empty_flock, test_config).norm()
        == doctest::Approx(0.0));
  CHECK(b0.compute_separation(single_flock, test_config).norm()
        == doctest::Approx(0.0));
  CHECK(b0.compute_alignment(single_flock, test_config).norm()
        == doctest::Approx(0.0));
  CHECK(b0.compute_cohesion(single_flock, test_config).norm()
        == doctest::Approx(0.0));

  bs::Boid b_overlap1({0.0, 0.0}, {1.0, 1.0});
  bs::Boid b_overlap2({0.0, 0.0}, {-1.0, -1.0});
  std::vector<bs::Boid> overlap_flock = {b_overlap1, b_overlap2};

  bs::Vector2D sep_overlap =
      b_overlap1.compute_separation(overlap_flock, test_config);
  CHECK_FALSE(std::isnan(sep_overlap.x_));
  CHECK_FALSE(std::isnan(sep_overlap.y_));

  b_overlap1.apply_force({0.0, 0.0}, 50.0, 20.0);
  CHECK_FALSE(std::isnan(b_overlap1.get_velocity().x_));
  CHECK_FALSE(std::isnan(b_overlap1.get_velocity().y_));

  bs::Boid b5({0.0, 0.0}, {1.0, 0.0});
  bs::Boid b6({10.0, 0.0}, {0.0, 1.0});
  std::vector<bs::Boid> flock1 = {b5, b6};

  test_config.separation_radius = 20.0;
  test_config.separation_factor = 0.5;
  test_config.cohesion_factor   = 0.1;

  bs::Vector2D sep = flock1[0].compute_separation(flock1, test_config);
  // (b2.pos - b1.pos) = (10, 0); separation = -(10, 0) * 0.5 = (-5, 0)
  CHECK(sep.x_ == doctest::Approx(-5.0));
  CHECK(sep.y_ == doctest::Approx(0.0));

  bs::Vector2D ali = flock1[0].compute_alignment(flock1, test_config);
  // mean_vel = (0, 1); diff = (0, 1) - (1, 0) = (-1, 1); ali = (-0.5, 0.5)
  CHECK(ali.x_ == doctest::Approx(-0.5));
  CHECK(ali.y_ == doctest::Approx(0.5));

  bs::Vector2D coh1 = flock1[0].compute_cohesion(flock1, test_config);
  // center = (10, 0); diff = (10, 0) - (0, 0); coh = (1, 0)
  CHECK(coh1.x_ == doctest::Approx(1.0));
  CHECK(coh1.y_ == doctest::Approx(0.0));

  bs::Boid b_toroidal({1050.0, -10.0}, {0.0, 0.0});

  test_config.border_height = 1000.0;
  test_config.border_width  = 1000.0;

  b_toroidal.apply_toroidal_boundary(test_config);
  CHECK(b_toroidal.get_position().x_ == doctest::Approx(50.0));
  CHECK(b_toroidal.get_position().y_ == doctest::Approx(990.0));

  bs::Boid b_window({950.0, 50.0}, {10.0, -10.0});

  test_config.border_turnfactor = 5.0;
  test_config.margin            = 100.0;

  b_window.apply_window_boundary(test_config);
  CHECK(b_window.get_velocity().x_ == doctest::Approx(5.0));  // 10.0 - 5.0
  CHECK(b_window.get_velocity().y_ == doctest::Approx(-5.0)); // -10.0 + 5.0

  bs::Boid b_F({0.0, 0.0}, {0.0, 0.0});
  bs::Vector2D hunter_pos{10.0, 0.0};
  b_F.apply_force(hunter_pos, 50.0, 20.0);
  // vel -= (hunter - pos) * (strength / dist) -> -(10,0) * (20/10) = (-20, 0)
  CHECK(b_F.get_velocity().x_ == doctest::Approx(-20.0));
  CHECK(b_F.get_velocity().y_ == doctest::Approx(0.0));

  bs::Boid b_F1({10.0, 0.0}, {5.0, 7.0});
  b_F1.apply_force(hunter_pos, 50.0, 20.0);
  CHECK(b_F1.get_velocity().x_ == doctest::Approx(5.0));
  CHECK(b_F1.get_velocity().y_ == doctest::Approx(7.0));

  bs::Boid f1({0.0, 0.0}, {1.0, 0.0});
  bs::Boid f2({10.0, 0.0}, {0.0, 1.0});
  bs::Boid f3({0.0, 10.0}, {-1.0, 0.0});
  std::vector<bs::Boid> big_flock = {f1, f2, f3};

  std::vector<bs::Boid> next_flock = big_flock;

  test_config.visual_range     = 50.0;
  test_config.cohesion_factor  = 0.1;
  test_config.alignment_factor = 0.5;

  // mass_center = (5, 5), c*mass_center = 0.1 * (5, 5) = (0.5, 0.5)
  bs::Vector2D coh_big = next_flock[0].compute_cohesion(big_flock, test_config);
  CHECK(coh_big.x_ == doctest::Approx(0.5));
  CHECK(coh_big.y_ == doctest::Approx(0.5));
  // mean_speed (between f2 and f3) = (-0.5, -0.5), mean_speed - f1.vel = (-1.5,
  // 0.5) -> a*(-1.5, 0.5) = (-0.75, 0.25)
  bs::Vector2D ali_big =
      next_flock[0].compute_alignment(big_flock, test_config);
  CHECK(ali_big.x_ == doctest::Approx(-0.75));
  CHECK(ali_big.y_ == doctest::Approx(0.25));

  std::vector<bs::Boid> flock2 = {bs::Boid({10.0, 10.0}, {0.0, 0.0}),
                                  bs::Boid({12.0, 12.0}, {0.0, 0.0})};

  bs::SimConfig config1;
  config1.visual_range    = 50.0;
  config1.cohesion_factor = 1.0;

  bs::Vector2D coh2 = flock2[0].compute_cohesion(flock2, config1);

  CHECK(coh2.x_ == doctest::Approx(2.0));
  CHECK(coh2.y_ == doctest::Approx(2.0));

  bs::Boid b1({0.0, 0.0}, {1.0, 0.0});
  bs::Boid b2({100.0, 100.0}, {1.0, 0.0});
  std::vector<bs::Boid> flock3 = {b1, b2};

  bs::SimConfig config2;
  config2.visual_range = 0.1;

  CHECK(b1.compute_alignment(flock3, config2).norm() == doctest::Approx(0.0));
  CHECK(b1.compute_cohesion(flock3, config2).norm() == doctest::Approx(0.0));

  bs::SimConfig config;

  config.min_vel       = 2.0;
  config.max_vel       = 10.0;
  config.margin        = 10.0;
  config.border_width  = 1000.0;
  config.border_height = 1000.0;

  bs::Boid b7({500.0, 500.0}, {5.0, 0.0});
  b7.set_min_vel(1.0);
  b7.set_max_vel(10.0);
  std::vector<bs::Boid> flock = {b7};

  bs::Vector2D pos_before = b7.get_position();
  bs::Vector2D vel_before = b7.get_velocity();

  b7.update(flock, config);

  CHECK(b7.get_velocity().x_ == doctest::Approx(vel_before.x_));
  CHECK(b7.get_velocity().y_ == doctest::Approx(vel_before.y_));

  CHECK(b7.get_position().x_
        == doctest::Approx(pos_before.x_ + vel_before.x_ * config.dt));
  CHECK(b7.get_position().y_
        == doctest::Approx(pos_before.y_ + vel_before.y_ * config.dt));

  bs::Boid b_slow({500.0, 500.0}, {0.5, 0.0});
  b_slow.set_min_vel(config.min_vel);
  b_slow.set_max_vel(config.max_vel);
  std::vector<bs::Boid> flock_slow = {b_slow};

  b_slow.update(flock_slow, config);
  CHECK(b_slow.get_velocity().norm() == doctest::Approx(config.min_vel));

  bs::Boid b_fast({500.0, 500.0}, {20.0, 0.0});
  b_fast.set_min_vel(config.min_vel);
  b_fast.set_max_vel(config.max_vel);
  std::vector<bs::Boid> flock_fast = {b_fast};

  b_fast.update(flock_fast, config);
  CHECK(b_fast.get_velocity().norm() == doctest::Approx(config.max_vel));

  bs::Boid b8({500.0, 500.0}, {5.0, 0.0});
  bs::Boid b9({503.0, 500.0}, {-5.0, 0.0});
  std::vector<bs::Boid> flock5 = {b8, b9};

  bs::Vector2D vel_before1 = b8.get_velocity();

  b8.update(flock5, config);

  CHECK_FALSE(b8.get_velocity().x_ == doctest::Approx(vel_before1.x_));
}

TEST_CASE("Testing statistics calculation")
{
  std::vector<bs::Boid> empty_flock;

  auto [avg_s_empty, std_s_empty] = bs::avg_speed(empty_flock);
  CHECK(avg_s_empty == doctest::Approx(0.0));
  CHECK(std_s_empty == doctest::Approx(0.0));

  auto [avg_p_empty, std_p_empty] = bs::avg_position(empty_flock);
  CHECK(avg_p_empty == doctest::Approx(0.0));
  CHECK(std_p_empty == doctest::Approx(0.0));

  bs::Vector2D center_empty = bs::get_flock_center(empty_flock);
  CHECK(center_empty.x_ == doctest::Approx(0.0));
  CHECK(center_empty.y_ == doctest::Approx(0.0));

  bs::Boid b3({3.0, 4.0}, {3.0, 4.0}); // norm = 5.0
  bs::Boid b4({6.0, 8.0}, {6.0, 8.0}); // norm = 10.0
  std::vector<bs::Boid> flock6 = {b3, b4};

  auto [avg_s, std_s] = bs::avg_speed(flock6);
  CHECK(avg_s == doctest::Approx(7.5));
  CHECK(std_s == doctest::Approx(2.5));

  auto [avg_p, std_p] = bs::avg_position(flock6);
  CHECK(avg_p == doctest::Approx(7.5));
  CHECK(std_p == doctest::Approx(2.5));

  bs::Vector2D center = bs::get_flock_center(flock6);
  // ((3+6)/2, (4+8)/2) = (4.5, 6.0)
  CHECK(center.x_ == doctest::Approx(4.5));
  CHECK(center.y_ == doctest::Approx(6.0));

  bs::Boid h1({0.0, 0.0}, {5.0, 0.0});      // norm = 5.0
  bs::Boid h2({0.0, 0.0}, {25.0, 0.0});     // norm = 25.0
  bs::Boid h3({0.0, 0.0}, {49.0, 0.0});     // norm = 49.0
  bs::Boid h_over({0.0, 0.0}, {60.0, 0.0}); // norm = 60.0
  std::vector<bs::Boid> h_flock = {h1, h2, h3, h_over};

  std::vector<int> bins1 = bs::calculate_speed_histogram(h_flock, 5, 50.0);

  REQUIRE(bins1.size() == 5);
  CHECK(bins1[0] == 1); // h1
  CHECK(bins1[1] == 0);
  CHECK(bins1[2] == 1); // h2
  CHECK(bins1[3] == 0);
  CHECK(bins1[4]
        == 2); // h3 + h_over (bin_idx >= num_bins -> bin_idx = num_bins - 1)

  CHECK_THROWS_AS(bs::calculate_speed_histogram(h_flock, 0, 50.0),
                  std::runtime_error);
  CHECK_THROWS_AS(bs::calculate_speed_histogram(h_flock, 5, 0.0),
                  std::runtime_error);
  CHECK_THROWS_AS(bs::calculate_speed_histogram(h_flock, 5, -10.0),
                  std::runtime_error);

  bs::Boid fast({0.0, 0.0}, {50.0, 0.0});
  std::vector<bs::Boid> flock7 = {fast};
  std::vector<int> bins2       = bs::calculate_speed_histogram(flock7, 5, 50.0);

  CHECK(bins2[4] == 1);
  CHECK(bins2[0] == 0);
}

TEST_CASE("Testing SimConfig creation")
{
  bs::SimConfig base_config;
  base_config.is_custom = false;

  bs::SimConfig hunter_cfg = bs::create_hunter_config(base_config);

  CHECK(hunter_cfg.is_hunter == true);
  CHECK(hunter_cfg.n_entities == 3);
  CHECK(hunter_cfg.min_vel == doctest::Approx(40.0));
  CHECK(hunter_cfg.max_vel == doctest::Approx(80.0));
}

TEST_CASE("Testing update_physics simulation loop")
{
  bs::SimConfig config;
  config.min_vel       = 1.0;
  config.max_vel       = 50.0;
  config.dt            = 0.35;
  config.border_width  = 1000.0;
  config.border_height = 1000.0;

  bs::SimConfig h_config = config;

  bs::Boid b1({500.0, 500.0}, {10.0, 0.0});
  b1.set_min_vel(1.0);
  b1.set_max_vel(50.0);

  bs::Boid h1({100.0, 100.0}, {5.0, 5.0});
  h1.set_min_vel(1.0);
  h1.set_max_vel(50.0);

  std::vector<bs::Boid> flock1  = {b1};
  std::vector<bs::Boid> kettle1 = {h1};
  bs::Vector2D mouse_pos_default{0.0, 0.0};

  bs::update_physics(flock1, kettle1, config, h_config, false, false,
                     mouse_pos_default, true);

  double expected_x = 500.0 + (10.0 * config.dt);
  CHECK(flock1[0].get_position().x_ == doctest::Approx(expected_x));
  CHECK_FALSE(kettle1[0].get_position().x_ == doctest::Approx(100.0));

  bs::Boid b_attract({500.0, 500.0}, {0.0, 10.0});
  b_attract.set_min_vel(1.0);
  b_attract.set_max_vel(50.0);

  std::vector<bs::Boid> flock_attract = {b_attract};
  std::vector<bs::Boid> kettle_empty1 = {};
  bs::Vector2D mouse_pos_right{600.0, 500.0};

  bs::update_physics(flock_attract, kettle_empty1, config, h_config, true,
                     false, mouse_pos_right, true);

  CHECK(flock_attract[0].get_velocity().x_ < 0.0);

  bs::Boid b_repel({500.0, 500.0}, {0.0, 10.0});
  b_repel.set_min_vel(1.0);
  b_repel.set_max_vel(50.0);

  std::vector<bs::Boid> flock_repel   = {b_repel};
  std::vector<bs::Boid> kettle_empty2 = {};

  bs::update_physics(flock_repel, kettle_empty2, config, h_config, false, true,
                     mouse_pos_right, true);

  CHECK(flock_repel[0].get_velocity().x_ > 0.0);
}