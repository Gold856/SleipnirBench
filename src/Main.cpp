// Copyright (c) Tyler Veness
// Copyright (c) Gold856

#include <benchmark/benchmark.h>
#include <functional>
#include <tuple>
#include <sleipnir/optimization/problem.hpp>
#include <sleipnir/autodiff/variable.hpp>
// #include <print>
constexpr auto ks = 0.20996;
constexpr auto kg = 0.49136;
constexpr auto kv = 0.57408;
constexpr auto ka = 0.020402;
constexpr auto Vmax = 12;
constexpr auto a = 93.333;

constexpr auto A = -kv / ka;
constexpr auto B = 1 / ka;
constexpr auto U1 = Vmax - ks - kg;
constexpr auto U2 = -Vmax + ks - kg;
std::tuple<double, double> current_state = {0.0, 0.0};
std::tuple<double, double> goal_state = {10.0, 0.0};

auto trapezoid_profile(slp::Variable v, std::tuple<double, double> state,
                       double a) {
  return std::get<0>(state) +
         (slp::pow(v, 2) - std::pow(std::get<1>(state), 2)) / (2 * a);
}
auto exponential_profile(slp::Variable v, std::tuple<double, double> state,
                         double A, double B, double U) {
  return std::get<0>(state) - std::get<1>(state) / A + v / A -
         (((B * U) / std::pow(A, 2)) *
          slp::log((A * v + B * U) / (A * std::get<1>(state) + B * U)));
}
auto inflection_point(std::string_view name,
                      std::function<slp::Variable(slp::Variable)> accelerate,
                      std::function<slp::Variable(slp::Variable)> brake,
                      float guess) {
  auto problem = slp::Problem();
  auto v = problem.decision_variable();
  auto difference = brake(v) - accelerate(v);
  problem.minimize(difference * difference);
  v.set_value(guess);

  problem.solve({.tolerance = 1e-3, .diagnostics = false});

  //   std::println("{}: v: {}; x err: {}", name, v.value(),
  //                (brake(v) - accelerate(v)).value());
  return v.value();
}

void Sleipnir_MotionProfile_TrapezoidTrapezoid(benchmark::State& state) {
  for (auto _ : state) {
    inflection_point(
        "trapezoid-trapezoid",
        [=](slp::Variable v) { return trapezoid_profile(v, current_state, a); },
        [=](slp::Variable v) { return trapezoid_profile(v, goal_state, -a); },
        0.5);
  }
}
BENCHMARK(Sleipnir_MotionProfile_TrapezoidTrapezoid);

void Sleipnir_MotionProfile_ExponentialExponential(benchmark::State& state) {
  for (auto _ : state) {
    inflection_point(
        "exponential-exponential",
        [=](slp::Variable v) {
          return exponential_profile(v, current_state, A, B, U1);
        },
        [=](slp::Variable v) {
          return exponential_profile(v, goal_state, A, B, U2);
        },
        0.5);
  }
}
BENCHMARK(Sleipnir_MotionProfile_ExponentialExponential);

void Sleipnir_MotionProfile_TrapezoidExponential(benchmark::State& state) {
  for (auto _ : state) {
    inflection_point(
        "trapezoid-exponential",
        [=](slp::Variable v) { return trapezoid_profile(v, current_state, a); },
        [=](slp::Variable v) {
          return exponential_profile(v, goal_state, A, B, U2);
        },
        0.5);
  }
}
BENCHMARK(Sleipnir_MotionProfile_TrapezoidExponential);

void Sleipnir_MotionProfile_ExponentialTrapezoid(benchmark::State& state) {
  for (auto _ : state) {
    inflection_point(
        "exponential-trapezoid",
        [=](slp::Variable v) {
          return exponential_profile(v, current_state, A, B, U1);
        },
        [=](slp::Variable v) { return trapezoid_profile(v, goal_state, -a); },
        0.5);
  }
}
BENCHMARK(Sleipnir_MotionProfile_ExponentialTrapezoid);
BENCHMARK_MAIN();