#include <memory>
#include <utility>
#include <vector>

#include "constants/constants.h"
#include "epemstar.hpp"
#include "grid_planning.hpp"
#include "grid_policy.hpp"
#include "mstar_type_defs.hpp"
#include "od_mstar.hpp"

using namespace mstar;

/**
 * Converts from (row, column) coordinates to vertex index
 */
OdCoord to_internal(std::vector<std::pair<int, int>> coord, int cols) {
  std::vector<RobCoord> out;
  for (auto &c : coord) {
    out.push_back(c.first * cols + c.second);
  }
  return OdCoord(out, {});
};

/**
 * Converts from vertex index to (row, column) format
 */
std::vector<std::pair<int, int>> from_internal(OdCoord coord, int cols) {
  std::vector<std::pair<int, int>> out;
  for (auto &c : coord.coord) {
    out.push_back({c / cols, c % cols});
  }
  return out;
};

/**
 * Converts from vertex index to (row, column) format
 */
std::vector<std::pair<int, int>> from_internal(Coord coord, int cols) {
  std::vector<std::pair<int, int>> out;
  for (auto &c : coord) {
    out.push_back({c / cols, c % cols});
  }
  return out;
};

std::vector<std::vector<std::pair<int, int>>> mstar::find_grid_path(
    const std::vector<std::vector<bool>> &obstacles,
    const std::vector<std::pair<int, int>> &init_pos,
    const std::vector<std::pair<int, int>> &goals, double inflation,
    int time_limit) {
  // compute time limit first, as the policies fully compute
  // Need to convert time limit to std::chrono format
  time_point t = std::chrono::system_clock::now();
  t += Clock::duration(std::chrono::seconds(time_limit));

  int cols = (int)obstacles[0].size();
  OdCoord _init = to_internal(init_pos, cols);
  OdCoord _goal = to_internal(goals, cols);
  const auto policy_construct_start = GetMonotonicTime();
  std::vector<std::shared_ptr<Policy>> policies = {};
  for (const auto &goal : goals) {
    policies.push_back(
        std::shared_ptr<Policy>(grid_policy_ptr(obstacles, goal)));
  }
  const auto policy_construct_end = GetMonotonicTime();
  OdMstar planner(policies, _goal, inflation, t,
                  std::shared_ptr<ODColChecker>(new SimpleGraphODColCheck()));
  const auto find_path_start = GetMonotonicTime();
  OdPath path = planner.find_path(_init);
  const auto find_path_end = GetMonotonicTime();
  std::vector<std::vector<std::pair<int, int>>> out;
  for (auto &coord : path) {
    out.push_back(from_internal(coord, cols));
  }
  std::cout << "Individual policy construct time (ms): "
            << (policy_construct_end - policy_construct_start) * 1000 << '\n';
  std::cout << "M* only runtime (ms): "
            << (find_path_end - find_path_start) * 1000 << '\n';
  std::cout << "Total Expansions: " << planner.total_expansions_ << '\n';
  std::cout << "Multiagent expansions: " << planner.multi_agent_expansions_ << '\n';
  return out;
}

std::vector<std::vector<std::pair<int, int>>> mstar::find_grid_epemstar_path(
    const std::vector<std::vector<bool>> &obstacles,
    const std::vector<std::pair<int, int>> &init_pos,
    const std::vector<std::pair<int, int>> &goals, double inflation,
    int time_limit, double increment) {
  // compute time limit first, as the policies fully compute
  // Need to convert time limit to std::chrono format
  time_point t = std::chrono::system_clock::now();
  t += Clock::duration(std::chrono::seconds(time_limit));

  int cols = (int)obstacles[0].size();
  Coord _init = to_internal(init_pos, cols).coord;
  Coord _goal = to_internal(goals, cols).coord;
  std::vector<std::shared_ptr<EPEMstar_Policy>> policies = {};
  for (const auto &goal : goals) {
    policies.push_back(std::shared_ptr<EPEMstar_Policy>(
        epem_grid_policy_ptr(obstacles, goal)));
  }
  EPEMstar planner(policies, _goal, inflation, t, increment,
                   std::shared_ptr<ColChecker>(new SimpleGraphColCheck()));
  Path path = planner.find_path(_init);
  std::vector<std::vector<std::pair<int, int>>> out;
  for (auto &coord : path) {
    out.push_back(from_internal(coord, cols));
  }
  return out;
}
