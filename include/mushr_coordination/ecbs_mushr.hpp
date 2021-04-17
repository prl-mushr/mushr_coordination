#ifndef ECBS_MUSHR_H
#define ECBS_MUSHR_H

#include <iostream>
#include <algorithm>

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include "libMultiRobotPlanning/ecbs_ta.hpp"
#include "libMultiRobotPlanning/next_best_assignment.hpp"
#include "timer.hpp"

using libMultiRobotPlanning::ECBSTA;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using libMultiRobotPlanning::NextBestAssignment;

// TODO: need namespace for the header and separate cpp file for implementation
struct State {
  State(int time, int x, int y, int index) : time(time), x(x), y(y), index(index) {}

  bool operator==(const State& s) const {
    return time == s.time && x == s.x && y == s.y && index == s.index;
  }

  bool equalExceptTime(const State& s) const { return x == s.x && y == s.y; }

  friend std::ostream& operator<<(std::ostream& os, const State& s) {
    return os << s.time << "," << s.index << ": (" << s.x << "," << s.y << ")";
    // return os << "(" << s.x << "," << s.y << ")";
  }

  int time;
  int x;
  int y;
  int index;
};

namespace std {
template <>
struct hash<State> {
  size_t operator()(const State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    boost::hash_combine(seed, s.index);
    return seed;
  }
};
}  // namespace std

///
enum class Action {
  Up,
  Down,
  Left,
  Right,
  Wait,
};

std::ostream& operator<<(std::ostream& os, const Action& a) {
  switch (a) {
    case Action::Up:
      os << "Up";
      break;
    case Action::Down:
      os << "Down";
      break;
    case Action::Left:
      os << "Left";
      break;
    case Action::Right:
      os << "Right";
      break;
    case Action::Wait:
      os << "Wait";
      break;
  }
  return os;
}

///
struct Conflict {
  enum Type {
    Vertex,
    Edge,
  };

  int time;
  size_t agent1;
  size_t agent2;
  Type type;

  int x1;
  int y1;
  int x2;
  int y2;

  friend std::ostream& operator<<(std::ostream& os, const Conflict& c) {
    switch (c.type) {
      case Vertex:
        return os << c.time << ": Vertex(" << c.x1 << "," << c.y1 << ")";
      case Edge:
        return os << c.time << ": Edge(" << c.x1 << "," << c.y1 << "," << c.x2
                  << "," << c.y2 << ")";
    }
    return os;
  }
};

struct VertexConstraint {
  VertexConstraint(int time, int x, int y) : time(time), x(x), y(y) {}
  int time;
  int x;
  int y;

  bool operator<(const VertexConstraint& other) const {
    return std::tie(time, x, y) < std::tie(other.time, other.x, other.y);
  }

  bool operator==(const VertexConstraint& other) const {
    return std::tie(time, x, y) == std::tie(other.time, other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const VertexConstraint& c) {
    return os << "VC(" << c.time << "," << c.x << "," << c.y << ")";
  }
};

namespace std {
template <>
struct hash<VertexConstraint> {
  size_t operator()(const VertexConstraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}  // namespace std

struct EdgeConstraint {
  EdgeConstraint(int time, int x1, int y1, int x2, int y2)
      : time(time), x1(x1), y1(y1), x2(x2), y2(y2) {}
  int time;
  int x1;
  int y1;
  int x2;
  int y2;

  bool operator<(const EdgeConstraint& other) const {
    return std::tie(time, x1, y1, x2, y2) <
           std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
  }

  bool operator==(const EdgeConstraint& other) const {
    return std::tie(time, x1, y1, x2, y2) ==
           std::tie(other.time, other.x1, other.y1, other.x2, other.y2);
  }

  friend std::ostream& operator<<(std::ostream& os, const EdgeConstraint& c) {
    return os << "EC(" << c.time << "," << c.x1 << "," << c.y1 << "," << c.x2
              << "," << c.y2 << ")";
  }
};

namespace std {
template <>
struct hash<EdgeConstraint> {
  size_t operator()(const EdgeConstraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.x1);
    boost::hash_combine(seed, s.y1);
    boost::hash_combine(seed, s.x2);
    boost::hash_combine(seed, s.y2);
    return seed;
  }
};
}  // namespace std

struct Constraints {
  std::unordered_set<VertexConstraint> vertexConstraints;
  std::unordered_set<EdgeConstraint> edgeConstraints;

  void add(const Constraints& other) {
    vertexConstraints.insert(other.vertexConstraints.begin(),
                             other.vertexConstraints.end());
    edgeConstraints.insert(other.edgeConstraints.begin(),
                           other.edgeConstraints.end());
  }

  bool overlap(const Constraints& other) const {
    for (const auto& vc : vertexConstraints) {
      if (other.vertexConstraints.count(vc) > 0) {
        return true;
      }
    }
    for (const auto& ec : edgeConstraints) {
      if (other.edgeConstraints.count(ec) > 0) {
        return true;
      }
    }
    return false;
  }

  friend std::ostream& operator<<(std::ostream& os, const Constraints& c) {
    for (const auto& vc : c.vertexConstraints) {
      os << vc << std::endl;
    }
    for (const auto& ec : c.edgeConstraints) {
      os << ec << std::endl;
    }
    return os;
  }
};

struct Location {
  Location() = default;
  Location(int x, int y) : x(x), y(y) {}
  int x;
  int y;

  bool operator<(const Location& other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  bool operator==(const Location& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const Location& c) {
    return os << "(" << c.x << "," << c.y << ")";
  }
} DUMMY_LOCATION(-1, -1);

namespace std {
template <>
struct hash<Location> {
  size_t operator()(const Location& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    return seed;
  }
};
}// namespace std

struct Waypoints {
  Waypoints() = default;
  Waypoints(std::vector<Location> points) : points(points) {}
  std::vector<Location> points;

  bool operator<(const Waypoints& other) const {
    return points < other.points;
  }

  bool operator==(const Waypoints& other) const {
    return points == other.points;
  }

  friend std::ostream& operator<<(std::ostream& os, const Waypoints& c) {
    for (const auto& p : c.points) {
        os << p << " ";
    }
    return os;
  }
};

namespace std {
template <>
struct hash<Waypoints> {
  size_t operator()(const Waypoints& s) const {
    size_t seed = 0;
    for (const auto& p: s.points) {
      boost::hash_combine(seed, p.x);
      boost::hash_combine(seed, p.y);
    }
    return seed;
  }
};
}// namespace std

#include <shortest_path_heuristic.hpp>

class Environment {
 public:
  Environment(size_t dimx, size_t dimy, size_t numw,
              const std::unordered_set<Location>& obstacles,
              const std::vector<State>& startStates,
              const std::vector<Waypoints>& goals,
              size_t maxTaskAssignments)
      : m_dimx(dimx),
        m_dimy(dimy),
        m_numw(numw), 
        m_obstacles(obstacles),
        m_agentIdx(0),
        m_goal(nullptr),
        m_constraints(nullptr),
        m_lastGoalConstraint(-1),
        m_maxTaskAssignments(maxTaskAssignments),
        m_numTaskAssignments(0),
        m_highLevelExpanded(0),
        m_lowLevelExpanded(0),
        m_heuristic(dimx, dimy, obstacles) {
    std::cout << "init environment" << std::endl;
    m_numAgents = startStates.size();
    for (auto& goal : goals) {
        m_waypoints.push_back(goal);
    }
    for (size_t i = 0; i < startStates.size(); ++i) {
      std::cout << startStates[i] << std::endl;
      for (const auto& goal : goals) {
        //std::cout << goal.points[0] << std::endl;
        int cost = 0;
        if (!goal.points.empty() && goal.points[0].x >= 0) {
          cost = m_heuristic.getValue(Location(startStates[i].x, startStates[i].y), goal.points[startStates[i].index]);
          // std::cout << cost << " ";
          for (size_t j = startStates[i].index; j < numw - 1; ++j) {
            std::cout << goal.points[j] << std::endl;
            cost += m_heuristic.getValue(goal.points[j], goal.points[j + 1]);
            // std::cout << cost << " ";
          }
        } else if (!goal.points.empty()) {
          cost = m_heuristic.getValue(Location(startStates[i].x, startStates[i].y), Location(startStates[i].x, startStates[i].y));
        }
        // std::cout << cost << " " << goal << " " << startStates[i] << std::endl;
        m_assignment.setCost(i, goal, cost);
      }
    }
    m_assignment.solve();
  }

  void setLowLevelContext(size_t agentIdx, const Constraints* constraints,
                          const Waypoints* task) {
    assert(constraints);
    //std::cout << "setLowLevel" << std::endl;
    m_agentIdx = agentIdx;
    m_goal = task;
    m_constraints = constraints;
    m_lastGoalConstraint = -1;
    if (m_goal != nullptr) {
      for (const auto& vc : constraints->vertexConstraints) {
        const Location* l = &(m_goal->points.back()); 
        if (vc.x == l->x && vc.y == l->y) {
          m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
        }
      }
    } else {
      for (const auto& vc : constraints->vertexConstraints) {
        m_lastGoalConstraint = std::max(m_lastGoalConstraint, vc.time);
      }
    }
    //std::cout << "setLLCtx: " << agentIdx << " " << m_lastGoalConstraint << std::endl;
  }

  int admissibleHeuristic(const State& s) {
    //std::cout << "admissibleHeuristic" << std::endl;
    //std::cout << m_goal->points[0] << std::endl;
    if (m_goal != nullptr && s.index < m_numw && m_goal->points[0].x > 0) {
      
      int cost = m_heuristic.getValue(Location(s.x, s.y), m_goal->points[s.index]);
      // std::cout << cost << " ";
      for (size_t i = s.index; i < m_numw - 1; i++) {
        cost += m_heuristic.getValue(m_goal->points[i], m_goal->points[i + 1]);
        // std::cout << cost << " ";
      }
      // std::cout << cost << " " << *m_goal << " " << s << std::endl;
      return cost;
      //return m_heuristic.getValue(Location(s.x, s.y), *m_goal);
    } else {
      std::cout << "exit" << std::endl;
      return 0;
    }
  }

  // low-level
  int focalStateHeuristic(
      const State& s, int /*gScore*/,
      const std::vector<PlanResult<State, Action, int> >& solution) {
    int numConflicts = 0;
    //std::cout << "focalStateHeuristic" << std::endl;
    for (size_t i = 0; i < solution.size(); ++i) {
      if (i != m_agentIdx) {
        if (solution[i].states.size() > 0) { 
          State state2 = getState(i, solution, s.time);
          State p = getPickUp(solution[i], 0);
          if (s.equalExceptTime(state2)) {
            ++numConflicts;
          } else if(s.time < p.time && s.equalExceptTime(p)) {
            ++numConflicts;
          }
        }
      }
    }
    return numConflicts;
  }

  // low-level
  int focalTransitionHeuristic(
      const State& s1a, const State& s1b, int /*gScoreS1a*/, int /*gScoreS1b*/,
      const std::vector<PlanResult<State, Action, int> >& solution) {
    //std::cout << "focalTransitionHeuristic" << std::endl;
    int numConflicts = 0;
    // std::cout << "focaltrans" << std::endl;
    for (size_t i = 0; i < solution.size(); ++i) {
      if (i != m_agentIdx && solution[i].states.size() > 0) {
        State s2a = getState(i, solution, s1a.time);
        State s2b = getState(i, solution, s1b.time);
        if (s1a.equalExceptTime(s2b) && s1b.equalExceptTime(s2a)) { 
          ++numConflicts;
        }
      }
    }
    return numConflicts;
  }

  // Count all conflicts
  int focalHeuristic(
      const std::vector<PlanResult<State, Action, int> >& solution) {
    //std::cout << "focalHeuristic" << std::endl;
    int numConflicts = 0;

    int max_t = 0;
    std::vector<State> pickup; 
    for (size_t i = 0; i < solution.size(); ++i) {
      max_t = std::max<int>(max_t, solution[i].states.size() - 1);
      pickup.push_back(getPickUp(solution[i], 0));
    }

    for (int t = 0; t < max_t; ++t) {
      // check drive-drive vertex collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1 = getState(i, solution, t);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2 = getState(j, solution, t);
          if (state1.equalExceptTime(state2)) {
            ++numConflicts;
          }
        }
      }
      for (size_t i = 0; i < solution.size(); ++i) {
        for (size_t j = 0; j < solution.size(); ++j) {
          if (i != j) {
            State state2 = getState(j, solution, t);
            if (t < pickup[i].time && pickup[i].equalExceptTime(state2)) {
              ++numConflicts;
            }
          }
        }
      }
      // drive-drive edge (swap)
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1a = getState(i, solution, t);
        State state1b = getState(i, solution, t + 1);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2a = getState(j, solution, t);
          State state2b = getState(j, solution, t + 1);
          if (state1a.equalExceptTime(state2b) &&
              state1b.equalExceptTime(state2a)) {
            ++numConflicts;
          }
        }
      }
    }
    return numConflicts;
  }

  bool isSolution(const State& s) {
    return s.index == m_numw && s.time > m_lastGoalConstraint;
  }

  void getNeighbors(const State& s,
                    std::vector<Neighbor<State, Action, int> >& neighbors) {
    // std::cout << "#VC " << constraints.vertexConstraints.size() << std::endl;
    // for(const auto& vc : constraints.vertexConstraints) {
    //   std::cout << "  " << vc.time << "," << vc.x << "," << vc.y <<
    //   std::endl;
    // }
    
    // std::cout << "getNeighbots" << std::endl;
    if (m_goal != nullptr && m_goal->points[0].x < 0) {
      State n(s.time + 1, s.x, s.y, m_numw);
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Wait, 0));
      }
      return;
    }
    const Location *cur = m_goal == nullptr || s.index >= m_numw ? nullptr : &(m_goal->points[s.index]);
    neighbors.clear();
    {
      State n(s.time + 1, s.x, s.y, s.index + ((cur != nullptr && s.x == cur->x && s.y == cur->y) ? 1 : (cur->x == -1 ? m_numw : 0)));
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Wait, (n.index == m_numw) ? 0 : 1));
      }
    }
    {
      State n(s.time + 1, s.x - 1, s.y, s.index + ((cur != nullptr && s.x - 1 == cur->x && s.y == cur->y) ? 1 : 0));
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Left, 1));
      }
    }
    {
      State n(s.time + 1, s.x + 1, s.y, s.index + ((cur != nullptr && s.x + 1 == cur->x && s.y == cur->y) ? 1 : 0));
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Right, 1));
      }
    }
    {
      State n(s.time + 1, s.x, s.y + 1, s.index + ((cur != nullptr && s.x == cur->x && s.y + 1 == cur->y) ? 1 : 0));
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(Neighbor<State, Action, int>(n, Action::Up, 1));
      }
    }
    {
      State n(s.time + 1, s.x, s.y - 1, s.index + ((cur != nullptr && s.x == cur->x && s.y - 1 == cur->y) ? 1 : 0));
      if (stateValid(n) && transitionValid(s, n)) {
        neighbors.emplace_back(
            Neighbor<State, Action, int>(n, Action::Down, 1));
      }
    }
  }

  bool getFirstConflict(
      const std::vector<PlanResult<State, Action, int> >& solution,
      Conflict& result) {
    //std::cout << "getfirstconflict" << std::endl;
    int max_t = 0;
    std::vector<State> pickup; 
    for (const auto& sol : solution) {
      max_t = std::max<int>(max_t, sol.states.size());
      pickup.push_back(getPickUp(sol, 0));
    }
    // std::cout << "pick up" << std::endl;
    // for (const auto& p : pickup) {
    //   std::cout << p << std::endl;
    // }

    for (int t = 0; t < max_t; ++t) {
      // check drive-drive vertex collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1 = getState(i, solution, t);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2 = getState(j, solution, t);
          if (state1.equalExceptTime(state2)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Vertex;
            result.x1 = state1.x;
            result.y1 = state1.y;
            // std::cout << "VC " << t << "," << state1.x << "," << state1.y <<
            // std::endl;
            return true;
          }
        }
      }
      // check slider collisions
      for (size_t i = 0; i < solution.size(); ++i) {
        for (size_t j = 0; j < solution.size(); ++j) {
          if (i != j) {
            State state2 = getState(j, solution, t);
            if (t < pickup[i].time && pickup[i].equalExceptTime(state2)) {
              // std::cout << "box" << t << "," << state2.x << "," << state2.y << std::endl;
              result.time = t;
              result.agent1 = i;
              result.agent2 = j;
              result.type = Conflict::Vertex;
              result.x1 = state2.x;
              result.y1 = state2.y;
              return true;
            }
          }
        }
      }
      // drive-drive edge (swap)
      for (size_t i = 0; i < solution.size(); ++i) {
        State state1a = getState(i, solution, t);
        State state1b = getState(i, solution, t + 1);
        for (size_t j = i + 1; j < solution.size(); ++j) {
          State state2a = getState(j, solution, t);
          State state2b = getState(j, solution, t + 1);
          if (state1a.equalExceptTime(state2b) &&
              state1b.equalExceptTime(state2a)) {
            result.time = t;
            result.agent1 = i;
            result.agent2 = j;
            result.type = Conflict::Edge;
            result.x1 = state1a.x;
            result.y1 = state1a.y;
            result.x2 = state1b.x;
            result.y2 = state1b.y;
            return true;
          }
        }
      }
    }

    return false;
  }

  void createConstraintsFromConflict(
      const Conflict& conflict, std::map<size_t, Constraints>& constraints) {
    //std::cout << "createconstraints" << std::endl;
    if (conflict.type == Conflict::Vertex) {
      Constraints c1;
      c1.vertexConstraints.emplace(
          VertexConstraint(conflict.time, conflict.x1, conflict.y1));
      constraints[conflict.agent1] = c1;
      constraints[conflict.agent2] = c1;
    } else if (conflict.type == Conflict::Edge) {
      Constraints c1;
      c1.edgeConstraints.emplace(EdgeConstraint(
          conflict.time, conflict.x1, conflict.y1, conflict.x2, conflict.y2));
      constraints[conflict.agent1] = c1;
      Constraints c2;
      c2.edgeConstraints.emplace(EdgeConstraint(
          conflict.time, conflict.x2, conflict.y2, conflict.x1, conflict.y1));
      constraints[conflict.agent2] = c2;
    }
  }

  void nextTaskAssignment(std::map<size_t, Waypoints>& tasks) {
    if (m_numTaskAssignments > m_maxTaskAssignments) {
      return;
    }

    int64_t cost = m_assignment.nextSolution(tasks);
    if (!tasks.empty()) {
      std::cout << "nextTaskAssignment: cost: " << cost << std::endl;
      for (const auto& s : tasks) {
        std::cout << s.first << "->" << s.second << std::endl;
      }

      ++m_numTaskAssignments;
    }
  }

  void onExpandHighLevelNode(int /*cost*/) { m_highLevelExpanded++; }

  void onExpandLowLevelNode(const State& /*s*/, int /*fScore*/,
                            int /*gScore*/) {
    m_lowLevelExpanded++;
  }

  int highLevelExpanded() { return m_highLevelExpanded; }

  int lowLevelExpanded() const { return m_lowLevelExpanded; }

  size_t numTaskAssignments() const { return m_numTaskAssignments; }

 private:
  State getState(size_t agentIdx,
                 const std::vector<PlanResult<State, Action, int> >& solution,
                 size_t t) {
    // std::cout << "getState" << std::endl;
    assert(agentIdx < solution.size());
    if (t < solution[agentIdx].states.size()) {
      return solution[agentIdx].states[t].first;
    }
    assert(!solution[agentIdx].states.empty());
    return solution[agentIdx].states.back().first;
  }

  State getPickUp(const PlanResult<State, Action, int>& solution,
                  size_t index) {
    assert(!solution.states.empty());
    // TODO: could be implement in O(log n) using b search
    for (const auto& state: solution.states) {
      if (state.first.index == index + 1) {
        return state.first;
      }
    }
    return solution.states[0].first;
    //std::cerr << "Wrong solution format" << std::endl;
  }

  bool stateValid(const State& s) {
    assert(m_constraints);
    const auto& con = m_constraints->vertexConstraints;
    return s.x >= 0 && s.x < m_dimx && s.y >= 0 && s.y < m_dimy && s.index <= m_numw &&
           m_obstacles.find(Location(s.x, s.y)) == m_obstacles.end() &&
           con.find(VertexConstraint(s.time, s.x, s.y)) == con.end();
  }

  bool transitionValid(const State& s1, const State& s2) {
    assert(m_constraints);
    const auto& con = m_constraints->edgeConstraints;
    return con.find(EdgeConstraint(s1.time, s1.x, s1.y, s2.x, s2.y)) ==
           con.end();
  }

 private:
  int m_dimx;
  int m_dimy;
  int m_numw;
  std::unordered_set<Location> m_obstacles;
  size_t m_agentIdx;
  const Waypoints* m_goal;
  const Constraints* m_constraints;
  int m_lastGoalConstraint;
  NextBestAssignment<size_t, Waypoints> m_assignment;
  size_t m_maxTaskAssignments;
  size_t m_numTaskAssignments;
  int m_highLevelExpanded;
  int m_lowLevelExpanded;
  ShortestPathHeuristic m_heuristic;
  size_t m_numAgents;
  std::vector<Waypoints> m_waypoints;
};

#endif // ECBS_MUSHR_H