#include "simple_planner.hpp"

#include <math.h>

namespace simple_planner {

// Plan:
//    The collection of takeoff positions define the waypoints.
//    Each drone flies through all takeoff positions back to it's own takeoff position, then lands.
//    Example: if there are 4 drones, each drone flies 4 line segments.
//    If the number of drones is less than 3 then additional waypoints are added.
//    Each drone maintains the same orientation throughout the flight.

// Timestamps:
//    Mission time starts at 0.
//    Plan will include a time buffer at each waypoint (TAKEOFF_NS, STABILIZE_NS).
//    Drone must get to waypoint by the indicated timestamp, or earlier.
//    Drone should start moving to the next waypoint at the indicated timestamp.
//    Drone controller can compute speed from the timestamp with knowledge of the time buffer values.

// Future improvements:
//    Check for crossing paths
//    Check arena bounds
//    Adjust yaw so that that drones can see markers

const double CRUISING_Z = 1.2;    // m
const double SEPARATION = 4.0;    // m
const double SPEED = 0.2;         // m/s
const int64_t TAKEOFF_NS = 5e9;   // ns
const int64_t STABILIZE_NS = 3e9; // ns

#define SUPER_SIMPLE

double distance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
{
  return sqrt(pow(p2.x - p1.y, 2) + pow(p2.y - p1.y, 2) + pow(p2.z - p1.z, 2));
}

SimplePlanner::SimplePlanner(const std::vector<geometry_msgs::msg::PoseStamped> &landing_poses)  // TODO just need Pose
{
#ifdef SUPER_SIMPLE
  assert(landing_poses.size() > 2);
#else
  assert(landing_poses.size() > 0);
#endif

  // Waypoints are directly above landing poses
  std::vector<geometry_msgs::msg::PoseStamped> waypoints = landing_poses;
  for (auto i = waypoints.begin(); i != waypoints.end(); i++) {
    i->pose.position.z = CRUISING_Z;
  }

#ifndef SUPER_SIMPLE
  if (waypoints.size() == 1) {
    // Add a 2nd waypoint
    geometry_msgs::msg::PoseStamped p = waypoints[0];
    p.pose.position.y += SEPARATION;
    waypoints.push_back(p);
  }

  if (waypoints.size() == 2) {
    // Add a 3rd waypoint to make an equilateral triangle
    geometry_msgs::msg::PoseStamped p = waypoints[0];
    const auto x1 = waypoints[0].pose.position.x;
    const auto y1 = waypoints[0].pose.position.y;
    const auto x2 = waypoints[1].pose.position.x;
    const auto y2 = waypoints[1].pose.position.y;
    p.pose.position.x = (x1 + x2 + sqrt(3) * (y1 - y2)) / 2;
    p.pose.position.y = (y1 + y2 + sqrt(3) * (x2 - x1)) / 2;
    waypoints.push_back(p);
  }
#endif

  int num_drones = landing_poses.size();
  int num_waypoints = waypoints.size();

  // Compute time to fly from this waypoint to next waypoint
  // Assume instant acceleration and constant speed
  std::vector<int64_t> flight_time_ns;
  for (int i = 0; i < num_waypoints; i++) {
    int next = (i + 1) % num_waypoints;
    flight_time_ns.push_back(distance(waypoints[next].pose.position, waypoints[i].pose.position) / SPEED * 1e9);
  }

  // Create num_drones plans, each with num_waypoints waypoints
  for (int i = 0; i < num_drones; i++) {
    // Init path
    nav_msgs::msg::Path path;
    path.header = waypoints[i].header;  // TODO necessary? Could emplace_back()
    plans_.push_back(path);

    // Mission time starts at 0, add time for takeoff
    rclcpp::Time timestamp(TAKEOFF_NS);

    for (int j = 0; j < num_waypoints; j++) {
      int curr = (i + j) % num_waypoints;
      plans_[i].poses.push_back(waypoints[curr]);
      plans_[i].poses[j].header.stamp = timestamp;

      // Add time to get to next waypoint, plus some extra
      timestamp = timestamp + rclcpp::Duration(flight_time_ns[curr] + STABILIZE_NS);
    }
    plans_[i].poses.push_back(waypoints[i]);
    plans_[i].poses[num_waypoints].header.stamp = timestamp;
  }
}

} // namespace simple_planner
