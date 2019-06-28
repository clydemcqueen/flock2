#include "simple_planner.hpp"

#include <math.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace simple_planner
{

// Plan:
//    The collection of takeoff positions define the waypoints.
//    Each drone flies through all takeoff positions back to it's own takeoff position, then lands.
//    Example: if there are 4 drones, each drone flies 4 line segments.
//    If the number of drones is less than 3 then additional waypoints are added.
//    Each drone maintains the same orientation throughout the flight.

// Timestamps:
//    Mission time starts at node->now().
//    Plan will include a time buffer at each waypoint (TAKEOFF_NS, STABILIZE_NS).
//    Drone must get to waypoint by the indicated timestamp or earlier.
//    Drone should start moving to the first waypoint as soon as the plan is received.
//    Drone should start moving to the next waypoint at the timestamp of the previous waypoint.
//    Drone controller can compute speed from the timestamp with knowledge of the time buffer values.

// Future improvements:
//    Create good plans for a small number of drones (1, 2, 3)
//    Check for crossing paths
//    Check arena bounds
//    Adjust yaw so that that drones can see markers

  const double CRUISING_Z = 1.0;      // m
  const double SEPARATION = 4.0;      // m
  const double SPEED = 0.2;           // m/s

  const rclcpp::Duration TAKEOFF{9000000000};  // includes time to send plan and send takeoff command and takeoff.
  const rclcpp::Duration STABILIZE{9000000000};

#define SUPER_SIMPLE

#ifdef SUPER_SIMPLE
// x, y, z, yaw
  double starting_locations[4][4] = {
    // Face the wall of markers in fiducial.world
//  -2.5,  1.5,  1.0,  0.0,
//  -1.5,  0.5,  1.0,  0.785,
//  -0.5,  1.5,  1.0,  0.0,
//  -1.5,  2.5,  1.0, -0.785

    -1.0, 1.0, 1.0, 0.0,
    -1.0, 0.5, 1.0, 0.0,
    -1.0, 0.0, 1.0, 0.0,
    -1.0, -0.5, 1.0, -0.0

    // Face all 4 directions in f2.world
    // -2.5,  1.5,  1.0,  0.0,
    // -1.5,  0.5,  1.0,  1.57,
    // -0.5,  1.5,  1.0,  3.14,
    // -1.5,  3.0,  1.0, -1.57
  };
#endif

  double distance(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2)
  {
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2) + pow(p2.z - p1.z, 2));
  }

  SimplePlanner::SimplePlanner(const std::vector<geometry_msgs::msg::PoseStamped> &landing_poses)
  {
    assert(landing_poses.size() > 0);

    num_drones_ = landing_poses.size();

    // Waypoints are directly above landing poses
    waypoints_ = landing_poses;
    for (auto i = waypoints_.begin(); i != waypoints_.end(); i++) {
      i->pose.position.z = CRUISING_Z;
    }

#ifdef SUPER_SIMPLE
    for (int i = waypoints_.size(); i < 4; i++) {
      geometry_msgs::msg::PoseStamped p = waypoints_[0];
      p.pose.position.x = starting_locations[i][0];
      p.pose.position.y = starting_locations[i][1];
      p.pose.position.z = starting_locations[i][2];
      tf2::Quaternion q;
      q.setRPY(0, 0, starting_locations[i][3]);
      p.pose.orientation = tf2::toMsg(q);
      waypoints_.push_back(p);
    }
#else
    if (waypoints_.size() == 1) {
      // Add a 2nd waypoint
      geometry_msgs::msg::PoseStamped p = waypoints_[0];
      p.pose.position.y += SEPARATION;
      waypoints_.push_back(p);
    }

    if (waypoints_.size() == 2) {
      // Add a 3rd waypoint to make an equilateral triangle
      geometry_msgs::msg::PoseStamped p = waypoints_[0];
      const auto x1 = waypoints_[0].pose.position.x;
      const auto y1 = waypoints_[0].pose.position.y;
      const auto x2 = waypoints_[1].pose.position.x;
      const auto y2 = waypoints_[1].pose.position.y;
      p.pose.position.x = (x1 + x2 + sqrt(3) * (y1 - y2)) / 2;
      p.pose.position.y = (y1 + y2 + sqrt(3) * (x2 - x1)) / 2;
      waypoints_.push_back(p);
    }
#endif
  }

  std::vector<nav_msgs::msg::Path> SimplePlanner::plans(const rclcpp::Time &now) const
  {
    std::vector<nav_msgs::msg::Path> plans;

    // Compute time to fly from this waypoint to next waypoint
    // Assume instant acceleration and constant speed
    std::vector<rclcpp::Duration> flight_time;
    for (int i = 0; i < waypoints_.size(); i++) {
      int next = (i + 1) % waypoints_.size();
      flight_time.emplace_back(distance(waypoints_[next].pose.position, waypoints_[i].pose.position) / SPEED * 1e9);
    }

    // Create num_drones_ plans, each with waypoints_.size() waypoints_
    for (int i = 0; i < num_drones_; i++) {
      // Init path
      nav_msgs::msg::Path path;
      path.header.stamp = now;
      plans.push_back(path);

      // Timestamp for waypoint 0
      rclcpp::Time timestamp = now + TAKEOFF;

      // Add waypoints to path
      for (int j = 0; j < waypoints_.size(); j++) {
        int curr = (i + j) % waypoints_.size();
        plans[i].poses.push_back(waypoints_[curr]);
        plans[i].poses[j].header.stamp = timestamp;

        // Timestamp for waypoint j + 1
        timestamp = timestamp + flight_time[curr];
      }

      // Last waypoint returns to the spot just above the landing pose
      plans[i].poses.push_back(waypoints_[i]);
      plans[i].poses[waypoints_.size()].header.stamp = timestamp;
    }

    return plans;
  }

} // namespace simple_planner
