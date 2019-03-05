#include "simple_planner.hpp"

#include <math.h>

namespace simple_planner {

// Plan:
//   The collection of takeoff positions define the waypoints.
//   Each drone flies through all takeoff positions back to it's own takeoff position, then lands.
//   Example: if there are 4 drones, each drone flies 4 line segments.
//   If the number of drones is less than 3 then additional waypoints are added.
//   Each drone maintains the same orientation throughout the flight.

// Future improvements:
//   Check for crossing paths
//   Check arena bounds
//   Adjust yaw so that that drones can see markers

const double CRUISING_Z = 1.2;
const double SEPARATION = 1.0;

SimplePlanner::SimplePlanner(const std::vector<geometry_msgs::msg::PoseStamped> &landing_poses)
{
  assert(landing_poses.size() > 0);

  // Waypoints are directly above landing poses
  std::vector<geometry_msgs::msg::PoseStamped> waypoints = landing_poses;
  for (auto i = waypoints.begin(); i != waypoints.end(); i++) {
    i->pose.position.z = CRUISING_Z;
  }

  if (waypoints.size() == 1) {
    // Add a 2nd waypoint
    geometry_msgs::msg::PoseStamped p = waypoints[0];
    p.pose.position.y += SEPARATION;
    waypoints.push_back(p);
  }

  if (waypoints.size() == 2) {
    // Add a waypoint to make an equilateral triangle
    geometry_msgs::msg::PoseStamped p = waypoints[0];
    const auto x1 = waypoints[0].pose.position.x;
    const auto y1 = waypoints[0].pose.position.y;
    const auto x2 = waypoints[1].pose.position.x;
    const auto y2 = waypoints[1].pose.position.y;
    p.pose.position.x = (x1 + x2 + sqrt(3) * (y1 - y2)) / 2;
    p.pose.position.y = (y1 + y2 + sqrt(3) * (x2 - x1)) / 2;
    waypoints.push_back(p);
  }

  for (int i = 0; i <= waypoints.size(); i++) {

    // Init path
    nav_msgs::msg::Path path;
    path.header = waypoints[i].header;
    plans_.push_back(path);

    // Copy waypoints
    for (int j = 0; j < waypoints.size(); j++) {
      plans_[i].poses.push_back(waypoints[(i + j) % waypoints.size()]);
    }
    plans_[i].poses.push_back(waypoints[i]);
  }
}

} // namespace simple_planner
