#include "drone.hpp"

#include "flock_base.hpp"

namespace flock_base {

// rclcpp::Time t() initializes nanoseconds to 0
inline bool valid(rclcpp::Time &t) { return t.nanoseconds() > 0; }

const rclcpp::Duration FLIGHT_DATA_TIMEOUT{4};  // We stopped receiving telemetry
const rclcpp::Duration ODOM_TIMEOUT{4};         // We stopped receiving odometry

std::map<State, const char *> g_states{
  {State::unknown, "unknown"},
  {State::landed, "landed"},
  {State::flying, "flying"},
};

std::map<Action, const char *> g_actions{
  {Action::takeoff, "takeoff"},
  {Action::land, "land"},
  {Action::connect, "connect"},
  {Action::disconnect, "disconnect"},
};

struct Transition
{
  State curr_state_;
  Action action_;
  State next_state_;
  bool send_to_drone_;

  Transition(State curr_state, Action action, State next_state, bool send_to_drone):
    curr_state_{curr_state}, action_{action}, next_state_{next_state}, send_to_drone_{send_to_drone}
  {}
};

bool valid_transition(const State state, const Action action, State &next_state, bool &internal)
{
  const static std::vector<Transition> valid_transitions{
    // Connect / disconnect
    Transition{State::unknown, Action::connect, State::landed, false},
    Transition{State::landed, Action::disconnect, State::unknown, false},
    Transition{State::flying, Action::disconnect, State::unknown, false},

    // Take off / land
    Transition{State::landed, Action::takeoff, State::flying, true},
    Transition{State::flying, Action::land, State::landed, true},
  };

  for (auto i = valid_transitions.begin(); i != valid_transitions.end(); i++) {
    if (i->curr_state_ == state && i->action_ == action) {
      next_state = i->next_state_;
      internal = i->send_to_drone_;
      return true;
    }
  }

  return false;
}

Drone::Drone(FlockBase *node, std::string ns) : node_{node}, ns_{ns}
{
  std::string pre = ns_.empty() ? "" : ns_ + "/";

  action_mgr_ = std::make_unique<ActionMgr>(ns_, node_->get_logger(),
    node_->create_client<tello_msgs::srv::TelloAction>(pre + "tello_action"));

  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(pre + "cmd_vel", 1);

  auto tello_response_cb = std::bind(&Drone::tello_response_callback, this, std::placeholders::_1);
  auto flight_data_cb = std::bind(&Drone::flight_data_callback, this, std::placeholders::_1);
  auto odom_cb = std::bind(&Drone::odom_callback, this, std::placeholders::_1);

  tello_response_sub_ = node_->create_subscription<tello_msgs::msg::TelloResponse>(pre + "tello_response", tello_response_cb);
  flight_data_sub_ = node_->create_subscription<tello_msgs::msg::FlightData>(pre + "flight_data", flight_data_cb);
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(pre + "filtered_odom", odom_cb);

  if (ns_.empty()) {
    RCLCPP_INFO(node_->get_logger(), "flock_base ready");
  } else {
    RCLCPP_INFO(node_->get_logger(), "flock_base %s ready", ns_.c_str());
  }
}

void Drone::start_action(Action action)
{
  if (action_mgr_->busy()) {
    RCLCPP_DEBUG(node_->get_logger(), "%s: busy, dropping %s", ns_.c_str(), g_actions[action]);
    return;
  }

  State next_state;
  bool send_to_drone;
  if (!valid_transition(state_, action, next_state, send_to_drone)) {
    RCLCPP_DEBUG(node_->get_logger(), "%s: %s not allowed in %s", ns_.c_str(), g_actions[action], g_states[state_]);
    return;
  }

  if (send_to_drone) {
    RCLCPP_INFO(node_->get_logger(), "%s: initiating %s", ns_.c_str(), g_actions[action]);
    action_mgr_->send(action, g_actions[action]);
  } else {
    transition_state(action);
  }
}

void Drone::transition_state(Action action)
{
  State next_state;
  bool send_to_drone;
  if (!valid_transition(state_, action, next_state, send_to_drone)) {
    RCLCPP_DEBUG(node_->get_logger(), "%s: %s not allowed in %s", ns_.c_str(), g_actions[action], g_states[state_]);
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "%s: transition to %s", ns_.c_str(), g_states[next_state]);
  state_ = next_state;
}

void Drone::tello_response_callback(tello_msgs::msg::TelloResponse::SharedPtr msg)
{
  ActionMgr::State result = action_mgr_->complete(msg);
  if (result == ActionMgr::State::succeeded) {
    transition_state(action_mgr_->action());
  } else if (result == ActionMgr::State::failed_lost_connection) {
    transition_state(Action::disconnect);
  }
}

void Drone::flight_data_callback(tello_msgs::msg::FlightData::SharedPtr msg)
{
  prev_flight_data_stamp_ = msg->header.stamp;

  if (state_ == State::unknown) {
    RCLCPP_INFO(node_->get_logger(), "%s: receiving flight data", ns_.c_str());
    transition_state(Action::connect);
  }
}

void Drone::odom_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  if (!valid(prev_odom_stamp_)) {
    RCLCPP_INFO(node_->get_logger(), "%s: receiving odom", ns_.c_str());
    // TODO
  }

  prev_odom_stamp_ = msg->header.stamp;
}

void Drone::set_velocity(double throttle, double strafe, double vertical, double yaw)
{
  twist_.linear.x = throttle;
  twist_.linear.y = strafe;
  twist_.linear.z = vertical;
  twist_.angular.z = yaw;
}

void Drone::spin_once()
{
  if (valid(prev_flight_data_stamp_) && node_->now() - prev_flight_data_stamp_ > FLIGHT_DATA_TIMEOUT) {
    RCLCPP_ERROR(node_->get_logger(), "%s: lost flight data", ns_.c_str());
    prev_flight_data_stamp_ = rclcpp::Time();
    // TODO
  }

  if (valid(prev_odom_stamp_) && node_->now() - prev_odom_stamp_ > ODOM_TIMEOUT) {
    RCLCPP_ERROR(node_->get_logger(), "%s: lost odometry", ns_.c_str());
    prev_odom_stamp_ = rclcpp::Time();
    // TODO
  }

  action_mgr_->spin_once();

  // If we're flying manually and the drone isn't busy, send a cmd_vel message
  if (node_->mission() && state_ == State::flying && !action_mgr_->busy()) {
    cmd_vel_pub_->publish(twist_);
  }
}

} // namespace flock_base