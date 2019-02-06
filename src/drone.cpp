#include "drone.hpp"

#include "flock_base.hpp"

namespace flock_base {

const rclcpp::Duration FLIGHT_DATA_TIMEOUT{1500000000};   // Nanoseconds
const rclcpp::Duration ODOM_TIMEOUT{1500000000};          // Nanoseconds
const int MIN_BATTERY{20};                                // Percent

std::map<State, const char *> g_states{
  {State::unknown, "unknown"},
  {State::ready, "ready"},
  {State::flight, "flight"},
  {State::ready_odom, "ready_odom"},
  {State::flight_odom, "flight_odom"},
  {State::low_battery, "low_battery"},
};

std::map<Event, const char *> g_events{
  {Event::connected, "connected"},
  {Event::disconnected, "disconnected"},
  {Event::odometry_started, "odometry_started"},
  {Event::odometry_stopped, "odometry_stopped"},
  {Event::low_battery, "low_battery"},
};

std::map<Action, const char *> g_actions{
  {Action::takeoff, "takeoff"},
  {Action::land, "land"},
};

struct EventTransition
{
  State curr_state_;
  Event event_;
  State next_state_;

  EventTransition(State curr_state, Event event, State next_state):
    curr_state_{curr_state}, event_{event}, next_state_{next_state}
  {}
};

struct ActionTransition
{
  State curr_state_;
  Action action_;
  State next_state_;

  ActionTransition(State curr_state, Action action, State next_state):
    curr_state_{curr_state}, action_{action}, next_state_{next_state}
  {}
};

bool valid_event_transition(const State state, const Event event, State &next_state)
{
  const static std::vector<EventTransition> valid_transitions{
    EventTransition{State::unknown, Event::connected, State::ready},

    EventTransition{State::ready, Event::disconnected, State::unknown},
    EventTransition{State::ready, Event::odometry_started, State::ready_odom},
    EventTransition{State::ready, Event::low_battery, State::low_battery},

    EventTransition{State::flight, Event::disconnected, State::unknown},
    EventTransition{State::flight, Event::odometry_started, State::flight_odom},
    EventTransition{State::flight, Event::low_battery, State::low_battery},

    EventTransition{State::ready_odom, Event::disconnected, State::unknown},
    EventTransition{State::ready_odom, Event::odometry_stopped, State::ready},
    EventTransition{State::ready_odom, Event::low_battery, State::low_battery},

    EventTransition{State::flight_odom, Event::disconnected, State::unknown},
    EventTransition{State::flight_odom, Event::odometry_stopped, State::flight},
    EventTransition{State::flight_odom, Event::low_battery, State::low_battery},

    EventTransition{State::low_battery, Event::disconnected, State::unknown},
  };

  for (auto i = valid_transitions.begin(); i != valid_transitions.end(); i++) {
    if (i->curr_state_ == state && i->event_ == event) {
      next_state = i->next_state_;
      return true;
    }
  }

  return false;
}

bool valid_action_transition(const State state, const Action action, State &next_state)
{
  const static std::vector<ActionTransition> valid_transitions{
    ActionTransition{State::ready, Action::takeoff, State::flight},
    ActionTransition{State::ready_odom, Action::takeoff, State::flight_odom},

    ActionTransition{State::flight, Action::land, State::ready},
    ActionTransition{State::flight_odom, Action::land, State::ready_odom},
  };

  for (auto i = valid_transitions.begin(); i != valid_transitions.end(); i++) {
    if (i->curr_state_ == state && i->action_ == action) {
      next_state = i->next_state_;
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

  RCLCPP_INFO(node_->get_logger(), "[%s] drone initialized", ns_.c_str());
}

void Drone::start_action(Action action)
{
  if (action_mgr_->busy()) {
    RCLCPP_INFO(node_->get_logger(), "[%s] busy, dropping %s", ns_.c_str(), g_actions[action]);
    return;
  }

  State next_state;
  if (!valid_action_transition(state_, action, next_state)) {
    RCLCPP_DEBUG(node_->get_logger(), "[%s] %s not allowed in %s", ns_.c_str(), g_actions[action], g_states[state_]);
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "[%s] initiating %s", ns_.c_str(), g_actions[action]);
  action_mgr_->send(action, g_actions[action]);
}

void Drone::transition_state(Action action)
{
  State next_state;
  if (!valid_action_transition(state_, action, next_state)) {
    RCLCPP_DEBUG(node_->get_logger(), "[%s] %s not allowed in %s", ns_.c_str(), g_actions[action], g_states[state_]);
    return;
  }

  transition_state(next_state);
}

void Drone::transition_state(Event event)
{
  State next_state;
  if (!valid_event_transition(state_, event, next_state)) {
    RCLCPP_DEBUG(node_->get_logger(), "[%s] %s not allowed in %s", ns_.c_str(), g_events[event], g_states[state_]);
    return;
  }

  transition_state(next_state);
}

void Drone::transition_state(State next_state)
{
  if (state_ != next_state) {
    RCLCPP_INFO(node_->get_logger(), "[%s] transition to %s", ns_.c_str(), g_states[next_state]);
    state_ = next_state;
  }
}

void Drone::tello_response_callback(tello_msgs::msg::TelloResponse::SharedPtr msg)
{
  ActionMgr::State result = action_mgr_->complete(msg);
  if (result == ActionMgr::State::succeeded) {
    transition_state(action_mgr_->action());
  }
}

void Drone::flight_data_callback(tello_msgs::msg::FlightData::SharedPtr msg)
{
  if (!receiving_flight_data()) {
    transition_state(Event::connected);
  }

  if (msg->bat < MIN_BATTERY && state_ != State::low_battery) {
    transition_state(Event::low_battery);
  }

  prev_flight_data_stamp_ = msg->header.stamp;
}

void Drone::odom_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  // It's possible (but unlikely) to get an odom message before flight data
  if (receiving_flight_data()) {
    if (!receiving_odometry()) {
      transition_state(Event::odometry_started);
    }

    prev_odom_stamp_ = msg->header.stamp;
  }
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
  // Check for flight data timeout
  if (receiving_flight_data() && node_->now() - prev_flight_data_stamp_ > FLIGHT_DATA_TIMEOUT) {
    transition_state(Event::disconnected);
    prev_flight_data_stamp_ = rclcpp::Time();
    prev_odom_stamp_ = rclcpp::Time();
  }

  // Check for odometry_started timeout
  if (receiving_odometry() && node_->now() - prev_odom_stamp_ > ODOM_TIMEOUT) {
    transition_state(Event::odometry_stopped);
    prev_odom_stamp_ = rclcpp::Time();
  }

  // Process any actions
  action_mgr_->spin_once();

  // If we're flying manually and the drone isn't busy, send a cmd_vel message
  if (node_->mission() && state_ == State::flight && !action_mgr_->busy()) {
    cmd_vel_pub_->publish(twist_);
  }
}

} // namespace flock_base