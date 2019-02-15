#include "drone_base.hpp"

namespace drone_base {

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

DroneBase::DroneBase() : Node{"drone_base"}
{
  action_mgr_ = std::make_unique<ActionMgr>(get_logger(),
    create_client<tello_msgs::srv::TelloAction>("tello_action"));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  auto tello_response_cb = std::bind(&DroneBase::tello_response_callback, this, std::placeholders::_1);
  auto flight_data_cb = std::bind(&DroneBase::flight_data_callback, this, std::placeholders::_1);
  auto odom_cb = std::bind(&DroneBase::odom_callback, this, std::placeholders::_1);

  tello_response_sub_ = create_subscription<tello_msgs::msg::TelloResponse>("tello_response", tello_response_cb);
  flight_data_sub_ = create_subscription<tello_msgs::msg::FlightData>("flight_data", flight_data_cb);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("filtered_odom", odom_cb);

  RCLCPP_INFO(get_logger(), "drone initialized");
}

void DroneBase::start_action(Action action)
{
  if (action_mgr_->busy()) {
    RCLCPP_INFO(get_logger(), "busy, dropping %s", g_actions[action]);
    return;
  }

  State next_state;
  if (!valid_action_transition(state_, action, next_state)) {
    RCLCPP_DEBUG(get_logger(), "%s not allowed in %s", g_actions[action], g_states[state_]);
    return;
  }

  RCLCPP_INFO(get_logger(), "initiating %s", g_actions[action]);
  action_mgr_->send(action, g_actions[action]);
}

void DroneBase::transition_state(Action action)
{
  State next_state;
  if (!valid_action_transition(state_, action, next_state)) {
    RCLCPP_DEBUG(get_logger(), "%s not allowed in %s", g_actions[action], g_states[state_]);
    return;
  }

  transition_state(next_state);
}

void DroneBase::transition_state(Event event)
{
  State next_state;
  if (!valid_event_transition(state_, event, next_state)) {
    RCLCPP_DEBUG(get_logger(), "%s not allowed in %s", g_events[event], g_states[state_]);
    return;
  }

  transition_state(next_state);
}

void DroneBase::transition_state(State next_state)
{
  if (state_ != next_state) {
    RCLCPP_INFO(get_logger(), "transition to %s", g_states[next_state]);
    state_ = next_state;
  }
}

inline bool button_down(const sensor_msgs::msg::Joy::SharedPtr curr, const sensor_msgs::msg::Joy &prev, int index)
{
  return curr->buttons[index] && !prev.buttons[index];
}

void DroneBase::joy_callback(sensor_msgs::msg::Joy::SharedPtr msg)
{
  static sensor_msgs::msg::Joy prev_msg;

  // Ignore the joystick if we're in a mission
  if (mission_) {
    prev_msg = *msg;
    return;
  }

  // Takeoff/land
  if (button_down(msg, prev_msg, joy_button_takeoff_)) {
    start_action(Action::takeoff);
  } else if (button_down(msg, prev_msg, joy_button_land_)) {
    start_action(Action::land);
  }

  // Trim (slow, steady) mode vs. joystick mode
  if (msg->axes[joy_axis_trim_lr_] || msg->axes[joy_axis_trim_fb_]) {
    const static double TRIM_SPEED{0.2};
    double throttle{0}, strafe{0}, vertical{0}, yaw{0};
    if (msg->axes[joy_axis_trim_lr_]) {
      if (msg->buttons[joy_button_shift_]) {
        yaw = TRIM_SPEED * msg->axes[joy_axis_trim_lr_];
      } else {
        strafe = TRIM_SPEED * msg->axes[joy_axis_trim_lr_];
      }
    }
    if (msg->axes[joy_axis_trim_fb_]) {
      if (msg->buttons[joy_button_shift_]) {
        throttle = TRIM_SPEED * msg->axes[joy_axis_trim_fb_];
      } else {
        vertical = TRIM_SPEED * msg->axes[joy_axis_trim_fb_];
      }
    }
    set_velocity(throttle, strafe, vertical, yaw);
  } else {
    set_velocity(
      msg->axes[joy_axis_throttle_],
      msg->axes[joy_axis_strafe_],
      msg->axes[joy_axis_vertical_],
      msg->axes[joy_axis_yaw_]);
  }

  prev_msg = *msg;
}

void DroneBase::start_mission_callback(std_msgs::msg::Empty::SharedPtr msg)
{
  mission_ = true;
}

void DroneBase::stop_mission_callback(std_msgs::msg::Empty::SharedPtr msg)
{
  mission_ = false;
}

void DroneBase::tello_response_callback(tello_msgs::msg::TelloResponse::SharedPtr msg)
{
  ActionMgr::State result = action_mgr_->complete(msg);
  if (result == ActionMgr::State::succeeded) {
    transition_state(action_mgr_->action());
  }
}

void DroneBase::flight_data_callback(tello_msgs::msg::FlightData::SharedPtr msg)
{
  if (!receiving_flight_data()) {
    transition_state(Event::connected);
  }

  if (msg->bat < MIN_BATTERY && state_ != State::low_battery) {
    transition_state(Event::low_battery);
  }

  prev_flight_data_stamp_ = msg->header.stamp;
}

void DroneBase::odom_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  // It's possible (but unlikely) to get an odom message before flight data
  if (receiving_flight_data()) {
    if (!receiving_odometry()) {
      transition_state(Event::odometry_started);
    }

    prev_odom_stamp_ = msg->header.stamp;
  }
}

void DroneBase::set_velocity(double throttle, double strafe, double vertical, double yaw)
{
  twist_.linear.x = throttle;
  twist_.linear.y = strafe;
  twist_.linear.z = vertical;
  twist_.angular.z = yaw;
}

void DroneBase::spin_once()
{
  // Check for flight data timeout
  if (receiving_flight_data() && now() - prev_flight_data_stamp_ > FLIGHT_DATA_TIMEOUT) {
    transition_state(Event::disconnected);
    prev_flight_data_stamp_ = rclcpp::Time();
    prev_odom_stamp_ = rclcpp::Time();
  }

  // Check for odometry_started timeout
  if (receiving_odometry() && now() - prev_odom_stamp_ > ODOM_TIMEOUT) {
    transition_state(Event::odometry_stopped);
    prev_odom_stamp_ = rclcpp::Time();
  }

  // Process any actions
  action_mgr_->spin_once();

  // If we're flying manually and the drone isn't busy, send a cmd_vel message
  if (!mission_ && state_ == State::flight && !action_mgr_->busy()) {
    cmd_vel_pub_->publish(twist_);
  }
}

} // namespace drone_base

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Create node
  auto node = std::make_shared<drone_base::DroneBase>();
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

  rclcpp::Rate r(20);
  while (rclcpp::ok())
  {
    // Do our work
    node->spin_once();

    // Respond to incoming messages
    rclcpp::spin_some(node);

    // Wait
    r.sleep();
  }

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}