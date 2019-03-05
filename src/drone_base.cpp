#include "drone_base.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace drone_base {

//=============================================================================
// Constants
//=============================================================================

const int SPIN_RATE = 20;
const double DT = 1.0 / SPIN_RATE;

const rclcpp::Duration FLIGHT_DATA_TIMEOUT{1500000000};   // Nanoseconds
const rclcpp::Duration ODOM_TIMEOUT{1500000000};          // Nanoseconds
const int MIN_BATTERY{20};                                // Percent

//=============================================================================
// Utilities
//=============================================================================

template<typename T>
inline T clamp(const T v, const T min, const T max)
{
  return v > max ? max : (v < min ? min : v);
}

double get_yaw(const geometry_msgs::msg::Pose &p)
{
  tf2::Quaternion q;
  tf2::fromMsg(p.orientation, q);
  tf2::Matrix3x3 m(q);
  tf2Scalar roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

inline bool close_enough(const geometry_msgs::msg::Pose &p1, const geometry_msgs::msg::Pose &p2)
{
  const double EPSILON_XYZ = 0.1;
  const double EPSILON_YAW = 0.1;

  return std::abs(p1.position.x - p2.position.x) < EPSILON_XYZ &&
    std::abs(p1.position.y - p2.position.y) < EPSILON_XYZ &&
    std::abs(p1.position.z - p2.position.z) < EPSILON_XYZ &&
    std::abs(get_yaw(p1) - get_yaw(p2)) < EPSILON_YAW;
}

inline bool button_down(const sensor_msgs::msg::Joy::SharedPtr curr, const sensor_msgs::msg::Joy &prev, int index)
{
  return curr->buttons[index] && !prev.buttons[index];
}

//=============================================================================
// States, events and actions
//=============================================================================

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
  // Allow for emergency landing in all states
  const static std::vector<ActionTransition> valid_transitions{
    ActionTransition{State::unknown, Action::land, State::unknown},

    ActionTransition{State::ready, Action::takeoff, State::flight},
    ActionTransition{State::ready, Action::land, State::ready},

    ActionTransition{State::flight, Action::land, State::ready},

    ActionTransition{State::ready_odom, Action::takeoff, State::flight_odom},
    ActionTransition{State::ready_odom, Action::land, State::ready_odom},

    ActionTransition{State::flight_odom, Action::land, State::ready_odom},

    ActionTransition{State::low_battery, Action::land, State::low_battery},
  };

  for (auto i = valid_transitions.begin(); i != valid_transitions.end(); i++) {
    if (i->curr_state_ == state && i->action_ == action) {
      next_state = i->next_state_;
      return true;
    }
  }

  return false;
}

//=============================================================================
// DroneBase node
//=============================================================================

DroneBase::DroneBase() : Node{"drone_base"}
{
  action_mgr_ = std::make_unique<ActionMgr>(get_logger(),
    create_client<tello_msgs::srv::TelloAction>("tello_action"));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  auto joy_cb = std::bind(&DroneBase::joy_callback, this, std::placeholders::_1);
  auto start_mission_cb = std::bind(&DroneBase::start_mission_callback, this, std::placeholders::_1);
  auto stop_mission_cb = std::bind(&DroneBase::stop_mission_callback, this, std::placeholders::_1);
  auto tello_response_cb = std::bind(&DroneBase::tello_response_callback, this, std::placeholders::_1);
  auto flight_data_cb = std::bind(&DroneBase::flight_data_callback, this, std::placeholders::_1);
  auto odom_cb = std::bind(&DroneBase::odom_callback, this, std::placeholders::_1);
  auto plan_cb = std::bind(&DroneBase::plan_callback, this, std::placeholders::_1);

  start_mission_sub_ = create_subscription<std_msgs::msg::Empty>("/start_mission", start_mission_cb);
  stop_mission_sub_ = create_subscription<std_msgs::msg::Empty>("/stop_mission", stop_mission_cb);
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>("joy", joy_cb);
  tello_response_sub_ = create_subscription<tello_msgs::msg::TelloResponse>("tello_response", tello_response_cb);
  flight_data_sub_ = create_subscription<tello_msgs::msg::FlightData>("flight_data", flight_data_cb);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("filtered_odom", odom_cb);
  plan_sub_ = create_subscription<nav_msgs::msg::Path>("global_plan", plan_cb);  // TODO local_plan

  RCLCPP_INFO(get_logger(), "drone initialized");
}

void DroneBase::spin_once()
{
  // Check for flight data timeout
  if (receiving_flight_data() && now() - prev_flight_data_stamp_ > FLIGHT_DATA_TIMEOUT) {
    transition_state(Event::disconnected);
    prev_flight_data_stamp_ = rclcpp::Time();
    prev_odom_stamp_ = rclcpp::Time();
  }

  // Check for odometry timeout
  if (receiving_odometry() && now() - prev_odom_stamp_ > ODOM_TIMEOUT) {
    transition_state(Event::odometry_stopped);
    prev_odom_stamp_ = rclcpp::Time();
  }

  // Process any actions
  action_mgr_->spin_once();

  // Manual flight
  if (!mission_) {
    if ((state_ == State::flight || state_ == State::flight_odom) && !action_mgr_->busy()) {
      cmd_vel_pub_->publish(twist_);
    }
  }

  // Automated flight
  if (mission_ && have_plan_) {
    // We have a plan
    if (plan_target_ < plan_.poses.size()) {
      // There's more to do
      if (state_ == State::ready_odom) {
        if (!action_mgr_->busy()) {
          RCLCPP_INFO(get_logger(), "start mission, taking off");
          start_action(Action::takeoff);
        }
      }
      else if (state_ == State::flight_odom) {
        if (close_enough(plan_.poses[plan_target_].pose, odom_.pose.pose)) {
          // Advance to the next target
          if (++plan_target_ < plan_.poses.size()) {
            set_pid_controllers();
          }
        } else {
          // Fly to target pose
          double ubar_x = x_controller_.calc(odom_.pose.pose.position.x, DT, 0); // No feedforward
          double ubar_y = y_controller_.calc(odom_.pose.pose.position.y, DT, 0);
          double ubar_z = z_controller_.calc(odom_.pose.pose.position.z, DT, 0);
          double ubar_yaw = yaw_controller_.calc(get_yaw(odom_.pose.pose), DT, 0);
          set_velocity(ubar_x, ubar_y, ubar_z, ubar_yaw); // TODO accel vs vel
          cmd_vel_pub_->publish(twist_);
        }
      }
      else if (state_ == State::flight) {
        RCLCPP_ERROR(get_logger(), "lost odometry during mission");
      }
    } else {
      // We're done
      if (state_ == State::flight || state_ == State::flight_odom) {
        if (!action_mgr_->busy()) {
          RCLCPP_INFO(get_logger(), "mission complete, landing");
          start_action(Action::land);
        }
      }
    }
  }
}

void DroneBase::start_mission_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  mission_ = true;
  have_plan_ = false;  // Start over -- wait for a new plan
  RCLCPP_INFO(get_logger(), "start mission, waiting for a plan");
}

void DroneBase::stop_mission_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  mission_ = false;
  RCLCPP_INFO(get_logger(), "stop mission");
}

void DroneBase::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
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

void DroneBase::tello_response_callback(const tello_msgs::msg::TelloResponse::SharedPtr msg)
{
  ActionMgr::State result = action_mgr_->complete(msg);
  if (result == ActionMgr::State::succeeded) {
    transition_state(action_mgr_->action());
  }
}

void DroneBase::flight_data_callback(const tello_msgs::msg::FlightData::SharedPtr msg)
{
  if (!receiving_flight_data()) {
    transition_state(Event::connected);
  }

  if (msg->bat < MIN_BATTERY && state_ != State::low_battery) {
    RCLCPP_ERROR(get_logger(), "LOW BATTERY %d", msg->bat);
    transition_state(Event::low_battery);
  }

  prev_flight_data_stamp_ = msg->header.stamp;
}

void DroneBase::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // It's possible (but unlikely) to get an odom message before flight data
  if (receiving_flight_data()) {
    if (!receiving_odometry()) {
      transition_state(Event::odometry_started);
    }

    prev_odom_stamp_ = msg->header.stamp;
    odom_ = *msg;
  }
}

void DroneBase::plan_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (mission_ && !have_plan_) {
    plan_ = *msg;
    have_plan_ = true;
    RCLCPP_INFO(get_logger(), "got a plan with %d waypoints", plan_.poses.size());

    // Go to first waypoint
    plan_target_ = 0;
    set_pid_controllers();
  }
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

void DroneBase::set_velocity(double throttle, double strafe, double vertical, double yaw)
{
  twist_.linear.x = clamp(throttle, -1.0, 1.0);
  twist_.linear.y = clamp(strafe, -1.0, 1.0);
  twist_.linear.z = clamp(vertical, -1.0, 1.0);
  twist_.angular.z = clamp(yaw, -1.0, 1.0);
}

void DroneBase::set_pid_controllers()
{
  double yaw = get_yaw(plan_.poses[plan_target_].pose);

  RCLCPP_INFO(get_logger(), "target: %g, %g, %g, %g", plan_.poses[plan_target_].pose.position.x,
    plan_.poses[plan_target_].pose.position.y, plan_.poses[plan_target_].pose.position.z, yaw);

  x_controller_.set_target(plan_.poses[plan_target_].pose.position.x);
  y_controller_.set_target(plan_.poses[plan_target_].pose.position.y);
  z_controller_.set_target(plan_.poses[plan_target_].pose.position.z);
  yaw_controller_.set_target(yaw);
}

} // namespace drone_base

//=============================================================================
// main
//=============================================================================

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Create node
  auto node = std::make_shared<drone_base::DroneBase>();
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

  rclcpp::Rate r(drone_base::SPIN_RATE);
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