#include "drone_base.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace drone_base {

//=============================================================================
// Constants
//=============================================================================

const int SPIN_RATE = 20;

const int MIN_BATTERY{20};  // Percent

//=============================================================================
// Utilities
//=============================================================================

double clamp(const double v, const double min, const double max)
{
  return v > max ? max : (v < min ? min : v);
}

bool button_down(const sensor_msgs::msg::Joy::SharedPtr curr, const sensor_msgs::msg::Joy &prev, int index)
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
// FlightController
//=============================================================================

FlightController::FlightController(rclcpp::Node &node,
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr &cmd_vel_pub) :
  node_(node), cmd_vel_pub_(cmd_vel_pub)
{
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER(node_, (*this), n, t, d)

  CXT_MACRO_INIT_PARAMETERS(FLIGHT_CONTROLLER_ALL_PARAMS, validate_parameters);

  reset();
}

void FlightController::validate_parameters()
{
  stabilize_time_ = rclcpp::Duration(static_cast<int64_t>(RCL_S_TO_NS(stabilize_time_sec_)));
}

void FlightController::parameters_changed(std::vector<rclcpp::Parameter> parameters)
{
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED((*this), n, t)

  CXT_MACRO_PARAMETERS_CHANGED_BODY(FLIGHT_CONTROLLER_ALL_PARAMS, parameters, validate_parameters)
}

void FlightController::reset()
{
  last_odom_time_ = rclcpp::Time();
  plan_ = nav_msgs::msg::Path();
}

void FlightController::set_plan(const nav_msgs::msg::Path::SharedPtr msg)
{
  reset();
  plan_ = *msg;
  // Go to first waypoint
  set_target(0);
}

void FlightController::set_target(int target)
{
  target_ = target;

  // Handle "done" case
  if (target_ < 0 || target_ >= plan_.poses.size()) {
    return;
  }

  // Set current target
  curr_target_.fromMsg(plan_.poses[target_].pose);
  curr_target_time_ = rclcpp::Time(plan_.poses[target_].header.stamp) - stabilize_time_;

  RCLCPP_INFO(node_.get_logger(), "target %d position: (%g, %g, %g), yaw %g, time %ld",
              target_,
              curr_target_.x,
              curr_target_.y,
              curr_target_.z,
              curr_target_.yaw,
              curr_target_time_.nanoseconds());

  // Set previous target, as well as velocity
  if (target_ == 0) {
    // Takeoff case
    prev_target_ = curr_target_;
    prev_target_time_ = node_.now();
    vx_ = vy_ = vz_ = vyaw_ = 0;
  } else {
    // Typical case
    prev_target_.fromMsg(plan_.poses[target_ - 1].pose);
    prev_target_time_ = rclcpp::Time(plan_.poses[target_ - 1].header.stamp);

    auto flight_time = (curr_target_time_ - prev_target_time_).seconds();
    assert(flight_time > 0);

    // Velocity vector from previous target to this target
    vx_ = (curr_target_.x - prev_target_.x) / flight_time;
    vy_ = (curr_target_.y - prev_target_.y) / flight_time;
    vz_ = (curr_target_.z - prev_target_.z) / flight_time;
    vyaw_ = norm_angle(curr_target_.yaw - prev_target_.yaw) / flight_time;

    RCLCPP_INFO(node_.get_logger(), "target %d velocity: (%g, %g, %g), yaw %g", target_, vx_, vy_, vz_, vyaw_);
  }

  // Initialize PID controllers to previous target, these will be updated in the odom callback
  x_controller_.set_target(prev_target_.x);
  y_controller_.set_target(prev_target_.y);
  z_controller_.set_target(prev_target_.z);
  yaw_controller_.set_target(prev_target_.yaw);
}

bool FlightController::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  bool retVal = false;

  rclcpp::Time msg_time(msg->header.stamp);

  if (valid(last_odom_time_)) {
    if (msg_time > curr_target_time_ + stabilize_time_) {
      if (curr_target_.close_enough(last_pose_)) {
        // Advance to the next target
        set_target(target_ + 1);
      } else {
        // Timeout
        retVal = true;
      }
    } else {
      // Compute expected position and set PID targets
      // The odom pipeline has a lag, so ignore messages that are older than prev_target_time_
      if (msg_time < curr_target_time_ && msg_time > prev_target_time_) {
        auto elapsed_time = (msg_time - prev_target_time_).seconds();
        x_controller_.set_target(prev_target_.x + vx_ * elapsed_time);
        y_controller_.set_target(prev_target_.y + vy_ * elapsed_time);
        z_controller_.set_target(prev_target_.z + vz_ * elapsed_time);
        yaw_controller_.set_target(norm_angle(prev_target_.yaw + vyaw_ * elapsed_time));
      }

      // Compute velocity
      auto dt = (msg_time - last_odom_time_).seconds();
      double ubar_x = x_controller_.calc(last_pose_.x, dt, 0);
      double ubar_y = y_controller_.calc(last_pose_.y, dt, 0);
      double ubar_z = z_controller_.calc(last_pose_.z, dt, 0);
      double ubar_yaw = yaw_controller_.calc(last_pose_.yaw, dt, 0);

      // Rotate ubar_x and ubar_y into the body frame
      double throttle, strafe;
      rotate_frame(ubar_x, ubar_y, last_pose_.yaw, throttle, strafe);

//      RCLCPP_INFO(node_.get_logger(), "%12ld "
//                  "x controller: target %7.3f, curr %7.3f, throttle %7.3f"
//                  "y controller: target %7.3f, curr %7.3f, strafe %7.3f",
//                  RCL_NS_TO_MS(msg_time.nanoseconds()),
//                  x_controller_.target(), last_pose_.x, throttle,
//                  y_controller_.target(), last_pose_.y, strafe);

      // Publish velocity
      publish_velocity(throttle, strafe, ubar_z, ubar_yaw);
    }
  }

  last_odom_time_ = msg_time;
  last_pose_.fromMsg(msg->pose.pose);
  return retVal;
}

void FlightController::publish_velocity(double throttle, double strafe, double vertical, double yaw)
{
  twist_.linear.x = clamp(throttle, -1.0, 1.0);
  twist_.linear.y = clamp(strafe, -1.0, 1.0);
  twist_.linear.z = clamp(vertical, -1.0, 1.0);
  twist_.angular.z = clamp(yaw, -1.0, 1.0);
  cmd_vel_pub_->publish(twist_);
}


//=============================================================================
// DroneBase node
//=============================================================================

DroneBase::DroneBase() : Node{"drone_base"}, fc_(*this, cmd_vel_pub_)
{
  // Suppress CLion warnings
  (void)cmd_vel_pub_;
  (void)start_mission_sub_;
  (void)stop_mission_sub_;
  (void)joy_sub_;
  (void)tello_response_sub_;
  (void)flight_data_sub_;
  (void)odom_sub_;
  (void)plan_sub_;

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOAD_PARAMETER((*this), cxt_, n, t, d)

  CXT_MACRO_INIT_PARAMETERS(DRONE_BASE_ALL_PARAMS, validate_parameters);

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cxt_, n, t)

  CXT_MACRO_REGISTER_PARAMETERS_CHANGED(DRONE_BASE_ALL_PARAMS, (*this), parameters_changed);

  action_mgr_ = std::make_unique<ActionMgr>(get_logger(),
    create_client<tello_msgs::srv::TelloAction>("tello_action"));

  cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  using std::placeholders::_1;
  auto joy_cb = std::bind(&DroneBase::joy_callback, this, _1);
  auto start_mission_cb = std::bind(&DroneBase::start_mission_callback, this, _1);
  auto stop_mission_cb = std::bind(&DroneBase::stop_mission_callback, this, _1);
  auto tello_response_cb = std::bind(&DroneBase::tello_response_callback, this, _1);
  auto flight_data_cb = std::bind(&DroneBase::flight_data_callback, this, _1);
  auto odom_cb = std::bind(&DroneBase::odom_callback, this, _1);
  auto plan_cb = std::bind(&DroneBase::plan_callback, this, _1);

  start_mission_sub_ = create_subscription<std_msgs::msg::Empty>("/start_mission", start_mission_cb);
  stop_mission_sub_ = create_subscription<std_msgs::msg::Empty>("/stop_mission", stop_mission_cb);
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>("joy", joy_cb);
  tello_response_sub_ = create_subscription<tello_msgs::msg::TelloResponse>("tello_response", tello_response_cb);
  flight_data_sub_ = create_subscription<tello_msgs::msg::FlightData>("flight_data", flight_data_cb);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("filtered_odom", odom_cb);
  plan_sub_ = create_subscription<nav_msgs::msg::Path>("plan", plan_cb);

  RCLCPP_INFO(get_logger(), "drone initialized");
}

void DroneBase::spin_once()
{
  rclcpp::Time ros_time = now();

  // Check for flight data timeout
  if (valid(flight_data_time_) && ros_time - flight_data_time_ > cxt_.flight_data_timeout_) {
    RCLCPP_ERROR(get_logger(), "flight data timeout, now %ld, last %ld", ros_time.nanoseconds(), flight_data_time_.nanoseconds());
    transition_state(Event::disconnected);
    flight_data_time_ = rclcpp::Time();  // Zero time is invalid
    odom_time_ = rclcpp::Time();
  }

  // Check for odometry timeout
  if (valid(odom_time_) && ros_time - odom_time_ > cxt_.odom_timeout_) {
    RCLCPP_ERROR(get_logger(), "odom timeout, now %ld, last %ld", ros_time.nanoseconds(), odom_time_.nanoseconds());
    transition_state(Event::odometry_stopped);
    odom_time_ = rclcpp::Time();
  }

  // Process any actions
  action_mgr_->spin_once();

  // Automated flight
  if (mission_ && fc_.have_plan()) {
    // We have a plan
    if (!fc_.is_plan_complete()) {
      // There's more to do
      if (state_ == State::ready_odom) {
        if (!action_mgr_->busy()) {
          RCLCPP_INFO(get_logger(), "start mission, taking off");
          start_action(Action::takeoff);
        }
      }
      else if (state_ == State::flight) {
        // Future: try to recover
        RCLCPP_ERROR(get_logger(), "lost odometry during mission");
        stop_mission();
      }
    } else {
      // We're done
      if (state_ == State::flight || state_ == State::flight_odom) {
        RCLCPP_INFO(get_logger(), "mission complete");
        stop_mission();
      }
    }
  }
}

void DroneBase::validate_parameters()
{
  cxt_.flight_data_timeout_ = rclcpp::Duration(static_cast<int64_t>(RCL_S_TO_NS(cxt_.flight_data_timeout_sec_)));
  cxt_.odom_timeout_ = rclcpp::Duration(static_cast<int64_t>(RCL_S_TO_NS(cxt_.odom_timeout_sec_)));
}

void DroneBase::parameters_changed(std::vector<rclcpp::Parameter> parameters)
{
#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cxt_, n, t)

  CXT_MACRO_PARAMETERS_CHANGED_BODY(DRONE_BASE_ALL_PARAMS, parameters, validate_parameters)

  // Explicitly call the FlightController's parameters_changed function. This is required
  // because the node.register_param_change_callback() function can only handle one callback. :(
  fc_.parameters_changed(parameters);
}

void DroneBase::start_mission_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  RCLCPP_INFO(get_logger(), "start mission");
  mission_ = true;
}

void DroneBase::stop_mission_callback(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  RCLCPP_INFO(get_logger(), "stop mission");
  stop_mission();
}

void DroneBase::stop_mission()
{
  mission_ = false;
  all_stop();
  if (state_ == State::flight || state_ == State::flight_odom) {
    // Future: queue action if busy
    start_action(Action::land);
  }
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

  // Manual flight
  if ((state_ == State::flight || state_ == State::flight_odom) && !action_mgr_->busy()) {
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
      fc_.publish_velocity(throttle, strafe, vertical, yaw);
    } else {
      fc_.publish_velocity(
        msg->axes[joy_axis_throttle_],
        msg->axes[joy_axis_strafe_],
        msg->axes[joy_axis_vertical_],
        msg->axes[joy_axis_yaw_]);
    }
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
  if (!valid(flight_data_time_)) {
    transition_state(Event::connected);
  }

  if (msg->bat < MIN_BATTERY && state_ != State::low_battery) {
    RCLCPP_ERROR(get_logger(), "low battery (%d)", msg->bat);
    transition_state(Event::low_battery);
    if (mission_) {
      stop_mission();
    }
  }

  flight_data_time_ = msg->header.stamp;
}

void DroneBase::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  // It's possible (but unlikely) to get an odom message before flight data
  if (valid(flight_data_time_)) {
    if (!valid(odom_time_)) {
      transition_state(Event::odometry_started);
    }

    else {
      // Automated flight
      if (mission_ && fc_.have_plan() && !fc_.is_plan_complete() && !action_mgr_->busy()) {
        if (fc_.odom_callback(msg)) {
          RCLCPP_ERROR(get_logger(), "didn't reach target");
          stop_mission();
        }
      }
    }

    odom_time_ = rclcpp::Time(msg->header.stamp);
  }
}

void DroneBase::plan_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (mission_) {
    fc_.set_plan(msg);
    RCLCPP_INFO(get_logger(), "got a plan with %d waypoints starting at time %ld",
      msg->poses.size(), rclcpp::Time(msg->header.stamp).nanoseconds());
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

  RCLCPP_INFO(get_logger(), "in state '%s', initiating action '%s'", g_states[state_], g_actions[action]);
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
    RCLCPP_INFO(get_logger(), "transition from '%s' to '%s'",g_states[state_], g_states[next_state]);
    state_ = next_state;
  }
}

void DroneBase::all_stop()
{
  RCLCPP_DEBUG(get_logger(), "ALL STOP");
  fc_.publish_velocity(0, 0, 0, 0);
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
  //auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_INFO);

  // rclcpp::Rate uses std::chrono::system_clock, so doesn't honor use_sim_time
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
