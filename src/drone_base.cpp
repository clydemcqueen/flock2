#include "drone_base.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "flight_controller_basic.hpp"
#include "flight_controller_simple.hpp"

namespace drone_base {

//=============================================================================
// Constants
//=============================================================================

const int SPIN_RATE = 20;

const int MIN_BATTERY{20};  // Percent

//=============================================================================
// Utilities
//=============================================================================

bool button_down(const sensor_msgs::msg::Joy::SharedPtr &curr, const sensor_msgs::msg::Joy &prev, int index)
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
  CXT_MACRO_INIT_PARAMETERS(DRONE_BASE_ALL_PARAMS, validate_parameters)

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_PARAMETER_CHANGED(cxt_, n, t)
  CXT_MACRO_REGISTER_PARAMETERS_CHANGED((*this), DRONE_BASE_ALL_PARAMS, validate_parameters)

//  fc_ = std::make_unique<FlightControllerBasic>(*this, cmd_vel_pub_);
  fc_ = std::make_unique<FlightControllerSimple>(*this, cmd_vel_pub_);

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

  start_mission_sub_ = create_subscription<std_msgs::msg::Empty>("/start_mission", 10, start_mission_cb);
  stop_mission_sub_ = create_subscription<std_msgs::msg::Empty>("/stop_mission", 10, stop_mission_cb);
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>("joy", 10, joy_cb);
  tello_response_sub_ = create_subscription<tello_msgs::msg::TelloResponse>("tello_response", 10, tello_response_cb);
  flight_data_sub_ = create_subscription<tello_msgs::msg::FlightData>("flight_data", 10, flight_data_cb);
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("filtered_odom", 10, odom_cb);
  plan_sub_ = create_subscription<nav_msgs::msg::Path>("plan", 10, plan_cb);

  RCLCPP_INFO(get_logger(), "drone initialized");
}

void DroneBase::spin_once()
{
  rclcpp::Time ros_time = now();

  // Check for flight data timeout
  if (PoseUtil::is_valid_time(flight_data_time_) && ros_time - flight_data_time_ > cxt_.flight_data_timeout_) {
    RCLCPP_ERROR(get_logger(), "flight data timeout, now %ld, last %ld",
                 RCL_NS_TO_MS(ros_time.nanoseconds()), RCL_NS_TO_MS(flight_data_time_.nanoseconds()));
    transition_state(Event::disconnected);
    flight_data_time_ = rclcpp::Time();  // Zero time is invalid
    odom_time_ = rclcpp::Time();
  }

  // Check for odometry timeout
  if (PoseUtil::is_valid_time(odom_time_) && ros_time - odom_time_ > cxt_.odom_timeout_) {
    RCLCPP_ERROR(get_logger(), "odom timeout, now %ld, last %ld",
                 RCL_NS_TO_MS(ros_time.nanoseconds()), RCL_NS_TO_MS(odom_time_.nanoseconds()));
    transition_state(Event::odometry_stopped);
    odom_time_ = rclcpp::Time();
  }

  // Process any actions
  action_mgr_->spin_once();

  // Automated flight
  if (mission_ && fc_->have_plan()) {
    // We have a plan
    if (!fc_->is_plan_complete()) {
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

  RCLCPP_INFO(get_logger(), "DroneBase Parameters");

#undef CXT_MACRO_MEMBER
#define CXT_MACRO_MEMBER(n, t, d) CXT_MACRO_LOG_PARAMETER(RCLCPP_INFO, get_logger(), cxt_, n, t, d)
  DRONE_BASE_ALL_PARAMS
}

void DroneBase::start_mission_callback(std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  RCLCPP_INFO(get_logger(), "start mission");
  mission_ = true;
}

void DroneBase::stop_mission_callback(std_msgs::msg::Empty::SharedPtr msg)
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

  // Manual flight
  if ((state_ == State::flight || state_ == State::flight_odom) && !action_mgr_->busy()) {
    // Trim (slow, steady) mode vs. joystick mode
    if (msg->axes[joy_axis_trim_lr_] != 0. || msg->axes[joy_axis_trim_fb_] != 0.) {
      const static double TRIM_SPEED{0.2};
      double throttle{0}, strafe{0}, vertical{0}, yaw{0};
      if (msg->axes[joy_axis_trim_lr_] != 0.) {
        if (msg->buttons[joy_button_shift_]) {
          yaw = TRIM_SPEED * msg->axes[joy_axis_trim_lr_];
        } else {
          strafe = TRIM_SPEED * msg->axes[joy_axis_trim_lr_];
        }
      }
      if (msg->axes[joy_axis_trim_fb_] != 0.) {
        if (msg->buttons[joy_button_shift_]) {
          throttle = TRIM_SPEED * msg->axes[joy_axis_trim_fb_];
        } else {
          vertical = TRIM_SPEED * msg->axes[joy_axis_trim_fb_];
        }
      }
      fc_->publish_velocity(throttle, strafe, vertical, yaw);
    } else {
      fc_->publish_velocity(
        msg->axes[joy_axis_throttle_],
        msg->axes[joy_axis_strafe_],
        msg->axes[joy_axis_vertical_],
        msg->axes[joy_axis_yaw_]);
    }
  }

  prev_msg = *msg;
}

void DroneBase::tello_response_callback(tello_msgs::msg::TelloResponse::SharedPtr msg)
{
  ActionMgr::State result = action_mgr_->complete(std::move(msg));
  if (result == ActionMgr::State::succeeded) {
    transition_state(action_mgr_->action());
  }
}

void DroneBase::flight_data_callback(tello_msgs::msg::FlightData::SharedPtr msg)
{
  if (!PoseUtil::is_valid_time(flight_data_time_)) {
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

void DroneBase::odom_callback(nav_msgs::msg::Odometry::SharedPtr msg)
{
  // It's possible (but unlikely) to get an odom message before flight data
  if (PoseUtil::is_valid_time(flight_data_time_)) {
    if (!PoseUtil::is_valid_time(odom_time_)) {
      transition_state(Event::odometry_started);
    }

    else {
      // Automated flight
      if (mission_ && fc_->have_plan() && !fc_->is_plan_complete() && !action_mgr_->busy()) {
        if (fc_->odom_callback(msg)) {
          RCLCPP_ERROR(get_logger(), "didn't reach target");
          stop_mission();
        }
      }
    }

    odom_time_ = rclcpp::Time(msg->header.stamp);
  }
}

void DroneBase::plan_callback(nav_msgs::msg::Path::SharedPtr msg)
{
  if (mission_) {
    RCLCPP_INFO(get_logger(), "Got plan with %d waypoints, plan msg time %ld, last odom time %ld, ros time %ld ",
                msg->poses.size(), RCL_NS_TO_MS(rclcpp::Time(msg->header.stamp).nanoseconds()),
                RCL_NS_TO_MS(odom_time_.nanoseconds()), RCL_NS_TO_MS(now().nanoseconds()));
    fc_->set_plan(msg);
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
  fc_->publish_velocity(0, 0, 0, 0);
}

} // namespace drone_base

//=============================================================================
// main
//=============================================================================

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

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
