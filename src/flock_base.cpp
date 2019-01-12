#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tello_msgs/srv/tello_action.hpp"
#include "tello_msgs/msg/flight_data.hpp"

#include "action_mgr.hpp"

namespace flock_base {

// XBox One joystick axes and buttons
const int JOY_AXIS_LEFT_LR = 0;           // Left stick left/right; 1.0 is left and -1.0 is right
const int JOY_AXIS_LEFT_FB = 1;           // Left stick forward/back; 1.0 is forward and -1.0 is back
const int JOY_AXIS_LEFT_TRIGGER = 2;      // Left trigger
const int JOY_AXIS_RIGHT_LR = 3;          // Right stick left/right; 1.0 is left and -1.0 is right
const int JOY_AXIS_RIGHT_FB = 4;          // Right stick forward/back; 1.0 is forward and -1.0 is back
const int JOY_AXIS_RIGHT_TRIGGER = 5;     // Right trigger
const int JOY_AXIS_TRIM_LR = 6;           // Trim left/right; 1.0 for left and -1.0 for right
const int JOY_AXIS_TRIM_FB = 7;           // Trim forward/back; 1.0 for forward and -1.0 for back
const int JOY_BUTTON_A = 0;               // A button
const int JOY_BUTTON_B = 1;               // B button
const int JOY_BUTTON_X = 2;               // X button
const int JOY_BUTTON_Y = 3;               // Y button
const int JOY_BUTTON_LEFT_BUMPER = 4;     // Left bumper
const int JOY_BUTTON_RIGHT_BUMPER = 5;    // Right bumper
const int JOY_BUTTON_VIEW = 6;            // View button
const int JOY_BUTTON_MENU = 7;            // Menu button
const int JOY_BUTTON_LOGO = 8;            // XBox logo button
const int JOY_BUTTON_LEFT_STICK = 9;      // Left stick button
const int JOY_BUTTON_RIGHT_STICK = 10;    // Right stick button

enum class Action
{
  takeoff,
  land,
  start_mission,
  stop_mission,
  connect,
  disconnect,
};

std::map<Action, std::string> g_action_strs{
  {Action::takeoff, "takeoff"},
  {Action::land, "land"},
  {Action::start_mission, "start_mission"},
  {Action::stop_mission, "stop_mission"},
  {Action::connect, "connect"},
  {Action::disconnect, "disconnect"},
};

enum class State
{
  unknown,
  landed,
  fly_manual,
  fly_mission,
};

std::map<State, std::string> g_state_strs{
  {State::unknown, "unknown"},
  {State::landed, "landed"},
  {State::fly_manual, "fly_manual"},
  {State::fly_mission, "fly_mission"},
};

struct Transition
{
  State curr_state_;
  Action action_;
  State next_state_;
  bool internal_;

  Transition(State curr_state, Action action, State next_state, bool internal):
    curr_state_{curr_state}, action_{action}, next_state_{next_state}, internal_{internal}
  {}
};

std::vector<Transition> g_transitions{
  // Connect / disconnect
  Transition{State::unknown, Action::connect, State::landed, true},
  Transition{State::landed, Action::disconnect, State::unknown, true},
  Transition{State::fly_manual, Action::disconnect, State::unknown, true},
  Transition{State::fly_mission, Action::disconnect, State::unknown, true},

  // Take off / land
  Transition{State::landed, Action::takeoff, State::fly_manual, false},
  Transition{State::fly_manual, Action::land, State::landed, false},

  // Start / stop mission
  Transition{State::fly_manual, Action::start_mission, State::fly_mission, true},
  Transition{State::fly_mission, Action::stop_mission, State::fly_manual, true},
};

bool find_transition(const State state, const Action action, State &next_state, bool &internal)
{
  for (auto i = g_transitions.begin(); i != g_transitions.end(); i++) {
    if (i->curr_state_ == state && i->action_ == action) {
      next_state = i->next_state_;
      internal = i->internal_;
      return true;
    }
  }

  return false;
}

class FlockBase : public rclcpp::Node
{
  double trim_speed_ = 0.2;

  // Actions
  std::unique_ptr<ActionMgr> action_mgr_;
  geometry_msgs::msg::Twist twist_;

  // Flight state
  State state_ = State::unknown;

  // Joystick assignments
  int joy_axis_throttle_ = JOY_AXIS_RIGHT_FB;
  int joy_axis_strafe_ = JOY_AXIS_RIGHT_LR;
  int joy_axis_vertical_ = JOY_AXIS_LEFT_FB;
  int joy_axis_yaw_ = JOY_AXIS_LEFT_LR;
  int joy_button_takeoff_ = JOY_BUTTON_MENU;
  int joy_button_land_ = JOY_BUTTON_VIEW;
  int joy_button_shift_ = JOY_BUTTON_LEFT_BUMPER;
  int joy_button_stop_mission_ = JOY_BUTTON_A;
  int joy_button_start_mission_ = JOY_BUTTON_B;
  int joy_axis_trim_lr_ = JOY_AXIS_TRIM_LR;
  int joy_axis_trim_fb_ = JOY_AXIS_TRIM_FB;

  // TODO trim

  // Publications
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr start_mission_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_mission_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // Subscriptions
  rclcpp::Subscription<tello_msgs::msg::TelloResponse>::SharedPtr tello_response_sub_;
  rclcpp::Subscription<tello_msgs::msg::FlightData>::SharedPtr flight_data_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

public:

  FlockBase() : Node{"flock_base"}
  {
    action_mgr_ = std::make_unique<ActionMgr>(get_logger(),
      create_client<tello_msgs::srv::TelloAction>("tello_action"));

    // TODO trim

    start_mission_pub_ = create_publisher<std_msgs::msg::Empty>("start_mission", 1);
    stop_mission_pub_ = create_publisher<std_msgs::msg::Empty>("stop_mission", 1);
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

    auto tello_response_cb = std::bind(&FlockBase::tello_response_callback, this, std::placeholders::_1);
    tello_response_sub_ = create_subscription<tello_msgs::msg::TelloResponse>("tello_response", tello_response_cb);

    auto flight_data_cb = std::bind(&FlockBase::flight_data_callback, this, std::placeholders::_1);
    flight_data_sub_ = create_subscription<tello_msgs::msg::FlightData>("flight_data", flight_data_cb);

    auto joy_cb = std::bind(&FlockBase::joy_callback, this, std::placeholders::_1);
    joy_sub_ = create_subscription<sensor_msgs::msg::Joy>("joy", joy_cb);
  }

  ~FlockBase()
  {}

  // Start an action. Can be called from a ROS callback: fast, doesn't block.
  void start_action(Action action)
  {
    if (action_mgr_->busy()) {
      RCLCPP_DEBUG(get_logger(), "busy, dropping %s", g_action_strs[action].c_str());
      return;
    }

    State next_state;
    bool internal;
    if (!find_transition(state_, action, next_state, internal)) {
      RCLCPP_DEBUG(get_logger(), "%s not allowed in %s", g_action_strs[action].c_str(), g_state_strs[state_].c_str());
      return;
    }

    if (internal) {
      internal_action(action);
    } else {
      RCLCPP_INFO(get_logger(), "initiating %s", g_action_strs[action].c_str());
      action_mgr_->send(action, g_action_strs[action]);
    }
  }

  // These actions don't require a call to tello_driver.
  void internal_action(Action action)
  {
    if (action == Action::start_mission) {
      start_mission_pub_->publish(std_msgs::msg::Empty());
    } else if (action == Action::stop_mission) {
      stop_mission_pub_->publish(std_msgs::msg::Empty());
    }

    transition_state(action);
  }

  // Transition to a new state.
  void transition_state(Action action)
  {
    State next_state;
    bool internal;
    if (!find_transition(state_, action, next_state, internal)) {
      RCLCPP_DEBUG(get_logger(), "%s not allowed in %s", g_action_strs[action].c_str(), g_state_strs[state_].c_str());
      return;
    }

    RCLCPP_INFO(get_logger(), "transition to %s", g_state_strs[next_state].c_str());
    state_ = next_state;
  }

  void tello_response_callback(tello_msgs::msg::TelloResponse::SharedPtr msg)
  {
    ActionMgr::State result = action_mgr_->complete(msg);
    if (result == ActionMgr::State::succeeded) {
      transition_state(action_mgr_->action());
    } else if (result == ActionMgr::State::failed_lost_connection) {
      transition_state(Action::disconnect);
    }
  }

  void flight_data_callback(tello_msgs::msg::FlightData::SharedPtr msg)
  {
    if (state_ == State::unknown) {
      RCLCPP_INFO(get_logger(), "receiving flight data");
      transition_state(Action::connect);
    }
  }

  void joy_callback(sensor_msgs::msg::Joy::SharedPtr msg)
  {
    if (msg->buttons[joy_button_takeoff_]) {
      start_action(Action::takeoff);
    } else if (msg->buttons[joy_button_land_]) {
      start_action(Action::land);
    } else if (msg->buttons[joy_button_start_mission_]) {
      start_action(Action::start_mission);
    } else if (msg->buttons[joy_button_stop_mission_]) {
      start_action(Action::stop_mission);
    }

    // TODO trim

    twist_.linear.x = msg->axes[joy_axis_throttle_];
    twist_.linear.y = msg->axes[joy_axis_strafe_];
    twist_.linear.z = msg->axes[joy_axis_vertical_];
    twist_.angular.z = msg->axes[joy_axis_yaw_];
  }

  void spin_once()
  {
    action_mgr_->spin_once();

    // If we're flying manually and the drone isn't busy, send a cmd_vel message
    if (state_ == State::fly_manual && !action_mgr_->busy()) {
      cmd_vel_pub_->publish(twist_);
    }
  }
};

} // namespace flock_base

int main(int argc, char **argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Init ROS
  rclcpp::init(argc, argv);

  // Create node
  auto node = std::make_shared<flock_base::FlockBase>();
  auto result = rcutils_logging_set_logger_level(node->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

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

  // Spin until rclcpp::ok() returns false
  rclcpp::spin(node);

  // Shut down ROS
  rclcpp::shutdown();

  return 0;
}