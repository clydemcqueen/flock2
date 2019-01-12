#include "drone.hpp"

#include "flock_base.hpp"

namespace flock_base {

std::map<State, std::string> g_state_strs{
  {State::unknown, "unknown"},
  {State::landed, "landed"},
  {State::fly_manual, "fly_manual"},
  {State::fly_mission, "fly_mission"},
};

std::map<Action, std::string> g_action_strs{
  {Action::takeoff, "takeoff"},
  {Action::land, "land"},
  {Action::start_mission, "start_mission"},
  {Action::stop_mission, "stop_mission"},
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
    Transition{State::fly_manual, Action::disconnect, State::unknown, false},
    Transition{State::fly_mission, Action::disconnect, State::unknown, false},

    // Take off / land
    Transition{State::landed, Action::takeoff, State::fly_manual, true},
    Transition{State::fly_manual, Action::land, State::landed, true},

    // Start / stop mission
    Transition{State::fly_manual, Action::start_mission, State::fly_mission, false},
    Transition{State::fly_mission, Action::stop_mission, State::fly_manual, false},
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

Drone::Drone(FlockBase *node) : node_{node}
{
  action_mgr_ = std::make_unique<ActionMgr>(node_->get_logger(),
    node_->create_client<tello_msgs::srv::TelloAction>("tello_action"));

  start_mission_pub_ = node_->create_publisher<std_msgs::msg::Empty>("start_mission", 1);
  stop_mission_pub_ = node_->create_publisher<std_msgs::msg::Empty>("stop_mission", 1);
  cmd_vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);

  auto tello_response_cb = std::bind(&Drone::tello_response_callback, this, std::placeholders::_1);
  tello_response_sub_ = node_->create_subscription<tello_msgs::msg::TelloResponse>("tello_response", tello_response_cb);

  auto flight_data_cb = std::bind(&Drone::flight_data_callback, this, std::placeholders::_1);
  flight_data_sub_ = node_->create_subscription<tello_msgs::msg::FlightData>("flight_data", flight_data_cb);
}

void Drone::start_action(Action action)
{
  if (action_mgr_->busy()) {
    RCLCPP_DEBUG(node_->get_logger(), "busy, dropping %s", g_action_strs[action].c_str());
    return;
  }

  State next_state;
  bool send_to_drone;
  if (!valid_transition(state_, action, next_state, send_to_drone)) {
    RCLCPP_DEBUG(node_->get_logger(), "%s not allowed in %s", g_action_strs[action].c_str(), g_state_strs[state_].c_str());
    return;
  }

  if (send_to_drone) {
    RCLCPP_INFO(node_->get_logger(), "initiating %s", g_action_strs[action].c_str());
    action_mgr_->send(action, g_action_strs[action]);

  } else {
    if (action == Action::start_mission) {
      start_mission_pub_->publish(std_msgs::msg::Empty());
    } else if (action == Action::stop_mission) {
      stop_mission_pub_->publish(std_msgs::msg::Empty());
    }

    transition_state(action);
  }
}

void Drone::transition_state(Action action)
{
  State next_state;
  bool send_to_drone;
  if (!valid_transition(state_, action, next_state, send_to_drone)) {
    RCLCPP_DEBUG(node_->get_logger(), "%s not allowed in %s", g_action_strs[action].c_str(), g_state_strs[state_].c_str());
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "transition to %s", g_state_strs[next_state].c_str());
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
  (void)msg;

  if (state_ == State::unknown) {
    RCLCPP_INFO(node_->get_logger(), "receiving flight data");
    transition_state(Action::connect);
  }
}

} // namespace flock_base