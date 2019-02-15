#ifndef ACTION_MGR_H
#define ACTION_MGR_H

#include "rclcpp/rclcpp.hpp"
#include "tello_msgs/srv/tello_action.hpp"
#include "tello_msgs/msg/tello_response.hpp"

namespace drone_base {

//==========================
// tello_driver uses a ROS service plus a ROS topic to simulate an action.
// This may be replaced when support for actions is completed in ROS2 Dashing.
//
// Overall flow:
// 1. drone_base sends a TelloAction::Request on the tello_action service
// 2. the drone responds with a TelloAction::Response
// 3. later, the drone sends a TelloResponse message on the tello_response topic
//
// Only one action can be active at a time.
//==========================

enum class Action;

class ActionMgr
{
public:

  enum class State
  {
    not_sent,                    // TelloAction::Request not sent yet
    waiting_for_future,          // Waiting for TelloAction::Response
    waiting_for_response,        // Waiting for TelloResponse
    succeeded,                   // Action succeeded, see the result string
    failed,                      // Action failed, see the result string
    failed_lost_connection,      // Action failed, there's no connection to the drone
  };

private:

  // Init by constructor
  rclcpp::Logger logger_;
  rclcpp::Client<tello_msgs::srv::TelloAction>::SharedPtr client_;
  State state_ = State::not_sent;

  // Init by send()
  Action action_;
  std::string action_str_;
  std::shared_future<tello_msgs::srv::TelloAction::Response::SharedPtr> future_;
  std::string result_str_;

public:

  explicit ActionMgr(rclcpp::Logger logger, rclcpp::Client<tello_msgs::srv::TelloAction>::SharedPtr client):
  logger_{logger}, client_{client}
  {}

  ~ActionMgr()
  {}

  State send(Action action, std::string action_str);
  State spin_once();
  State complete(tello_msgs::msg::TelloResponse::SharedPtr msg);

  Action action() { return action_; }
  std::string action_str() { return action_str_; }
  std::string result_str() { return result_str_; }

  bool busy() { return state_ == State::waiting_for_future || state_ == State::waiting_for_response; }
};

} // namespace drone_base

#endif // ACTION_MGR_H

