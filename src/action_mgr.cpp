#include "action_mgr.hpp"

// TODO time out if tello_driver hasn't responded in ~10s, this can happen if tello_driver was restarted

namespace drone_base {

ActionMgr::State ActionMgr::send(Action action, std::string action_str)
{
  action_ = action;
  action_str_ = action_str;

  RCLCPP_DEBUG(logger_, "send %s to tello_driver", action_str.c_str());

  auto request = std::make_shared<tello_msgs::srv::TelloAction::Request>();
  request->cmd = action_str;
  future_ = client_->async_send_request(request);

  state_ = ActionMgr::State::waiting_for_future;
  return state_;
}

ActionMgr::State ActionMgr::spin_once()
{
  if (state_ == ActionMgr::State::waiting_for_future &&
    future_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {

    // Get the initial response
    tello_msgs::srv::TelloAction::Response::SharedPtr response = future_.get();

    if (response->rc == response->OK) {
      RCLCPP_DEBUG(logger_, "%s accepted", action_str_.c_str());
      state_ = ActionMgr::State::waiting_for_response;

    } else if (response->rc == response->ERROR_BUSY) {
      RCLCPP_ERROR(logger_, "%s failed, drone is busy", action_str_.c_str());
      result_str_ = "drone is busy";
      state_ = ActionMgr::State::failed;

    } else if (response->rc == response->ERROR_NOT_CONNECTED) {
      RCLCPP_ERROR(logger_, "%s failed, lost connection", action_str_.c_str());
      result_str_ = "lost connection";
      state_ = ActionMgr::State::failed_lost_connection;
    }
  }

  return state_;
}

ActionMgr::State ActionMgr::complete(tello_msgs::msg::TelloResponse::SharedPtr msg)
{
  // The tello_response message may arrive before the future is ready -- that's OK
  if (!busy()) {
    RCLCPP_ERROR(logger_, "unexpected response %s", msg->str.c_str());
    result_str_ = "unexpected response";
    state_ = ActionMgr::State::failed;

  } else if (msg->rc == msg->OK) {
    RCLCPP_DEBUG(logger_, "%s succeeded with %s", action_str_.c_str(), msg->str.c_str());
    result_str_ = msg->str;
    state_ = ActionMgr::State::succeeded;

  } else if (msg->rc == msg->ERROR) {
    RCLCPP_ERROR(logger_, "%s failed with %s", action_str_.c_str(), msg->str.c_str());
    result_str_ = msg->str;
    state_ = ActionMgr::State::failed;

  } else if (msg->rc == msg->TIMEOUT) {
    RCLCPP_ERROR(logger_, "%s failed, drone timed out", msg->str.c_str());
    result_str_ = "drone timed out";
    state_ = ActionMgr::State::failed;
  }

  return state_;
}

} // namespace drone_base