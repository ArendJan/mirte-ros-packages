#pragma once
#include <memory>

#include <rclcpp/node.hpp>

#include <tmx_cpp/tmx.hpp>
#include <vector>

class Mirte_Board;

struct TMXTimer {
  std::chrono::duration<double, std::milli> duration;
  std::shared_ptr<rclcpp::TimerBase> timer;
  std::shared_ptr<rclcpp::CallbackGroup> callback_group;
  std::vector<std::function<void()>> callbacks;
};

struct NodeData {
  std::shared_ptr<rclcpp::Node> nh;
  std::shared_ptr<tmx_cpp::TMX> tmx;
  std::shared_ptr<Mirte_Board> board;
  std::function<void(std::chrono::duration<double, std::milli>,
                     std::function<void()>)>
      add_timer;
  // timers: vector of duration and vector of callback, one ros timer per
  // duration
  std::vector<TMXTimer> timers;
};
