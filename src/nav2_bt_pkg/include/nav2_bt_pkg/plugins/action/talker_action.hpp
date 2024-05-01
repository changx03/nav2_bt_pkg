#pragma once

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/dummy_behavior.hpp"

#include <string>

namespace nav2_bt_pkg
{
class TalkerAction : public nav2_behavior_tree::BtActionNode<nav2_msgs::action::DummyBehavior>
{
  public:
    TalkerAction(const std::string& xml_tag_name, const std::string& action_name, const BT::NodeConfiguration& conf);

    void on_tick() override;

    void initialize();

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({BT::InputPort<std::string>("msg")});
    }

  private:
    bool initialized_;
};
} // namespace nav2_bt_pkg
