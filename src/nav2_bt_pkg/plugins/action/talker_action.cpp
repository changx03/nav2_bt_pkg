#include "nav2_bt_pkg/plugins/action/talker_action.hpp"

#include "rclcpp/rclcpp.hpp"

namespace nav2_bt_pkg
{

TalkerAction::TalkerAction(const std::string& xml_tag_name, const std::string& action_name, const BT::NodeConfiguration& conf)
    : BtActionNode<nav2_msgs::action::DummyBehavior>(xml_tag_name, action_name, conf),
      initialized_(false)
{
}

void TalkerAction::initialize()
{
    initialized_ = true;
}

void TalkerAction::on_tick()
{
    if(!initialized_) { initialize(); }

    std::string msg;
    getInput("msg", msg);

    RCLCPP_INFO_STREAM(node_->get_logger(), msg);
}

} // namespace nav2_bt_pkg

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config)
    { return std::make_unique<nav2_bt_pkg::TalkerAction>(name, "talker", config); };

    factory.registerBuilder<nav2_bt_pkg::TalkerAction>("Talker", builder);
}
