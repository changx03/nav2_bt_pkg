#include "nav2_behavior_tree/bt_action_node.hpp"
#include "nav2_msgs/action/dummy_behavior.hpp"

#include <memory>
#include <string>

namespace nav2_behavior_tree
{

class TalkerAction : public BtActionNode<nav2_msgs::action::DummyBehavior>
{
  public:
    TalkerAction(const std::string& xml_tag_name, const std::string& action_name, const BT::NodeConfiguration& conf)
        : BtActionNode(xml_tag_name, action_name, conf)
    {
    }

    void on_tick() override
    {
        std::string msg;
        getInput("msg", msg);
        RCLCPP_INFO(node_->get_logger(), msg.c_str());
    }

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({BT::InputPort<std::string>("msg")});
    }

  private:
};

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"

BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config)
    {
        // This action_name must be in the ServerHandler
        return std::make_unique<nav2_behavior_tree::TalkerAction>(name, "talker", config);
    };

    factory.registerBuilder<nav2_behavior_tree::TalkerAction>("Talker", builder);
}
