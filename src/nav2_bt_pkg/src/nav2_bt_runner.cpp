#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_bt_pkg/server_handler.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_ros/transform_listener.h"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

class BehaviorTreeHandler
{
  public:
    BehaviorTreeHandler()
    {
        node_ = rclcpp::Node::make_shared("behavior_tree_handler");

        tf_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        auto timer_interface =
            std::make_shared<tf2_ros::CreateTimerROS>(node_->get_node_base_interface(), node_->get_node_timers_interface());
        tf_->setCreateTimerInterface(timer_interface);
        tf_->setUsingDedicatedThread(true);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, node_, false);

        odom_smoother_ = std::make_shared<nav2_util::OdomSmoother>(node_);

        const std::vector<std::string> plugin_libs = {"nav2_compute_path_to_pose_action_bt_node",
                                                      "nav2_compute_path_through_poses_action_bt_node",
                                                      "nav2_smooth_path_action_bt_node",
                                                      "nav2_follow_path_action_bt_node",
                                                      "nav2_spin_action_bt_node",
                                                      "nav2_wait_action_bt_node",
                                                      "nav2_assisted_teleop_action_bt_node",
                                                      "nav2_back_up_action_bt_node",
                                                      "nav2_drive_on_heading_bt_node",
                                                      "nav2_clear_costmap_service_bt_node",
                                                      "nav2_is_stuck_condition_bt_node",
                                                      "nav2_goal_reached_condition_bt_node",
                                                      "nav2_initial_pose_received_condition_bt_node",
                                                      "nav2_goal_updated_condition_bt_node",
                                                      "nav2_globally_updated_goal_condition_bt_node",
                                                      "nav2_is_path_valid_condition_bt_node",
                                                      "nav2_reinitialize_global_localization_service_bt_node",
                                                      "nav2_rate_controller_bt_node",
                                                      "nav2_distance_controller_bt_node",
                                                      "nav2_speed_controller_bt_node",
                                                      "nav2_truncate_path_action_bt_node",
                                                      "nav2_truncate_path_local_action_bt_node",
                                                      "nav2_goal_updater_node_bt_node",
                                                      "nav2_recovery_node_bt_node",
                                                      "nav2_pipeline_sequence_bt_node",
                                                      "nav2_round_robin_node_bt_node",
                                                      "nav2_transform_available_condition_bt_node",
                                                      "nav2_time_expired_condition_bt_node",
                                                      "nav2_path_expiring_timer_condition",
                                                      "nav2_distance_traveled_condition_bt_node",
                                                      "nav2_single_trigger_bt_node",
                                                      "nav2_is_battery_low_condition_bt_node",
                                                      "nav2_navigate_through_poses_action_bt_node",
                                                      "nav2_navigate_to_pose_action_bt_node",
                                                      "nav2_remove_passed_goals_action_bt_node",
                                                      "nav2_planner_selector_bt_node",
                                                      "nav2_controller_selector_bt_node",
                                                      "nav2_goal_checker_selector_bt_node",
                                                      "nav2_controller_cancel_bt_node",
                                                      "nav2_path_longer_on_approach_bt_node",
                                                      "nav2_assisted_teleop_cancel_bt_node",
                                                      "nav2_wait_cancel_bt_node",
                                                      "nav2_spin_cancel_bt_node",
                                                      "nav2_back_up_cancel_bt_node",
                                                      "nav2_drive_on_heading_cancel_bt_node",
                                                      "nav2_goal_updated_controller_bt_node",
                                                      "nav2_talker_action_bt_node" // customized action BT plugin
                                                      };
        for(const auto& p : plugin_libs)
        {
            factory_.registerFromPlugin(BT::SharedLibrary::getOSName(p));
        }
    }

    ~BehaviorTreeHandler()
    {
    }

    bool loadBehaviorTree(const std::string& filename)
    {
        // Read the input BT XML from the specified file into a string
        std::ifstream xml_file(filename);

        if(!xml_file.good())
        {
            RCLCPP_ERROR(node_->get_logger(), "Couldn't open input XML file: %s", filename.c_str());
            return false;
        }

        auto xml_string = std::string(std::istreambuf_iterator<char>(xml_file), std::istreambuf_iterator<char>());

        // Create the blackboard that will be shared by all of the nodes in the tree
        blackboard = BT::Blackboard::create();

        // Put items on the blackboard
        blackboard->set<rclcpp::Node::SharedPtr>("node", node_); // NOLINT
        blackboard->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(20)); // NOLINT
        blackboard->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(10)); // NOLINT
        blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_); // NOLINT
        blackboard->set<bool>("initial_pose_received", false); // NOLINT
        blackboard->set<int>("number_recoveries", 0); // NOLINT
        blackboard->set<std::shared_ptr<nav2_util::OdomSmoother>>("odom_smoother", odom_smoother_); // NOLINT

        // set dummy goal on blackboard
        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp = node_->now();
        goal.header.frame_id = "map";
        goal.pose.position.x = 0.0;
        goal.pose.position.y = 0.0;
        goal.pose.position.z = 0.0;
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = 0.0;
        goal.pose.orientation.w = 1.0;

        blackboard->set<geometry_msgs::msg::PoseStamped>("goal", goal); // NOLINT

        // Create the Behavior Tree from the XML input
        try
        {
            tree = factory_.createTreeFromText(xml_string, blackboard);
        }
        catch(BT::RuntimeError& exp)
        {
            RCLCPP_ERROR(node_->get_logger(), "%s: %s", filename.c_str(), exp.what());
            return false;
        }

        return true;
    }

    BT::Blackboard::Ptr blackboard;
    BT::Tree tree;

  private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother_;
    BT::BehaviorTreeFactory factory_;
};

void tickBehaviorTree(const std::string& filepath)
{
    auto server_handler = std::make_shared<nav2_bt_pkg::ServerHandler>();
    if(!server_handler->isActive()) { server_handler->activate(); }
    auto bt_handler = std::make_shared<BehaviorTreeHandler>();
    auto bt_file_res = bt_handler->loadBehaviorTree(filepath);
    if(!bt_file_res) { std::cout << "Cannot load the behavior tree!" << std::endl; }

    BT::NodeStatus result{BT::NodeStatus::RUNNING};
    while(result == BT::NodeStatus::RUNNING)
    {
        std::cout << "Node status: " << result << std::endl;
        result = bt_handler->tree.tickRoot();
        std::this_thread::sleep_for(10ms);
    }

    std::cout << "Node final status: " << result << std::endl;

    server_handler->deactivate();
}

int main(int argc, char** argv)
{
    rclcpp::init(0, nullptr);

    fs::path bt_filepath = ament_index_cpp::get_package_share_directory("nav2_bt_pkg");
    bt_filepath /= fs::path("behavior_trees");
    std::string xml_filename{"navigate_to_pose_w_replanning_and_recovery.xml"};
    std::cout << "argc: " << argc << std::endl;
    if(argc > 1) { xml_filename.assign(argv[1]); }
    bt_filepath /= fs::path(xml_filename);
    std::cout << "XML Path: " << bt_filepath << std::endl;

    tickBehaviorTree(bt_filepath.string());

    rclcpp::shutdown();

    return 0;
}
