
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "rms_msgs/action/point_cloud_collection.hpp"
#include "rms_msgs/action/point_cloud_registration.hpp"

using Collection = rms_msgs::action::PointCloudCollection;
using Registration = rms_msgs::action::PointCloudRegistration;

class RMSMain : public rclcpp::Node
{
public:
    explicit RMSMain(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("rms_main", options)
    {
        using namespace std::placeholders;

        std::vector<std::string>> manipulators;
        this->declare_parameter<std::vector<std::string>>("manipulators", {"vx250", "vx300s"});
        this->get_parameter("manipulators", manipulators);

        std::string _collection_action;
        this->declare_parameter<std::string>("collection_action", "collect_pointclouds_at_viewpoints")
        this->get_parameter("collection_action", _collection_action)

        for (const auto & manipulator : manipulators)
        {
            auto _collection_client = rclcpp_action::create_client<Collection>(this, _collection_action);

            _collection_clients_.emplace(manipulator, collection_client);
            pending_manipulators_.emplace(manipulator);

            _collection_client->on_action_goal_response(
                [this, manipulator](const typename Collection::GoalHandle::SharedPtr & goal_handle)
                {
                    this->_collection_goal_response_callback(manipulator, goal_handle);
                }
            );

            _collection_client->on_action_feedback(
                [this, manipulator](const typename Collection::GoalHandle::SharedPtr & goal_handle, const typename Collection::Feedback::SharedPtr & feedback)
                {
                    this->_collection_feedback_callback(manipulator, goal_handle, feedback);
                }
            );

            collection_client->on_action_result(
                [this, manipulator](const typename Collection::GoalHandle::SharedPtr & goal_handle, const typename Collection::Result::SharedPtr & result) 
                {
                    this->_collection_result_callback(manipulator, goal_handle, result);
                }
            );
        }
        
    }
private:
    std::unordered_map<std::string, rclcpp_action::Client<Collection>::SharedPtr> _collection_clients_;
    std::set<std::string> pending_manipulators_;
    // std::unordered_map<std::string, rclcpp_action::Client<Registration>::SharedPtr> _registration_clients_;

    void _collection_goal_response_callback(const std::string & manipulator, const typename Collection::GoalHandle::SharedPtr & goal_handle)
    {
        if (!goal_handle) 
        {
            RCLCPP_ERROR(this->get_logger(), "goal was rejected by server for manipulator %s", manipulator.c_str());
        } 
        else 
        {
            RCLCPP_INFO(this->get_logger(), "goal accepted by server, waiting for result for manipulator %s", manipulator.c_str());
        }
    }

    void _collection_feedback_callback(const std::string & manipulator, const typename Collection::GoalHandle::SharedPtr & goal_handle, const typename Collection::Feedback::SharedPtr & feedback)
    {
        double progress = feedback->progress;  
        RCLCPP_INFO(this->get_logger(), "%s: %.2f%%", manipulator.c_str(), progress);
    }

    void _collection_result_callback(const std::string & manipulator, const typename Collection::GoalHandle::SharedPtr & goal_handle, const typename Collection::Result::SharedPtr & result)
    {
        bool success = result->success;
        RCLCPP_INFO(this->get_logger(), "%s: success = %d", manipulator.c_str(), success);

        pending_manipulators_.erase(manipulator);

        if (pending_manipulators_.empty())
        {
            RCLCPP_INFO(this->get_logger(), "all manipulators have completed their pointcloud collection");
        }
    }



};


RCLCPP_COMPONENTS_REGISTER_NODE(RMSMain)