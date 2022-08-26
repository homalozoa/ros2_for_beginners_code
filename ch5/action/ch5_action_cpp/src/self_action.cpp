// Copyright (c) 2022 Homalozoa
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cstdlib>
#include <memory>
#include <string>

#include "ch5_action_interfaces/action/count.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class ServerNode : public rclcpp::Node
{
    using CountT = ch5_action_interfaces::action::Count;
    using CountFbT = ch5_action_interfaces::action::Count_Feedback;
    using CountResT = ch5_action_interfaces::action::Count_Result;

public:
    //
    explicit ServerNode(const std::string & node_name)
    : Node(node_name)
    {
        running_ = false;
        this->count_server_ = rclcpp_action::create_server<CountT>(
            this,
            "sec_count",
            std::bind(&ServerNode::goal_handler, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ServerNode::cancel_handler, this, std::placeholders::_1),
            std::bind(&ServerNode::accepted_handler, this, std::placeholders::_1));
    }
private:
    rclcpp_action::Server<CountT>::SharedPtr count_server_;
    bool running_;
    rclcpp_action::GoalResponse goal_handler(
        const rclcpp_action::GoalUUID &,
        std::shared_ptr<const CountT::Goal> goal)
    {
        if (running_) {
            return rclcpp_action::GoalResponse::REJECT;
        }
        running_ = true;
        RCLCPP_INFO_STREAM(this->get_logger(),
            std::string("Goal count is ") << 
            std::to_string(goal->goal_count));
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse cancel_handler(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CountT>> goal_handle)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), std::string("Canceled"));
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void accepted_handler(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<CountT>> goal_handle)
    {
        RCLCPP_INFO_STREAM(this->get_logger(), std::string("Accepted"));
        std::thread{std::bind(&ServerNode::execute, this, std::placeholders::_1), goal_handle}.detach();
    }
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<CountT>> goal_handle)
    {
        auto count_goal = goal_handle->get_goal()->goal_count;
        auto result = std::make_shared<CountT::Result>();
        auto feedback = std::make_shared<CountT::Feedback>();
        rclcpp::Rate rsleep(1);
        while (count_goal > 0 && rclcpp::ok()) {
            rsleep.sleep();
            feedback->local_count += 1;
            //RCLCPP_INFO(this->get_logger(), "flag1");
            goal_handle->publish_feedback(feedback);
            //RCLCPP_INFO(this->get_logger(), "flag2");
            count_goal -= 1;
        }
        result->global_count = feedback->local_count;
        goal_handle->succeed(result);
        running_ = false;
    }
};

class ClientNode : public rclcpp::Node
{
    using CountT = ch5_action_interfaces::action::Count;
    using CountGoalT = ch5_action_interfaces::action::Count::Goal;

public:
    explicit ClientNode(const std::string & node_name)
    : Node(node_name)
    {
        using namespace std::chrono_literals;
        count_client_ = rclcpp_action::create_client<CountT>(
            this,
            "sec_count");
        auto clientimer_cb = 
        [&]() -> void {
            CountGoalT goal;
            uint8_t count = rand() % 5;
            goal.goal_count = (count > 0) ? count : 1;
            auto send_options = rclcpp_action::Client<CountT>::SendGoalOptions();
            send_options.goal_response_callback = std::bind(&ClientNode::goal_response_callback, this, std::placeholders::_1);
            send_options.feedback_callback = std::bind(&ClientNode::feedback_response_callback, this, std::placeholders::_1, std::placeholders::_2);
            send_options.result_callback = std::bind(&ClientNode::result_callback, this, std::placeholders::_1);
            RCLCPP_INFO_STREAM(this->get_logger(),
                std::string("-------------"));
            RCLCPP_INFO_STREAM(this->get_logger(),
                std::string("Sent goal: ") <<
                std::to_string(goal.goal_count));
            count_client_->async_send_goal(goal, send_options);
        };
        clientimer_ = this->create_wall_timer(3s, clientimer_cb);
    }
private:
    rclcpp_action::Client<CountT>::SharedPtr count_client_;
    rclcpp::TimerBase::SharedPtr clientimer_;

    void goal_response_callback(
        const rclcpp_action::ClientGoalHandle<CountT>::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_WARN_STREAM(this->get_logger(),
                std::string("Goal was rejected by server."));
            RCLCPP_INFO(this->get_logger(),
                "-------------");
        } else {
            RCLCPP_INFO_STREAM(this->get_logger(),
                std::string("Goal was accepted."));
        } 
    }
    void feedback_response_callback(
        rclcpp_action::ClientGoalHandle<CountT>::SharedPtr goal_handle,
        const std::shared_ptr<const CountT::Feedback> feedback)
    {
        RCLCPP_INFO_STREAM(this->get_logger(),
            std::string("Feedback: ") <<
            std::to_string(feedback->local_count));
    }
    void result_callback(
        const rclcpp_action::ClientGoalHandle<CountT>::WrappedResult & result)
    {
        RCLCPP_INFO_STREAM(this->get_logger(),
            std::string("Result: ") <<
            std::to_string(result.result->global_count));
        RCLCPP_INFO_STREAM(this->get_logger(),
            std::string("-------------"));
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto srv_node_ = std::make_shared<ServerNode>("action_server");
    auto cli_node_ = std::make_shared<ClientNode>("action_client");
    auto exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    exec_->add_node(srv_node_->get_node_base_interface());
    exec_->add_node(cli_node_->get_node_base_interface());
    exec_->spin();
    
    rclcpp::shutdown();
    return 0;
}
