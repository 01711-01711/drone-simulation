#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <thread>
#include <iostream>
#include <memory>
#include <atomic>

namespace gazebo
{
  class JointEffortPlugin : public ModelPlugin
  {
  public:
    JointEffortPlugin() : spinning_(false) {}

    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
    {
      model_ = model;

      if (!sdf->HasElement("joint_name") || !sdf->HasElement("topic_name")) {
        gzerr << "[JointEffortPlugin] Missing <joint_name> or <topic_name> in SDF.\n";
        return;
      }

      joint_name_ = sdf->Get<std::string>("joint_name");
      topic_name_ = sdf->Get<std::string>("topic_name");

      std::cerr << "[JointEffortPlugin] Attempting to load plugin for joint: " << joint_name_ << "\n";

      joint_ = model_->GetJoint(joint_name_);

      if (!joint_) {
        gzerr << "[JointEffortPlugin] Joint not found: " << joint_name_ << "\n";
        return;
      }

      if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
      }

      node_ = std::make_shared<rclcpp::Node>("joint_effort_plugin_" + joint_name_);

      sub_ = node_->create_subscription<std_msgs::msg::Float64>(
        topic_name_, 10,
        [this](std_msgs::msg::Float64::SharedPtr msg) {
          joint_->SetForce(0, msg->data);
          std::cerr << "[JointEffortPlugin] Received effort: " << msg->data << std::endl;
        });

      std::cerr << "[JointEffortPlugin] Subscribed to " << topic_name_
                << " for joint " << joint_name_ << std::endl;

      std::cerr << "[JointEffortPlugin] Loaded plugin for joint: " << joint_name_ << std::endl;

      executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
      executor_->add_node(node_);

      spinning_ = true;
      thread_ = std::thread([this]() {
        while (rclcpp::ok() && spinning_) {
          executor_->spin_some();
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
      });
    }

    void Reset() override
    {
      if (joint_) joint_->SetForce(0, 0.0);
    }

    ~JointEffortPlugin()
    {
      spinning_ = false;
      if (thread_.joinable()) thread_.join();
      if (executor_) executor_->remove_node(node_);
      rclcpp::shutdown();
    }

  private:
    physics::ModelPtr model_;
    physics::JointPtr joint_;
    std::string joint_name_;
    std::string topic_name_;
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_;
    std::thread thread_;
    std::atomic<bool> spinning_;
  };

  GZ_REGISTER_MODEL_PLUGIN(gazebo::JointEffortPlugin)
}
