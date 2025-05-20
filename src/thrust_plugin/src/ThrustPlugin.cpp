#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include <ignition/math/Vector3.hh>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <memory>
#include <unordered_map>
#include <vector>
#include <string>

namespace gazebo
{
  class ThrustPlugin : public ModelPlugin
  {
  public:
    void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
    {
      try
      {
        model_ = model;
        node_ = std::make_shared<rclcpp::Node>("thrust_plugin_node");
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node_);
        std::thread([&executor]() { executor.spin(); }).detach();

        if (!sdf->HasElement("rotors"))
        {
          gzerr << "[ThrustPlugin] No <rotors> blocks found in plugin SDF.\n";
          return;
        }

        sdf::ElementPtr rotorElem = sdf->GetElement("rotors");

        while (rotorElem)
        {
          // Safety checks for each field
          if (!rotorElem->HasElement("linkName") ||
              !rotorElem->HasElement("topicName") ||
              !rotorElem->HasElement("thrustCoefficient") ||
              !rotorElem->HasElement("torqueCoefficient") ||
              !rotorElem->HasElement("direction") ||
              !rotorElem->HasElement("maxForce"))
          {
            gzerr << "[ThrustPlugin] Missing one or more required parameters in a <rotors> block.\n";
            rotorElem = rotorElem->GetNextElement("rotors");
            continue;
          }

          // Read parameters
          std::string linkName = rotorElem->Get<std::string>("linkName");
          std::string topicName = rotorElem->Get<std::string>("topicName");
          double thrustCoeff = rotorElem->Get<double>("thrustCoefficient");
          double torqueCoeff = rotorElem->Get<double>("torqueCoefficient");
          double direction = rotorElem->Get<double>("direction");
          double maxForce = rotorElem->Get<double>("maxForce");

          // Optional PID
          double p = rotorElem->HasElement("p") ? rotorElem->Get<double>("p") : 1.5;
          double i = rotorElem->HasElement("i") ? rotorElem->Get<double>("i") : 0.0;
          double d = rotorElem->HasElement("d") ? rotorElem->Get<double>("d") : 0.1;

          physics::LinkPtr link = model_->GetLink(linkName);
          if (!link)
          {
            gzerr << "[ThrustPlugin] Link not found: " << linkName << "\n";
            rotorElem = rotorElem->GetNextElement("rotors");
            continue;
          }

          // Capture parameters for this rotor
          auto callback = [link, thrustCoeff, torqueCoeff, direction, maxForce, p, i, d]
              (const std_msgs::msg::Float64::SharedPtr msg)
          {
            double input = msg->data;
            double thrust = std::clamp(input * thrustCoeff * direction, -maxForce, maxForce);
            double torque = std::clamp(input * torqueCoeff * direction, -maxForce, maxForce);

            link->AddRelativeForce(ignition::math::Vector3d(0, 0, thrust));
            link->AddRelativeTorque(ignition::math::Vector3d(0, 0, torque));
          };

          // Subscribe
          auto sub = node_->create_subscription<std_msgs::msg::Float64>(topicName, 10, callback);
          subscribers_.push_back(sub);

          gzdbg << "[ThrustPlugin] Subscribed to " << topicName << " for link " << linkName << "\n";

          rotorElem = rotorElem->GetNextElement("rotors");
        }

        gzdbg << "[ThrustPlugin] Plugin loaded with " << subscribers_.size() << " rotors.\n";
      }
      catch (const std::exception &e)
      {
        gzerr << "[ThrustPlugin] Exception during Load(): " << e.what() << "\n";
      }
    }

  private:
    physics::ModelPtr model_;
    rclcpp::Node::SharedPtr node_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> subscribers_;
  };

  GZ_REGISTER_MODEL_PLUGIN(ThrustPlugin)
}
