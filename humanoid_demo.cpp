#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <memory>
#include <cstdint>
#include <utility>
// ===============================
// 1. Base Motor & Derived Motors
// ===============================
class MotorBase {
protected:
    std::string name_;
    double rated_torque_Nm_;
public:
    MotorBase(std::string name, double rated_torque_Nm)
        : name_(std::move(name)), rated_torque_Nm_(rated_torque_Nm) {}
    virtual ~MotorBase() = default;
    virtual void actuate(double torque) {
        std::cout << "[MotorBase] " << name_
                  << " torque command: " << torque << " Nm\n";
    }
};
class LegMotor : public MotorBase {
public:
    LegMotor(std::string name, double rated_torque_Nm)
        : MotorBase(std::move(name), rated_torque_Nm) {}
    void actuate(double torque) override {
        std::cout << "[LegMotor] " << name_
                  << " applying torque for gait: " << torque << " Nm\n";
    }
};
class ArmMotor : public MotorBase {
public:
    ArmMotor(std::string name, double rated_torque_Nm)
        : MotorBase(std::move(name), rated_torque_Nm) {}
    void actuate(double torque) override {
        std::cout << "[ArmMotor] " << name_
                  << " applying torque for manipulation: " << torque << " Nm\n";
    }
};
// ======================================
// 2. Communication Layers (Inheritance)
// ======================================
class CommunicationBase {
public:
    virtual ~CommunicationBase() = default;
    virtual void transmit(const std::vector<std::uint8_t>& data) = 0;
};
class CANFDInterface : public CommunicationBase {
public:
    void transmit(const std::vector<std::uint8_t>& data) override {
        std::cout << "[CANFD] TX " << data.size()
                  << " bytes to STM32 joint controllers\n";
    }
};
class EtherCATInterface : public CommunicationBase {
public:
    void transmit(const std::vector<std::uint8_t>& data) override {
        std::cout << "[EtherCAT] TX " << data.size()
                  << " bytes to torso/arm drives\n";
    }
};
// ======================================
// 3. ROS2 / microROS Style Integration
// (Lightweight, no external dependencies)
// ======================================
class Ros2Node {
    std::string node_name_;
public:
    explicit Ros2Node(std::string name) : node_name_(std::move(name)) {}
    void spin_once() {
        std::cout << "[ROS2] Node '" << node_name_
                  << "' spinning once (stub).\n";
    }
    void publish(const std::string& topic, const std::string& msg) {
        std::cout << "[ROS2] Publish on '" << topic
                  << "': " << msg << "\n";
    }
};
class MicroRosClient {
    std::string agent_name_;
public:
    explicit MicroRosClient(std::string name) : agent_name_(std::move(name)) {}
    void send_status(const std::string& status) {
        std::cout << "[microROS] Agent '" << agent_name_
                  << "' status: " << status << "\n";
    }
};
// ======================================
// 4. Humanoid Controller (Inheritance)
// ======================================
enum class GaitPhase { STAND, WALK, RUN, JUMP, NAVIGATE };
class ControllerBase {
public:
    virtual ~ControllerBase() = default;
    virtual void execute() = 0;
};
class HumanoidController : public ControllerBase {
    std::unordered_map<std::string, std::unique_ptr<MotorBase>> motors_;
    std::unique_ptr<CommunicationBase> canfd_;
    std::unique_ptr<CommunicationBase> ethercat_;
    Ros2Node ros_node_;
    MicroRosClient micro_ros_;
    GaitPhase phase_{GaitPhase::STAND};
public:
    HumanoidController()
        : canfd_(std::make_unique<CANFDInterface>()),
          ethercat_(std::make_unique<EtherCATInterface>()),
          ros_node_("humanoid_controller"),
          micro_ros_("stm32_leg_agent")
    {
        // Legs
        motors_["left_hip"]   = std::make_unique<LegMotor>("left_hip", 30.0);
        motors_["right_hip"]  = std::make_unique<LegMotor>("right_hip", 30.0);
        motors_["left_knee"]  = std::make_unique<LegMotor>("left_knee", 25.0);
        motors_["right_knee"] = std::make_unique<LegMotor>("right_knee", 25.0);
        // Arms
        motors_["left_shoulder"]  = std::make_unique<ArmMotor>("left_shoulder", 20.0);
        motors_["right_shoulder"] = std::make_unique<ArmMotor>("right_shoulder", 20.0);
    }
    void set_phase(GaitPhase p) {
        phase_ = p;
        std::cout << "[HumanoidController] Gait phase changed.\n";
    }
    void execute() override {
        std::cout << "[HumanoidController] Executing control loop.\n";
        ros_node_.spin_once();
        double base_torque = (phase_ == GaitPhase::WALK) ? 15.0 :
                             (phase_ == GaitPhase::RUN)  ? 25.0 :
                             (phase_ == GaitPhase::JUMP) ? 30.0 : 5.0;
        motors_["left_hip"]->actuate(base_torque);
        motors_["right_hip"]->actuate(base_torque);
        std::vector<std::uint8_t> can_frame{0x10, 0x20, 0x30};
        canfd_->transmit(can_frame);
        std::vector<std::uint8_t> ec_frame{0xAA, 0xBB, 0xCC};
        ethercat_->transmit(ec_frame);
        ros_node_.publish("/humanoid/gait_phase", "phase=" + std::to_string(static_cast<int>(phase_)));
        micro_ros_.send_status("legs nominal, phase=" + std::to_string(static_cast<int>(phase_)));
    }
};
// ======================================
// 5. Demo Main
// ======================================
int main() {
    std::cout << "Starting MultiModule Humanoid Control Demo...\n";
    HumanoidController ctrl;
    ctrl.set_phase(GaitPhase::STAND);
    ctrl.execute();
    ctrl.set_phase(GaitPhase::WALK);
    ctrl.execute();
    ctrl.set_phase(GaitPhase::RUN);
    ctrl.execute();
    return 0;
}