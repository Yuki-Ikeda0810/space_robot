#include <pluginlib/class_list_macros.h>
#include "joint_multi_controller/joint_multi_controller.h"

namespace chaser
{
  JointMultiController::JointMultiController() {
    m_current_mode = ControleMode::VELOCITY;
  }

  JointMultiController::~JointMultiController() {
  }

  // JointMultiController() {};
  // ~JointMultiController() {sub_command_.shutdown();};

  bool JointMultiController::init(
      // T* hw, ros::NodeHandle &n) {
      hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n) {
      // hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n) {
    ROS_INFO("[JointMultiController] init start");

    // Get joint name from parameter server.
    bool isRet = false;
    isRet = n.getParam("joint", m_joint_name);
    if (!isRet) {
      ROS_ERROR("[JointMultiController] No joint given (namespace: %s)", n.getNamespace().c_str());
      return false;
    }
    ROS_INFO("[JointMultiController] joint:%s", m_joint_name.c_str());

    // Init JointPositionController.
    // isRet = m_jointPositionController.init(hw, n);
    isRet = m_jointPositionController.init(robot, n);
    if (!isRet) {
      ROS_ERROR("[JointMultiController] Failed to init JointPositionController");
      return false;
    }

    // Init JointVelocityController.
    // To avoid below error, don't call m_jointPositionController.init(robot, n).
    // > Tried to advertise a service that is already advertised in this node
    // > [/chaser/jointn_chaser_controller/pid/set_parameters]
    // control_toolbox::Pid pid_controller;
    // Load PID Controller using gains set on parameter server
    // isRet = pid_controller.init(ros::NodeHandle(n, "pid_v"));
    // if (!isRet) {
      // ROS_ERROR("[JointMultiController] Failed to init Pid");
      // return false;
    // }
    
    // isRet = m_jointVelocityController.init(hw, n);
    isRet = m_jointVelocityController.init(robot, n);
    // isRet = m_jointVelocityController.init(robot, m_joint_name, pid_controller);
    if (!isRet) {
      ROS_ERROR("[JointMultiController] Failed to init JointVelocityController");
      return false;
    }

    // Init JointEffortController
    // isRet = m_jointEffortController.init(hw, n);
    isRet = m_jointEffortController.init(robot, n);
    if (!isRet) {
      ROS_ERROR("[JointMultiController] Failed to init JointEffortController");
      return false;
    }

    // Start command subscriber
    m_sub_cmd = n.subscribe<std_msgs::Float64MultiArray>(
        "cmd", 1, &JointMultiController::setCommandCB, this);

    ROS_INFO("[JointMultiController] init end");
    return true;
  }

  void JointMultiController::starting(const ros::Time& time) {
    ROS_INFO("[JointMultiController] starting start  joint:%s", m_joint_name.c_str());

    // Start controllers.
    m_jointPositionController.starting(time);
    m_jointVelocityController.starting(time);
    m_jointEffortController.starting(time);

    ROS_INFO("[JointMultiController] starting end");
  }

  void JointMultiController::update(const ros::Time& time, const ros::Duration& period) {
//    ROS_INFO("[JointMultiController] update start  joint:%s mode:%d",
//        m_joint_name.c_str(), m_current_mode);

    // Update controller.
    switch (m_current_mode) {
      case ControleMode::POSTION:
        m_jointPositionController.update(time, period);
        break;
      case ControleMode::VELOCITY:
        m_jointVelocityController.update(time, period);
        break;
      case ControleMode::EFFORT:
        m_jointEffortController.update(time, period);
        break;
      default:
        ROS_ERROR("[JointMultiController] Current mode is invalid.");
        break;
    }
    
//    ROS_INFO("[JointMultiController] update end");
  }

  void JointMultiController::setCommandCB(const std_msgs::Float64MultiArray msg) {
//    ROS_INFO("[JointMultiController] setCommandCB start  joint:%s", m_joint_name.c_str());

    // Check parameter.
    unsigned int command_mode = 0;
    double command_data = 0.0;
    if (msg.data.size() >= 1) {
      command_mode = (unsigned int)msg.data[0];
//      ROS_INFO("[JointMultiController] mode:%d", command_mode);
      if (msg.data.size() == 1) {
        if (command_mode == ControleMode::STOP) {
          /* NOP */
        } else {
          ROS_ERROR("[JointMultiController] message data format is invalid.");
          return;
        }
      } else {
        command_data = msg.data[1];
//        ROS_INFO("[JointMultiController] data:%f", command_data);
      }
    } else {
      ROS_ERROR("[JointMultiController] message data size is invalid.");
      return;
    }

    // Check command mode and current mode.
    unsigned int tmp_command_mode = command_mode;
    if (tmp_command_mode == ControleMode::STOP) {
      tmp_command_mode = ControleMode::VELOCITY;  // Use JointVelocityController for stop.
    }
    bool isChangeController = false;
    if (tmp_command_mode != m_current_mode) {
//      ROS_INFO("[JointMultiController] Controller is changed.");
      isChangeController = true;
    }

    
    // Set command to controller.
    // If controller is changed, init controller with starting().
    // Because pid_controller needs init.
    const ros::Time time; // Dummy time for calling starting(). This value is not used.
    switch (command_mode) {
      case ControleMode::STOP:
         // Use JointVelocityController for stop.
        m_current_mode = ControleMode::VELOCITY;
        if (isChangeController) m_jointVelocityController.starting(time);
        m_jointVelocityController.command_buffer_.writeFromNonRT(0.0);
        // m_jointVelocityController.setCommand(0.0);  // Set velocity 0.0[rad/s] for stop.
        break;
      case ControleMode::POSTION:
        m_current_mode = ControleMode::POSTION;
        if (isChangeController) m_jointPositionController.starting(time);
        m_jointPositionController.command_buffer_.writeFromNonRT(command_data);
        // m_jointPositionController.setCommand(command_data);
        break;
      case ControleMode::VELOCITY:
        m_current_mode = ControleMode::VELOCITY;
        if (isChangeController) m_jointVelocityController.starting(time);
        m_jointVelocityController.command_buffer_.writeFromNonRT(command_data);
        //m_jointVelocityController.setCommand(command_data);
        break;
      case ControleMode::EFFORT:
        m_current_mode = ControleMode::EFFORT;
        if (isChangeController) m_jointEffortController.starting(time);
        m_jointEffortController.command_buffer_.writeFromNonRT(command_data);
        break;
      default:
        ROS_ERROR("[JointMultiController] Command mode is invalid.");
        break;
    }

//    ROS_INFO("[JointMultiController] setCommandCB end");
  }
}

// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS(chaser::JointMultiController, controller_interface::ControllerBase)