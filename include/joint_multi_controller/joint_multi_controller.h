#ifndef JOINT_MULTI_CONTROLLER_H
#define JOINT_MULTI_CONTROLLER_H

#include <controller_interface/controller.h>

// #include <effort_controllers/joint_effort_controller.h>
// #include <effort_controllers/joint_position_controller.h>
// #include <effort_controllers/joint_velocity_controller.h>
#include <velocity_controllers/joint_velocity_controller.h>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <hardware_interface/joint_command_interface.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

#include <ros/node_handle.h>
#include <realtime_tools/realtime_buffer.h>

namespace chaser {

  /**
    * JointMultiController has below fuctions.
    * - Subscribe the topic "/chaser/jointN_multi_controller/cmd".
    * -- type: std_msgs::Float64MultiArray data
    * -- data[0]: Controle mode -- STOP, POSTION, VELOCITY, EFFORT -- See enum ControleMode.
    * -- data[1]: Order value. POSTION[rad] or VELOCITY[rad/s] or EFFORT[N]
    * - Set command data to JointPositionController or JointVelocityController or JointEffortController,
    *   and update them.
    */
  
  // template <class T>
  // class JointMultiController : public controller_interface::Controller<T> {
  class JointMultiController : public controller_interface::Controller<hardware_interface::VelocityJointInterface> {

    public:
      enum ControleMode {
        STOP     = 0,
        POSTION  = 1,
        VELOCITY = 2,
        EFFORT   = 3
      };

      JointMultiController();
      ~JointMultiController();

      // Init all JointControllers.
      // bool init(T* hw, ros::NodeHandle &n);
      bool init(hardware_interface::VelocityJointInterface *robot, ros::NodeHandle &n);

      // Start all JointControllers.
      void starting(const ros::Time& time);

      // Update current mode JointController.
      void update(const ros::Time& time, const ros::Duration& period);

    protected:

    private:
      ControleMode m_current_mode;
      std::string m_joint_name;
      ros::Subscriber m_sub_cmd;

      // JointControllers
      // effort_controllers::JointPositionController m_jointPositionController;
      // effort_controllers::JointVelocityController m_jointVelocityController;
      // effort_controllers::JointEffortController   m_jointEffortController;

      velocity_controllers::JointVelocityController m_jointPositionController;
      velocity_controllers::JointVelocityController m_jointVelocityController;
      velocity_controllers::JointVelocityController m_jointEffortController;

      // Topic "/chaser/jointN_multi_controller/cmd" callback.
      // Set command data to the command mode JointController, and change current mode.
      void setCommandCB(const std_msgs::Float64MultiArray msg);
  };

}

#endif /* end of include guard: JOINT_MULTI_CONTROLLER_H */
