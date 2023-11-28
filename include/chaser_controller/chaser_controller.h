#ifndef CHASER_CONTROLLER_H
#define CHASER_CONTROLLER_H

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

namespace gazebo {

  /**
    * ChaserController has below fuctions.
    * - Subscribe the topic "/chaser/cmd_armN".
    * - Publish the topic "/chaser/jointN_multi_controller/cmd" with received "/chaser/cmd_armN" data.
    * - Publish the topic "/chaser/obs_armN".
    */
  class ChaserController : public ModelPlugin {

    public: 
      ChaserController();
      ~ChaserController();

      // Load the controller
      void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

    protected:
      // Update the controller called at every step
      virtual void UpdateChild();
      // Finalize the controller
      virtual void FiniChild();

    private:
      static const int JOINT_NUM     = 3;    // old:7
      static const int END_JOINT_NUM = 0;    // old:3
      static const int CMD_ARM_SIZE  = 3;
      static const int QUEUE_SIZE    = 1;

      // Topics "/chaser/cmd_armN"
      const std::string TOPIC_CMD_ARM_1 = "/chaser/cmd_arm1";
      const std::string TOPIC_CMD_ARM_2 = "/chaser/cmd_arm2";
      // const std::string TOPIC_CMD_ARM_3 = "/chaser/cmd_arm3";
      // const std::string TOPIC_CMD_ARM_4 = "/chaser/cmd_arm4";
      // const std::string TOPIC_CMD_ARM_5 = "/chaser/cmd_arm5";

      // Topics "/chaser/obs_armN"
      const std::string TOPIC_OBS_ARM_1 = "/chaser/obs_arm1";
      const std::string TOPIC_OBS_ARM_2 = "/chaser/obs_arm2";
      // const std::string TOPIC_OBS_ARM_3 = "/chaser/obs_arm3";
      // const std::string TOPIC_OBS_ARM_4 = "/chaser/obs_arm4";
      // const std::string TOPIC_OBS_ARM_5 = "/chaser/obs_arm5";

      // jointN_multi_controller topics
      const std::string TOPIC_CMD_CNT_1 = "/chaser/joint1_multi_controller/cmd";
      const std::string TOPIC_CMD_CNT_2 = "/chaser/joint2_multi_controller/cmd";
      const std::string TOPIC_CMD_CNT_3 = "/chaser/joint3_multi_controller/cmd";
      // const std::string TOPIC_CMD_CNT_4 = "/chaser/joint4_multi_controller/cmd";
      // const std::string TOPIC_CMD_CNT_5 = "/chaser/joint5_multi_controller/cmd";
      // const std::string TOPIC_CMD_CNT_6 = "/chaser/joint6_multi_controller/cmd";
      // const std::string TOPIC_CMD_CNT_7 = "/chaser/joint7_multi_controller/cmd";
      // const std::string TOPIC_CMD_CNT_8 = "/chaser/finger_joint1_controller/cmd";
      // const std::string TOPIC_CMD_CNT_9 = "/chaser/finger_joint2_controller/cmd";
      // const std::string TOPIC_CMD_CNT_10 = "/chaser/finger_joint3_controller/cmd";

      // Joint names
      const std::string JOINT_NAME_1 = "arm_joint_1";
      const std::string JOINT_NAME_2 = "arm_joint_2";
      const std::string JOINT_NAME_3 = "arm_joint_3";
      // const std::string JOINT_NAME_4 = "arm_joint_4";
      // const std::string JOINT_NAME_5 = "arm_joint_5";
      // const std::string JOINT_NAME_6 = "arm_joint_6";
      // const std::string JOINT_NAME_7 = "arm_joint_7";
      // const std::string END_JOINT_NAME_1  = "end_effector_finger_joint_1";
      // const std::string END_JOINT_NAME_2  = "end_effector_finger_joint_2";
      // const std::string END_JOINT_NAME_3  = "end_effector_finger_joint_3";

      boost::shared_ptr<ros::NodeHandle> m_rosnode;
      ros::Subscriber m_sub_1;
      ros::Subscriber m_sub_2;
      // ros::Subscriber m_sub_3;
      // ros::Subscriber m_sub_4;
      // ros::Subscriber m_sub_5;
      ros::Publisher m_pub_1;
      ros::Publisher m_pub_2;
      // ros::Publisher m_pub_3;
      // ros::Publisher m_pub_4;
      // ros::Publisher m_pub_5;
      ros::Publisher m_pub_cnt[JOINT_NUM + END_JOINT_NUM];
      physics::JointPtr m_joints[JOINT_NUM];
      // physics::JointPtr m_end_joints[END_JOINT_NUM];
      event::ConnectionPtr m_update_connection;
      boost::mutex m_lock;

      // Topic "/chaser/cmd_armN" callbacks.
      // Publish contorl data to JointController.
      void CmdArm1Callback(const std_msgs::Float64MultiArray cmd);
      void CmdArm2Callback(const std_msgs::Float64MultiArray cmd);
      // void CmdArm3Callback(const std_msgs::Float64MultiArray cmd);
      // void CmdArm4Callback(const std_msgs::Float64MultiArray cmd);
      // void CmdArm5Callback(const std_msgs::Float64MultiArray cmd);

      // Publish contorl data to JointController.
      void PublishToJointController(
          const double mode, const double data, const unsigned int jointIndex);

      // Publish JointState of joint1 and joint2 using pub.
      void PublishJointState(
          physics::JointPtr joint1, physics::JointPtr joint2, ros::Publisher &pub);

      // Get JointStateData (name, position, velocity, effort) from joint.
      void GetJointStateData(
          physics::JointPtr joint,
          std::string &name,
          double &position,
          double &velocity,
          double &effort);
  };

}

#endif /* end of include guard: CHASER_CONTROLLER_H */
