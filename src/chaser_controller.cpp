#include "chaser_controller/chaser_controller.h"
#include <stdio.h>

namespace gazebo
{
  ChaserController::ChaserController() {}

  ChaserController::~ChaserController() {}

  void ChaserController::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
  {
    ROS_INFO("[ChaserController] Load start");

    // Get ROS node.
    std::string robot_namespace = "";
    if (!sdf->HasElement("robotNamespace")) {
      ROS_INFO("[ChaserController] missing <robotNamespace>, "
          "defaults to \"%s\"", robot_namespace.c_str());
    } else {
      robot_namespace = sdf->GetElement("robotNamespace")->Get<std::string>();
    }
    m_rosnode.reset(new ros::NodeHandle(robot_namespace));

    // Subscribe topic for receiving command.
    m_sub_1 = m_rosnode->subscribe(
        TOPIC_CMD_ARM_1, QUEUE_SIZE, &ChaserController::CmdArm1Callback, this);
    if (!m_sub_1) {
      ROS_ERROR("[ChaserController] Faild to subscribe the topic %s.", TOPIC_CMD_ARM_1.c_str());
    }
    m_sub_2 = m_rosnode->subscribe(
        TOPIC_CMD_ARM_2, QUEUE_SIZE, &ChaserController::CmdArm2Callback, this);
    if (!m_sub_2) {
      ROS_ERROR("[ChaserController] Faild to subscribe the topic %s.", TOPIC_CMD_ARM_2.c_str());
    }
    // m_sub_3 = m_rosnode->subscribe(
    //     TOPIC_CMD_ARM_3, QUEUE_SIZE, &ChaserController::CmdArm3Callback, this);
    // if (!m_sub_3) {
    //   ROS_ERROR("[ChaserController] Faild to subscribe the topic %s.", TOPIC_CMD_ARM_3.c_str());
    // }
    // m_sub_4 = m_rosnode->subscribe(
    //     TOPIC_CMD_ARM_4, QUEUE_SIZE, &ChaserController::CmdArm4Callback, this);
    // if (!m_sub_4) {
    //   ROS_ERROR("[ChaserController] Faild to subscribe the topic %s.", TOPIC_CMD_ARM_4.c_str());
    // }
    // m_sub_5 = m_rosnode->subscribe(
    //     TOPIC_CMD_ARM_5, QUEUE_SIZE, &ChaserController::CmdArm5Callback, this);
    // if (!m_sub_5) {
    //   ROS_ERROR("[ChaserController] Faild to subscribe the topic %s.", TOPIC_CMD_ARM_5.c_str());
    // }

    // Advertise topic for publishing joint state.
    m_pub_1 = m_rosnode->advertise<sensor_msgs::JointState>(TOPIC_OBS_ARM_1, QUEUE_SIZE);
    if (!m_pub_1) {
      ROS_ERROR("[ChaserController] Faild to advertise the topic %s.", TOPIC_OBS_ARM_1.c_str());
    }
    m_pub_2 = m_rosnode->advertise<sensor_msgs::JointState>(TOPIC_OBS_ARM_2, QUEUE_SIZE);
    if (!m_pub_2) {
      ROS_ERROR("[ChaserController] Faild to advertise the topic %s.", TOPIC_OBS_ARM_2.c_str());
    }
    // m_pub_3 = m_rosnode->advertise<sensor_msgs::JointState>(TOPIC_OBS_ARM_3, QUEUE_SIZE);
    // if (!m_pub_3) {
    //   ROS_ERROR("[ChaserController] Faild to advertise the topic %s.", TOPIC_OBS_ARM_3.c_str());
    // }
    // m_pub_4 = m_rosnode->advertise<sensor_msgs::JointState>(TOPIC_OBS_ARM_4, QUEUE_SIZE);
    // if (!m_pub_4) {
    //   ROS_ERROR("[ChaserController] Faild to advertise the topic %s.", TOPIC_OBS_ARM_4.c_str());
    // }
    // m_pub_5 = m_rosnode->advertise<sensor_msgs::JointState>(TOPIC_OBS_ARM_5, QUEUE_SIZE);
    // if (!m_pub_5) {
    //   ROS_ERROR("[ChaserController] Faild to advertise the topic %s.", TOPIC_OBS_ARM_5.c_str());
    // }

    // Advertise topic for publishing command to JointMultiController.
    m_pub_cnt[0] = m_rosnode->advertise<std_msgs::Float64MultiArray>(TOPIC_CMD_CNT_1, QUEUE_SIZE);
    m_pub_cnt[1] = m_rosnode->advertise<std_msgs::Float64MultiArray>(TOPIC_CMD_CNT_2, QUEUE_SIZE);
    m_pub_cnt[2] = m_rosnode->advertise<std_msgs::Float64MultiArray>(TOPIC_CMD_CNT_3, QUEUE_SIZE);
    // m_pub_cnt[3] = m_rosnode->advertise<std_msgs::Float64MultiArray>(TOPIC_CMD_CNT_4, QUEUE_SIZE);
    // m_pub_cnt[4] = m_rosnode->advertise<std_msgs::Float64MultiArray>(TOPIC_CMD_CNT_5, QUEUE_SIZE);
    // m_pub_cnt[5] = m_rosnode->advertise<std_msgs::Float64MultiArray>(TOPIC_CMD_CNT_6, QUEUE_SIZE);
    // m_pub_cnt[6] = m_rosnode->advertise<std_msgs::Float64MultiArray>(TOPIC_CMD_CNT_7, QUEUE_SIZE);
    // m_pub_cnt[7] = m_rosnode->advertise<std_msgs::Float64MultiArray>(TOPIC_CMD_CNT_8, QUEUE_SIZE);  // end effector finger1
    // m_pub_cnt[8] = m_rosnode->advertise<std_msgs::Float64MultiArray>(TOPIC_CMD_CNT_9, QUEUE_SIZE);  // end effector finger2
    // m_pub_cnt[9] = m_rosnode->advertise<std_msgs::Float64MultiArray>(TOPIC_CMD_CNT_10, QUEUE_SIZE); // end effector finger3
    for (int count = 0; count < JOINT_NUM + END_JOINT_NUM; count++) {
      if (!m_pub_cnt[count]) {
        ROS_ERROR("[ChaserController] Faild to advertise the topic. index: %d", count);
      }
    }

    //TODO Get links.
//    parent->SetLinearVel(math::Vector3(0, 0, -0.1));
//
//    physics::Link_V links = parent->GetLinks();
//    for (int count = 0; count < links.size(); ++count) {
//      ROS_ERROR("[ChaserController] %s.", links[count]->GetName().c_str());
//      links[count]->SetLinearVel(math::Vector3(0, -0.1, 0));
//    }
//
//    physics::LinkPtr link = parent->GetLink("base_link");
//    if (link != NULL) { 
//      link->SetLinearVel(math::Vector3(0.1, -0.1, 0.1));
//      ROS_ERROR("[ChaserController] OK");
//    } else {
//      ROS_ERROR("[ChaserController] NG");
//    }

    // Get joints.
    for (int count = 0; count < JOINT_NUM; count++) {
      m_joints[count] = NULL;
    }
    m_joints[0] = parent->GetJoint(JOINT_NAME_1);
    m_joints[1] = parent->GetJoint(JOINT_NAME_2);
    m_joints[2] = parent->GetJoint(JOINT_NAME_3);
    // m_joints[3] = parent->GetJoint(JOINT_NAME_4);
    // m_joints[4] = parent->GetJoint(JOINT_NAME_5);
    // m_joints[5] = parent->GetJoint(JOINT_NAME_6);
    // m_joints[6] = parent->GetJoint(JOINT_NAME_7);

    for (int count = 0; count < JOINT_NUM; count++) {
      if (m_joints[count] == NULL) {
        ROS_ERROR("[ChaserController] missing the joint in model. index: %d", count);
      }
    }

    // Get end effector joints.
    // for (int count = 0; count < END_JOINT_NUM; count++) {
    //   m_end_joints[count] = NULL;
    // }
    // m_end_joints[0] = parent->GetJoint(END_JOINT_NAME_1);
    // m_end_joints[1] = parent->GetJoint(END_JOINT_NAME_2);
    // m_end_joints[2] = parent->GetJoint(END_JOINT_NAME_3);
    // for (int count = 0; count < END_JOINT_NUM; count++) {
    //   if (m_end_joints[count] == NULL) {
    //     ROS_ERROR("[ChaserController] missing the end effector joint in model. index: %d", count);
    //   }
    // }

    // Set initial joint position.
    if (sdf->HasElement("joint1Position")) {
      double joint1Position = sdf->GetElement("joint1Position")->Get<double>();
      if (m_joints[0] != NULL) m_joints[0]->SetPosition(0, joint1Position);
    }
    if (sdf->HasElement("joint2Position")) {
      double joint2Position = sdf->GetElement("joint2Position")->Get<double>();
      if (m_joints[1] != NULL) m_joints[1]->SetPosition(0, joint2Position);
    }
    if (sdf->HasElement("joint3Position")) {
      double joint3Position = sdf->GetElement("joint3Position")->Get<double>();
      if (m_joints[2] != NULL) m_joints[2]->SetPosition(0, joint3Position);
    }
    // if (sdf->HasElement("joint4Position")) {
    //   double joint4Position = sdf->GetElement("joint4Position")->Get<double>();
    //   if (m_joints[3] != NULL) m_joints[3]->SetPosition(0, joint4Position);
    // }
    // if (sdf->HasElement("joint5Position")) {
    //   double joint5Position = sdf->GetElement("joint5Position")->Get<double>();
    //   if (m_joints[4] != NULL) m_joints[4]->SetPosition(0, joint5Position);
    // }
    // if (sdf->HasElement("joint6Position")) {
    //   double joint6Position = sdf->GetElement("joint6Position")->Get<double>();
    //   if (m_joints[5] != NULL) m_joints[5]->SetPosition(0, joint6Position);
    // }
    // if (sdf->HasElement("joint7Position")) {
    //   double joint7Position = sdf->GetElement("joint7Position")->Get<double>();
    //   if (m_joints[6] != NULL) m_joints[6]->SetPosition(0, joint7Position);
    // }
    // if (sdf->HasElement("endJointPosition")) {
    //   double endJointPosition = sdf->GetElement("endJointPosition")->Get<double>();
    //   if (m_end_joints[0] != NULL) m_end_joints[0]->SetPosition(0, endJointPosition);
    //   if (m_end_joints[1] != NULL) m_end_joints[1]->SetPosition(0, endJointPosition);
    //   if (m_end_joints[2] != NULL) m_end_joints[2]->SetPosition(0, endJointPosition);
    // }

    // listen to the update event (broadcast every simulation iteration)
    m_update_connection =
        event::Events::ConnectWorldUpdateBegin(
            boost::bind(&ChaserController::UpdateChild, this));

    ROS_INFO("[ChaserController] Load end");
  }

  void ChaserController::UpdateChild()
  {
//    ROS_INFO("[ChaserController] UpdateChild start");

    PublishJointState(m_joints[0], m_joints[1], m_pub_1);
    PublishJointState(m_joints[2], 0, m_pub_2);
    // PublishJointState(m_joints[4], m_joints[5], m_pub_3);
    // PublishJointState(m_joints[6], m_end_joints[0], m_pub_4);
    // PublishJointState(m_end_joints[1], m_end_joints[2], m_pub_5);

//    ROS_INFO("[ChaserController] UpdateChild end");
  }

  void ChaserController::FiniChild() {
    m_rosnode->shutdown();
  }

  void ChaserController::CmdArm1Callback(const std_msgs::Float64MultiArray cmd)
  {
    if (cmd.data.size() != CMD_ARM_SIZE) {
       ROS_ERROR("[ChaserController] /chaser/cmd_arm1 data size is invalid.");
       return;
    }
//    ROS_INFO("[ChaserController] CmdArm1Callback is invoked. %f, %f, %f",
//        cmd.data[0], cmd.data[1], cmd.data[2]);
    PublishToJointController(cmd.data[0], cmd.data[1], 0 /*arm_joint_1*/);
    PublishToJointController(cmd.data[0], cmd.data[2], 1 /*arm_joint_2*/);
  }

  void ChaserController::CmdArm2Callback(const std_msgs::Float64MultiArray cmd)
  {
    // debug 
    // std::cout << "debug :" << cmd << std::endl;

    if (cmd.data.size() != CMD_ARM_SIZE) {
       ROS_ERROR("[ChaserController] /chaser/cmd_arm2 data size is invalid.");
       return;
    }
//    ROS_INFO("[ChaserController] CmdArm2Callback is invoked. %f, %f, %f",
//        cmd.data[0], cmd.data[1], cmd.data[2]);
    PublishToJointController(cmd.data[0], cmd.data[1], 2 /*arm_joint_3*/);
    // PublishToJointController(cmd.data[0], cmd.data[2], 3 /*arm_joint_4*/);
  }

//   void ChaserController::CmdArm3Callback(const std_msgs::Float64MultiArray cmd)
//   {
//     if (cmd.data.size() != 3) {
//        ROS_ERROR("[ChaserController] /chaser/cmd_arm3 data size is invalid.");
//        return;
//     }
// //    ROS_INFO("[ChaserController] CmdArm3Callback is invoked. %f, %f, %f",
// //      cmd.data[0], cmd.data[1], cmd.data[2]);
//     PublishToJointController(cmd.data[0], cmd.data[1], 4 /*arm_joint_5*/);
//     PublishToJointController(cmd.data[0], cmd.data[2], 5 /*arm_joint_6*/);
//   }

//   void ChaserController::CmdArm4Callback(const std_msgs::Float64MultiArray cmd)
//   {
//     if (cmd.data.size() != 3) {
//        ROS_ERROR("[ChaserController] /chaser/cmd_arm4 data size is invalid.");
//        return;
//     }
// //    ROS_INFO("[ChaserController] CmdArm4Callback is invoked. %f, %f, %f",
// //        cmd.data[0], cmd.data[1], cmd.data[2]);
//     PublishToJointController(cmd.data[0], cmd.data[1], 6 /*arm_joint_7*/);
//     PublishToJointController(cmd.data[0], cmd.data[2], 7 /*end_effector_finger_1*/);
//   }

//   void ChaserController::CmdArm5Callback(const std_msgs::Float64MultiArray cmd)
//   {
//     if (cmd.data.size() != 3) {
//        ROS_ERROR("[ChaserController] /chaser/cmd_arm4 data size is invalid.");
//        return;
//     }
// //    ROS_INFO("[ChaserController] CmdArm5Callback is invoked. %f, %f, %f",
// //        cmd.data[0], cmd.data[1], cmd.data[2]);
//     PublishToJointController(cmd.data[0], cmd.data[1], 8 /*end_effector_finger_2*/);
//     PublishToJointController(cmd.data[0], cmd.data[2], 9 /*end_effector_finger_3*/);
//   }

  void ChaserController::PublishToJointController(
      const double mode, const double data, const unsigned int jointIndex)
  {
    if (jointIndex >= JOINT_NUM + END_JOINT_NUM) {
       ROS_ERROR("[ChaserController] Joint index is invalid.");
       return;
    }

    // Publish data to JointMultiController
    std_msgs::Float64MultiArray msg;
    msg.data = {mode, data};
    if (m_pub_cnt[jointIndex]) {
      m_pub_cnt[jointIndex].publish(msg);
    }
  }

  void ChaserController::PublishJointState(
      physics::JointPtr joint1, physics::JointPtr joint2, ros::Publisher &pub)
  {
    // Get JointState data from joint1.
    std::string name1 = "Not Loaded";
    double position1 = 0.0;
    double velocity1 = 0.0;
    double effort1   = 0.0;
    if (joint1 != NULL) {
      GetJointStateData(joint1, name1, position1, velocity1, effort1);
    }

    // Get JointState data from joint2.
    std::string name2 = "Not Loaded";
    double position2 = 0.0;
    double velocity2 = 0.0;
    double effort2   = 0.0;
    if (joint2 != NULL) {
       GetJointStateData(joint2, name2, position2, velocity2, effort2);
    }

    // Set JointState data.
    sensor_msgs::JointState data;
    data.header.stamp = ros::Time::now();
    data.header.frame_id = pub.getTopic().c_str();
    data.name = {name1.c_str(), name2.c_str()};
    data.position = {position1, position2};
    data.velocity = {velocity1, velocity2};
    data.effort   = {effort1,   effort2};

    // Publish JointState data.
    if (pub) {
      pub.publish(data);
    }
  }

  void ChaserController::GetJointStateData(
      physics::JointPtr joint,
      std::string &name,
      double &position,
      double &velocity,
      double &effort)
  {
    name = joint->GetName();

    ignition::math::Angle angle = joint->Position(0);
    angle.Normalize();  // Normalize the angle in the range -Pi to Pi. 
    position = angle.Radian();

    velocity = joint->GetVelocity(0);

    effort   = joint->GetForce(0);
  }

  // Register controller to pluginlib
  GZ_REGISTER_MODEL_PLUGIN(ChaserController)
}
