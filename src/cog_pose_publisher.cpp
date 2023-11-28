#include "cog_pose_publisher/cog_pose_publisher.h"

namespace gazebo
{
  CoGPosePublisher::CoGPosePublisher() {}

  CoGPosePublisher::~CoGPosePublisher() {}

  void CoGPosePublisher::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
  {
    ROS_INFO("[CoGPosePublisher] Load start");

    // Get ROS node.
    std::string robot_namespace = "";
    if (!sdf->HasElement("robotNamespace")) {
      ROS_INFO("[CoGPosePublisher] missing <robotNamespace>, "
          "defaults to \"%s\"", robot_namespace.c_str());
    } else {
      robot_namespace = sdf->GetElement("robotNamespace")->Get<std::string>();
    }
    m_rosnode.reset(new ros::NodeHandle(robot_namespace));

    // Advertise topic for publishing CoG pose.
    std::string topicName = "";
    if (!sdf->HasElement("topicName")) {
      ROS_ERROR("[CoGPosePublisher] missing <topicName>.");
      return;
    } else {
      topicName = sdf->GetElement("topicName")->Get<std::string>();
    }
    m_pub = m_rosnode->advertise<geometry_msgs::PoseStamped>(topicName, QUEUE_SIZE);
    if (!m_pub) {
      ROS_ERROR("[CoGPosePublisher] Faild to advertise the topic %s.", topicName.c_str());
    }

    // Get link.
    std::string linkName = "linkName";
    if (!sdf->HasElement("linkName")) {
      ROS_ERROR("[CoGPosePublisher] missing <linkName>.");
      return;
    } else {
      linkName = sdf->GetElement("linkName")->Get<std::string>();
      ROS_INFO("[CoGPosePublisher] linkName:%s", linkName.c_str());
    }

    m_link = NULL;
    m_link = parent->GetLink(linkName);
    if (m_link == NULL) {
      ROS_ERROR("[CoGPosePublisher] missing the link in model. name: %s", linkName.c_str());
    }

    // listen to the update event (broadcast every simulation iteration)
    m_update_connection =
        event::Events::ConnectWorldUpdateBegin(
            boost::bind(&CoGPosePublisher::UpdateChild, this));

    ROS_INFO("[CoGPosePublisher] Load end");
  }

  void CoGPosePublisher::UpdateChild()
  {
//    ROS_INFO("[CoGPosePublisher] UpdateChild start");

    PublishCoGPose(m_link, m_pub);

//    ROS_INFO("[CoGPosePublisher] UpdateChild end");
  }

  void CoGPosePublisher::FiniChild() {
    m_rosnode->shutdown();
  }

  void CoGPosePublisher::PublishCoGPose(physics::LinkPtr link, ros::Publisher &pub)
  {
    // Get CoG pose from link.
    std::string name = "Not Loaded";
    double position_x = 0.0;
    double position_y = 0.0;
    double position_z = 0.0;
    double orientation_x = 0.0;
    double orientation_y = 0.0;
    double orientation_z = 0.0;
    double orientation_w = 0.0;
    if (link != NULL) {
      name= link->GetName();

      // Get the pose of the body's center of gravity in the world coordinate frame.
      ignition::math::Pose3d pose = link->WorldCoGPose(); 
      position_x = pose.Pos().X();
      position_y = pose.Pos().Y();
      position_z = pose.Pos().Z();
      orientation_x = pose.Rot().X();
      orientation_y = pose.Rot().Y();
      orientation_z = pose.Rot().Z();
      orientation_w = pose.Rot().W();
    }

    // Set JointState data.
    geometry_msgs::PoseStamped data;
    data.header.stamp = ros::Time::now();
    data.header.frame_id = name.c_str();
    data.pose.position.x = position_x;
    data.pose.position.y = position_y;
    data.pose.position.z = position_z;
    data.pose.orientation.x = orientation_x;
    data.pose.orientation.y = orientation_y;
    data.pose.orientation.z = orientation_z;
    data.pose.orientation.w = orientation_w;

    // Publish JointState data.
    if (pub) {
      pub.publish(data);
    }
  }

  // Register controller to pluginlib
  GZ_REGISTER_MODEL_PLUGIN(CoGPosePublisher)
}
