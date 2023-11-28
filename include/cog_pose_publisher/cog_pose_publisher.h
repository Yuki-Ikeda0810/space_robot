#ifndef COG_POSE_PUBLISHER_H
#define COG_POSE_PUBLISHER_H

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace gazebo {

  /**
    * CoGPosePublisher has below fuctions.
    * - Publish the topic <topicName> with the center of gravity of the link <linkName>.
    */
  class CoGPosePublisher : public ModelPlugin {

    public: 
      CoGPosePublisher();
      ~CoGPosePublisher();

      // Load the controller
      void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

    protected:
      // Update the controller called at every step
      virtual void UpdateChild();
      // Finalize the controller
      virtual void FiniChild();

    private:
      static const int QUEUE_SIZE = 1;

      boost::shared_ptr<ros::NodeHandle> m_rosnode;
      ros::Publisher m_pub;
      physics::LinkPtr m_link;
      event::ConnectionPtr m_update_connection;

      // Publish the center of gravity of the link with pub.
      void PublishCoGPose(physics::LinkPtr link, ros::Publisher &pub);
  };

}

#endif /* end of include guard: COG_POSE_PUBLISHER_H */
