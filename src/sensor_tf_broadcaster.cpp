#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "sensor_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;
  //geometry_msgs::TransformStamped transformStampedff;
  //geometry_msgs::TransformStamped transformStampedmf;
  

  ros::Rate rate(10.0);
  while (node.ok()){
    transform.setOrigin( tf::Vector3(0.0, -0.01, -0.02) );
    transform.setRotation( tf::Quaternion(0, 0, 0, 1) );
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "fftip", "ffsensor"));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "mftip", "mfsensor"));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "rftip", "rfsensor"));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "lftip", "lfsensor"));
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "thtip", "thsensor"));
    rate.sleep();
  }

};