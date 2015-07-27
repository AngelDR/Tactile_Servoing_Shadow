#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <sensor_msgs/JointState.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include "std_msgs/Float64.h"
#include "tekscan_client/GetPressureMap.h"

int main(int argc, char** argv){
  ros::init(argc, argv, "sensor_tf_broadcaster");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform transform;
  //geometry_msgs::TransformStamped transformStampedff;
  //geometry_msgs::TransformStamped transformStampedmf;
    // Clases para transformaciones con tf2
  tf2_ros::Buffer tfBuffer;
  tf::TransformListener tfListener; 
  geometry_msgs::TransformStamped ff_transf;

  ros::ServiceClient pressure_client = node.serviceClient<tekscan_client::GetPressureMap>("GetPressureMap");
  tekscan_client::GetPressureMap srv_pressure;
  srv_pressure.request.num = 0;

    // >> Marker visualization forces
  ros::Publisher vis_pub_ff = node.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  ros::Publisher vis_pub_mf = node.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  ros::Publisher vis_pub_th = node.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );

  visualization_msgs::Marker marker;
  marker.header.frame_id = "forearm";
  marker.ns = "force_sensor";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.01;
  marker.scale.y = 0.001;
  marker.scale.z = 0.001;
  marker.color.a = 1.0; // Don't forget to set the alpha!
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  

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

    if (pressure_client.call(srv_pressure))
    {

        // Visualization
        // Visualizacion forces
        try{
          ros::Time now = ros::Time::now();
          tf::StampedTransform transform;
          tfListener.waitForTransform("forearm", "ffsensor", now, ros::Duration(3.0));
          tfListener.lookupTransform("forearm", "ffsensor", now, transform);
          //ROS_INFO("Transformation  x = %f, y = %f, z = %f)", transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
          marker.header.stamp = ros::Time();
          marker.pose.position.x = transform.getOrigin().getX();
          marker.pose.position.y = transform.getOrigin().getY();
          marker.pose.position.z = transform.getOrigin().getZ();
          marker.pose.orientation.x = transform.getRotation().getX();
          marker.pose.orientation.y = transform.getRotation().getY();
          marker.pose.orientation.z = -1.58;//transform.getRotation().getZ();
          marker.pose.orientation.w = transform.getRotation().getW();
          // Mostrar tamaño flecha proporcional a fuerza
          marker.scale.x = 0.01 * srv_pressure.response.applied_force[1];
          vis_pub_ff.publish( marker );
        }catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }

            // Visualization
        // Visualizacion forces
        try{
          ros::Time now = ros::Time::now();
          tf::StampedTransform transform;
          tfListener.waitForTransform("forearm", "mfsensor", now, ros::Duration(3.0));
          tfListener.lookupTransform("forearm", "mfsensor", now, transform);
          //ROS_INFO("Transformation  x = %f, y = %f, z = %f)", transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
          marker.header.stamp = ros::Time();
          marker.pose.position.x = transform.getOrigin().getX();
          marker.pose.position.y = transform.getOrigin().getY();
          marker.pose.position.z = transform.getOrigin().getZ();
          marker.pose.orientation.x = transform.getRotation().getX();
          marker.pose.orientation.y = transform.getRotation().getY();
          marker.pose.orientation.z = -1.58;//transform.getRotation().getZ();
          marker.pose.orientation.w = transform.getRotation().getW();
          // Mostrar tamaño flecha proporcional a fuerza
          marker.scale.x = 0.01 * srv_pressure.response.applied_force[1];
          vis_pub_mf.publish( marker );
        }catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }
    }
    else
    {
        ROS_ERROR("Failed to call service pressure");
        return 1;
    }    
  }

};