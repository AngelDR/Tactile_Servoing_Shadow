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
#include <math.h>
#define PI 3.14159265

#include <Eigen/Dense>

using namespace Eigen;


// Funcion para obtener el punto con el valor de fuerza mayor
int getCellMaxValue(boost::array<double,16ul> sensor_values){
	double max_value = sensor_values[0];
	int cell = 0;
	for(int i = 0;i < 16; i++)
	{ 
	    if (sensor_values[i] > max_value)
	    {
	    	max_value = sensor_values[i];
	    	cell = i;
	    }
	}
	return cell;
}

// Funcion para obtener la pose de una celda del sensor 
// error en X e Y depende de las medidas del sensor. 17mm x 17mm
// distancia entre cada celdilla -> x = 4mm; y 4mm
std::vector<double> getCellPose(int position){
	double x,y,z;
	std::vector<double> pose;
	double step = 0.004;
	double radio = 0.007;
	// angulo celda = 20.75
	double angle = (20.75*PI)/180;
	switch (position)
	{
		case 1:
			x = -sin(2*angle) * radio;
			y = -cos(2*angle) * radio;
			z = 2 * step;
			break;
		case 2:
			x = -sin(angle) * radio;
			y = -cos(angle) * radio;
			z = 2 * step;
			break;
		case 3:
			x = 0;
			y = radio * (-1);
			z = 2 * step;
			break;
		case 4:
			x = sin(angle) * radio;
			y = -cos(angle) * radio;
			z = 2* step;
			break;
		case 5:
			x = -sin(2*angle) * radio;
			y = -cos(2*angle) * radio;
			z = step;
			break;
		case 6:
			x = -sin(angle) * radio;
			y = -cos(angle) * radio;
			z = step;
			break;
		case 7:
			x = 0;
			y = radio * (-1);
			z = step;
			break;
		case 8:
			x = sin(angle) * radio;
			y = -cos(angle) * radio;
			z = step;
			break;
		case 9:
			x = -sin(2*angle) * radio;
			y = -cos(2*angle) * radio;
			z = 0;
			break;
		case 10:
			x = -sin(angle) * radio;
			y = -cos(angle) * radio;
			z = 0;
			break;
		case 11:
			x = 0;
			y = radio * (-1);
			z = 0;
			break;
		case 12:
			x = sin(angle) * radio;
			y = -cos(angle) * radio;
			z = 0;
			break;
		case 13:
			x = -sin(2*angle) * radio;
			y = -cos(2*angle) * radio;
			z = -step;
			break;
		case 14:
			x = -sin(angle) * radio;
			y = -cos(angle) * radio;
			z = -step;
			break;
		case 15:
			x = 0;
			y = radio * (-1);
			z = -step;
			break;
		case 16:
			x = sin(angle) * radio;
			y = -cos(angle) * radio;
			z = -step;
			break;
	}
	pose.push_back(x);
	pose.push_back(y);
	pose.push_back(z);
	return pose;
}


//


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
  /**ros::Publisher vis_pub_ff = node.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  ros::Publisher vis_pub_mf = node.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
  ros::Publisher vis_pub_th = node.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
	**/

  // >> Array markers
  ros::Publisher marker_publisher = node.advertise<visualization_msgs::MarkerArray>("forces_marker_array", 0);
  visualization_msgs::MarkerArray  marker_array;
  marker_array.markers.resize(5);

  for(int i=0; i < 5; i++){
	  marker_array.markers[i].header.frame_id = "forearm";
	  marker_array.markers[i].ns = "force_sensor";
	  marker_array.markers[i].id = i;
	  marker_array.markers[i].type = visualization_msgs::Marker::ARROW;
	  marker_array.markers[i].action = visualization_msgs::Marker::ADD;
	  marker_array.markers[i].scale.x = 0.01;
	  marker_array.markers[i].scale.y = 0.001;
	  marker_array.markers[i].scale.z = 0.001;
	  marker_array.markers[i].color.a = 1.0; // Don't forget to set the alpha!
	  marker_array.markers[i].color.r = 0.0;
	  marker_array.markers[i].color.g = 1.0;
	  marker_array.markers[i].color.b = 0.0;
  }

  ros::Rate rate(10.0);

  int cell;
  std::vector<double> pose_cell;

  Matrix4d transformation_matrix;
  MatrixXd vector_cell(4,1);

  while (node.ok()){
    transform.setOrigin( tf::Vector3(0.0, 0.0, -0.015) );
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

	      	tf::Quaternion q = transform.getRotation();
	      	tf::Matrix3x3 rotation_matrix(q);
	      	transformation_matrix <<  rotation_matrix[0].getX(), rotation_matrix[0].getY(), rotation_matrix[0].getZ(), transform.getOrigin().getX(), 
	                                rotation_matrix[1].getX(), rotation_matrix[1].getY(), rotation_matrix[1].getZ(), transform.getOrigin().getY(),
	                                rotation_matrix[2].getX(), rotation_matrix[2].getY(), rotation_matrix[2].getZ(), transform.getOrigin().getZ(),
	                                0,0,0,1;
	      	cell = getCellMaxValue(srv_pressure.response.ff_values);
	      	pose_cell = getCellPose(cell);
	      	vector_cell << pose_cell.at(0),pose_cell.at(1),pose_cell.at(2),1;
	      	vector_cell = transformation_matrix * vector_cell;
	      	ROS_INFO("Point transformed");

	      	//ROS_INFO("Transformation  x = %f, y = %f, z = %f)", transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
	      	marker_array.markers[1].header.stamp = ros::Time();
	      	marker_array.markers[1].pose.position.x = vector_cell(0,0);
	      	marker_array.markers[1].pose.position.y = vector_cell(1,0);
	      	marker_array.markers[1].pose.position.z = vector_cell(2,0);
	      	// Rotar arrow -> para que eje x apunte hacia interior mano
	      	tf::Transform transform_arrow;
			//transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
			transform_arrow.setRotation( tf::createQuaternionFromRPY(0,0,-1.57) );
			transform.mult(transform,transform_arrow);
	      	marker_array.markers[1].pose.orientation.x = transform.getRotation().getX();
	      	marker_array.markers[1].pose.orientation.y = transform.getRotation().getY();
	      	marker_array.markers[1].pose.orientation.z = transform.getRotation().getZ();
	      	marker_array.markers[1].pose.orientation.w = transform.getRotation().getW();
	      	// Mostrar tamaño flecha proporcional a fuerza
	      	marker_array.markers[1].scale.x = 0.01 * srv_pressure.response.applied_force[1];

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
	      	
	      	tf::Quaternion q = transform.getRotation();
	      	tf::Matrix3x3 rotation_matrix(q);
	      	transformation_matrix <<  rotation_matrix[0].getX(), rotation_matrix[0].getY(), rotation_matrix[0].getZ(), transform.getOrigin().getX(), 
	                                rotation_matrix[1].getX(), rotation_matrix[1].getY(), rotation_matrix[1].getZ(), transform.getOrigin().getY(),
	                                rotation_matrix[2].getX(), rotation_matrix[2].getY(), rotation_matrix[2].getZ(), transform.getOrigin().getZ(),
	                                0,0,0,1;
	      	cell = getCellMaxValue(srv_pressure.response.mf_values);
	      	pose_cell = getCellPose(cell);
	      	vector_cell << pose_cell.at(0),pose_cell.at(1),pose_cell.at(2),1;
	      	vector_cell = transformation_matrix * vector_cell;
	      	ROS_INFO("Point transformed");

	      	//ROS_INFO("Transformation  x = %f, y = %f, z = %f)", transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
	      	marker_array.markers[2].header.stamp = ros::Time();
	      	marker_array.markers[2].pose.position.x = vector_cell(0,0);
	      	marker_array.markers[2].pose.position.y = vector_cell(1,0);
	      	marker_array.markers[2].pose.position.z = vector_cell(2,0);
	      	// Rotar arrow -> para que eje x apunte hacia interior mano
	      	tf::Transform transform_arrow;
			//transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
			transform_arrow.setRotation( tf::createQuaternionFromRPY(0,0,-1.57) );
			transform.mult(transform,transform_arrow);
	      	marker_array.markers[2].pose.orientation.x = transform.getRotation().getX();
	      	marker_array.markers[2].pose.orientation.y = transform.getRotation().getY();
	      	marker_array.markers[2].pose.orientation.z = transform.getRotation().getZ();
	      	marker_array.markers[2].pose.orientation.w = transform.getRotation().getW();
	      	// Mostrar tamaño flecha proporcional a fuerza
	      	marker_array.markers[2].scale.x = 0.01 * srv_pressure.response.applied_force[2];
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
          	tfListener.waitForTransform("forearm", "rfsensor", now, ros::Duration(3.0));
          	tfListener.lookupTransform("forearm", "rfsensor", now, transform);
	      	
	      	tf::Quaternion q = transform.getRotation();
	      	tf::Matrix3x3 rotation_matrix(q);
	      	transformation_matrix <<  rotation_matrix[0].getX(), rotation_matrix[0].getY(), rotation_matrix[0].getZ(), transform.getOrigin().getX(), 
	                                rotation_matrix[1].getX(), rotation_matrix[1].getY(), rotation_matrix[1].getZ(), transform.getOrigin().getY(),
	                                rotation_matrix[2].getX(), rotation_matrix[2].getY(), rotation_matrix[2].getZ(), transform.getOrigin().getZ(),
	                                0,0,0,1;
	      	cell = getCellMaxValue(srv_pressure.response.rf_values);
	      	pose_cell = getCellPose(cell);
	      	vector_cell << pose_cell.at(0),pose_cell.at(1),pose_cell.at(2),1;
	      	vector_cell = transformation_matrix * vector_cell;
	      	ROS_INFO("Point transformed");

	      	//ROS_INFO("Transformation  x = %f, y = %f, z = %f)", transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
	      	marker_array.markers[3].header.stamp = ros::Time();
	      	marker_array.markers[3].pose.position.x = vector_cell(0,0);
	      	marker_array.markers[3].pose.position.y = vector_cell(1,0);
	      	marker_array.markers[3].pose.position.z = vector_cell(2,0);
	      	// Rotar arrow -> para que eje x apunte hacia interior mano
	      	tf::Transform transform_arrow;
			//transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
			transform_arrow.setRotation( tf::createQuaternionFromRPY(0,0,-1.57) );
			transform.mult(transform,transform_arrow);
	      	marker_array.markers[3].pose.orientation.x = transform.getRotation().getX();
	      	marker_array.markers[3].pose.orientation.y = transform.getRotation().getY();
	      	marker_array.markers[3].pose.orientation.z = transform.getRotation().getZ();
	      	marker_array.markers[3].pose.orientation.w = transform.getRotation().getW();
	      	// Mostrar tamaño flecha proporcional a fuerza
	      	marker_array.markers[3].scale.x = 0.01 * srv_pressure.response.applied_force[3];

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
          	tfListener.waitForTransform("forearm", "lfsensor", now, ros::Duration(3.0));
          	tfListener.lookupTransform("forearm", "lfsensor", now, transform);
 	      	
 	      	tf::Quaternion q = transform.getRotation();
	      	tf::Matrix3x3 rotation_matrix(q);
	      	transformation_matrix <<  rotation_matrix[0].getX(), rotation_matrix[0].getY(), rotation_matrix[0].getZ(), transform.getOrigin().getX(), 
	                                rotation_matrix[1].getX(), rotation_matrix[1].getY(), rotation_matrix[1].getZ(), transform.getOrigin().getY(),
	                                rotation_matrix[2].getX(), rotation_matrix[2].getY(), rotation_matrix[2].getZ(), transform.getOrigin().getZ(),
	                                0,0,0,1;
	      	cell = getCellMaxValue(srv_pressure.response.lf_values);
	      	pose_cell = getCellPose(cell);
	      	vector_cell << pose_cell.at(0),pose_cell.at(1),pose_cell.at(2),1;
	      	vector_cell = transformation_matrix * vector_cell;
	      	ROS_INFO("Point transformed");

	      	//ROS_INFO("Transformation  x = %f, y = %f, z = %f)", transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
	      	marker_array.markers[4].header.stamp = ros::Time();
	      	marker_array.markers[4].pose.position.x = vector_cell(0,0);
	      	marker_array.markers[4].pose.position.y = vector_cell(1,0);
	      	marker_array.markers[4].pose.position.z = vector_cell(2,0);
	      	// Rotar arrow -> para que eje x apunte hacia interior mano
	      	tf::Transform transform_arrow;
			//transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
			transform_arrow.setRotation( tf::createQuaternionFromRPY(0,0,-1.57) );
			transform.mult(transform,transform_arrow);
	      	marker_array.markers[4].pose.orientation.x = transform.getRotation().getX();
	      	marker_array.markers[4].pose.orientation.y = transform.getRotation().getY();
	      	marker_array.markers[4].pose.orientation.z = transform.getRotation().getZ();
	      	marker_array.markers[4].pose.orientation.w = transform.getRotation().getW();
	      	// Mostrar tamaño flecha proporcional a fuerza
	      	marker_array.markers[4].scale.x = 0.01 * srv_pressure.response.applied_force[4];

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
          	tfListener.waitForTransform("forearm", "thsensor", now, ros::Duration(3.0));
          	tfListener.lookupTransform("forearm", "thsensor", now, transform);
	      	
	      	tf::Quaternion q = transform.getRotation();
	      	tf::Matrix3x3 rotation_matrix(q);
	      	transformation_matrix <<  rotation_matrix[0].getX(), rotation_matrix[0].getY(), rotation_matrix[0].getZ(), transform.getOrigin().getX(), 
	                                rotation_matrix[1].getX(), rotation_matrix[1].getY(), rotation_matrix[1].getZ(), transform.getOrigin().getY(),
	                                rotation_matrix[2].getX(), rotation_matrix[2].getY(), rotation_matrix[2].getZ(), transform.getOrigin().getZ(),
	                                0,0,0,1;
	      	cell = getCellMaxValue(srv_pressure.response.th_values);
	      	pose_cell = getCellPose(cell);
	      	vector_cell << pose_cell.at(0),pose_cell.at(1),pose_cell.at(2),1;
	      	vector_cell = transformation_matrix * vector_cell;
	      	ROS_INFO("Point transformed");

	      	//ROS_INFO("Transformation  x = %f, y = %f, z = %f)", transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
	      	marker_array.markers[0].header.stamp = ros::Time();
	      	marker_array.markers[0].pose.position.x = vector_cell(0,0);
	      	marker_array.markers[0].pose.position.y = vector_cell(1,0);
	      	marker_array.markers[0].pose.position.z = vector_cell(2,0);
	      	// Rotar arrow -> para que eje x apunte hacia interior mano
	      	tf::Transform transform_arrow;
			//transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
			transform_arrow.setRotation( tf::createQuaternionFromRPY(0,0,1.57) );
			transform.mult(transform,transform_arrow);
	      	marker_array.markers[0].pose.orientation.x = transform.getRotation().getX();
	      	marker_array.markers[0].pose.orientation.y = transform.getRotation().getY();
	      	marker_array.markers[0].pose.orientation.z = transform.getRotation().getZ();
	      	marker_array.markers[0].pose.orientation.w = transform.getRotation().getW();
	      	// Mostrar tamaño flecha proporcional a fuerza
	      	marker_array.markers[0].scale.x = 0.01 * srv_pressure.response.applied_force[0];
        }catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }


        marker_publisher.publish(marker_array);
        ROS_INFO("Marker array published...");
    }
    else
    {
        ROS_ERROR("Failed to call service pressure");
        return 1;
    }    
  }

};