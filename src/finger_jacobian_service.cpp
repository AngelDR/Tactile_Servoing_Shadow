

/* Author: Angel Delgado */

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>
// Robot state publishing
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <std_msgs/Float64.h>
#include <pr2_controllers_msgs/JointControllerState.h>
// PI
#include <boost/math/constants/constants.hpp>
#include "tactile_servoing_shadow/getFingerJacobianMatrix.h"
#include "tactile_servoing_shadow/jacobian_matrix.h"


// Shared robot_model & robot_state
robot_model::RobotModelPtr sharedKinematic_model;
robot_state::RobotStatePtr sharedKinematic_state; 
const robot_state::JointModelGroup* joint_model;
std::string* tip_name;

// GENERIC FINGER -> FIRST, MIDDLE AND RING FINGER
/**
* Joint position callback: j0  (only for first, middle, ring and little fingers)
*/
void _j0Callback(const pr2_controllers_msgs::JointControllerState::ConstPtr& msg)
{
  const double vSet_point= (double) msg->set_point;
  const std::vector<std::string> &joint_names = joint_model->getJointModelNames();
  sharedKinematic_state->setJointPositions(joint_names[3],&vSet_point);

  // j0 usado por ff, mf, rf y lf.  * Cambia el modo de obtener los nombres de las articulaciones, porque el numero de las articulaciones son distintos
  if((*tip_name == "fftip") || (*tip_name == "mftip") || (*tip_name == "rftip"))
    sharedKinematic_state->setJointPositions(joint_names[2],&vSet_point);
  else
    sharedKinematic_state->setJointPositions(joint_names[4],&vSet_point);
}
/**
* Joint position callback: j1  (only for thumb)
*/
void _j1Callback(const pr2_controllers_msgs::JointControllerState::ConstPtr& msg)
{
  const double vSet_point= (double) msg->set_point;
  const std::vector<std::string> &joint_names = joint_model->getJointModelNames();
  sharedKinematic_state->setJointPositions(joint_names[4],&vSet_point);
}
/**
* Joint position callback: j2 (only for thumb)
*/
void _j2Callback(const pr2_controllers_msgs::JointControllerState::ConstPtr& msg)
{
  const double vSet_point= (double) msg->set_point;
  const std::vector<std::string> &joint_names = joint_model->getJointModelNames();
  sharedKinematic_state->setJointPositions(joint_names[3],&vSet_point);  
}
/**
* Joint position callback: j3  (usado por todos los dedos -> )
* aquí se implementa la obtención the iK y Jacobian 
*/
void _j3Callback(const pr2_controllers_msgs::JointControllerState::ConstPtr& msg)
{
  const double vSet_point= (double) msg->set_point;
  const std::vector<std::string> &joint_names = joint_model->getJointModelNames();
  //sharedKinematic_state->setJointPositions(joint_names[1],&vSet_point);
  
  // j3 usado por th, ff, mf, rf y lf.  * Cambia el modo de obtener los nombres de las articulaciones, porque el numero de las articulaciones son distintos
  if((*tip_name == "lftip") || (*tip_name == "thtip"))
    sharedKinematic_state->setJointPositions(joint_names[2],&vSet_point);
  else
    sharedKinematic_state->setJointPositions(joint_names[1],&vSet_point);
}
/**
* Joint position callback: j4
*/
void _j4Callback(const pr2_controllers_msgs::JointControllerState::ConstPtr& msg)
{
  const double vSet_point= (double) msg->set_point;
  const std::vector<std::string> &joint_names = joint_model->getJointModelNames();
  //sharedKinematic_state->setJointPositions(joint_names[0],&vSet_point);
  
  // j3 usado por th, ff, mf, rf y lf.  * Cambia el modo de obtener los nombres de las articulaciones, porque el numero de las articulaciones son distintos
  if((*tip_name == "lftip") || (*tip_name == "thtip"))
    sharedKinematic_state->setJointPositions(joint_names[1],&vSet_point);
  else
    sharedKinematic_state->setJointPositions(joint_names[0],&vSet_point);
}
/**
* Joint position callback: j5
*/
void _j5Callback(const pr2_controllers_msgs::JointControllerState::ConstPtr& msg)
{
  const double vSet_point= (double) msg->set_point;
  const std::vector<std::string> &joint_names = joint_model->getJointModelNames();
  // j5 solo usado por lf, th
  sharedKinematic_state->setJointPositions(joint_names[0],&vSet_point);
}

/**
* Joint position callback: j5
*/
void fake_control_Callback(const sensor_msgs::JointState::ConstPtr& msg)
{
  const double vSet_point_1 = (double) msg->position[2];
  const double vSet_point_2 = (double) msg->position[3];
  const double vSet_point_3 = (double) msg->position[4];
  const double vSet_point_4 = (double) msg->position[5];
  ROS_INFO("First finger positions: %f,  %f,  %f,  %f", vSet_point_1,vSet_point_2, vSet_point_3, vSet_point_4);

  const std::vector<std::string> &joint_names = joint_model->getJointModelNames();
  // Actualizar first finger
  sharedKinematic_state->setJointPositions(joint_names[0],&vSet_point_1);
  sharedKinematic_state->setJointPositions(joint_names[1],&vSet_point_2);
  sharedKinematic_state->setJointPositions(joint_names[2],&vSet_point_3);
  sharedKinematic_state->setJointPositions(joint_names[3],&vSet_point_4);
}

bool jacobian_service(tactile_servoing_shadow::getFingerJacobianMatrix::Request &req, tactile_servoing_shadow::getFingerJacobianMatrix::Response &res)
{
  switch(req.finger_id){
    case 1:
     joint_model = sharedKinematic_model->getJointModelGroup("thumb");
      //ROS_INFO("Joint model = thumb");
     *tip_name = "thtip";
     break;
    case 2:
     joint_model = sharedKinematic_model->getJointModelGroup("first_finger");
       //ROS_INFO("Joint model = first finger");
     *tip_name = "fftip";
     break;
    case 3:  
     joint_model = sharedKinematic_model->getJointModelGroup("middle_finger");
       //ROS_INFO("Joint model = middle finger");
     *tip_name = "mftip";
     break; 
    case 4:
     joint_model = sharedKinematic_model->getJointModelGroup("ring_finger");
       //ROS_INFO("Joint model = ring finger");
     *tip_name = "rftip";
     break; 
    case 5:
     joint_model = sharedKinematic_model->getJointModelGroup("little_finger");
       //ROS_INFO("Joint model = little finger");
     *tip_name = "lftip";
     break;
  }
  Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
  Eigen::MatrixXd jacobian;
  sharedKinematic_state->getJacobian(joint_model, sharedKinematic_state->getLinkModel(joint_model->getLinkModelNames().back()),
          reference_point_position,
          jacobian);
  int pos = 0;
  for(int i = 0;i < 6;i++){
    for(int j = 0; j < 4;j++){
      res.jacobian[pos] = jacobian(i,j);
      pos++;
    }
  }
  ROS_INFO_STREAM("Jacobian: \n" << jacobian);
}

/**
 * Node main
 * 
 */
int main(int argc, char **argv)
{
  ros::init (argc, argv, "finger_jacobian_service");
  ros::NodeHandle nh; 
  ros::Publisher jacobian_pub = nh.advertise<tactile_servoing_shadow::jacobian_matrix>("jacobian",1000);
  ros::ServiceServer service = nh.advertiseService("/GetJacobianMatrix", jacobian_service); 
  
  //  definir subscribers Mano Shadow real
  ros::Subscriber subs_Th_J1;
  ros::Subscriber subs_Th_J2;
  ros::Subscriber subs_Th_J3;
  ros::Subscriber subs_Th_J4;
  ros::Subscriber subs_Th_J5;
  
  ros::Subscriber subs_Ff_J0;
  ros::Subscriber subs_Ff_J3;
  ros::Subscriber subs_Ff_J4;
  
  ros::Subscriber subs_Mf_J0;
  ros::Subscriber subs_Mf_J3;
  ros::Subscriber subs_Mf_J4;
  
  ros::Subscriber subs_Rf_J0;
  ros::Subscriber subs_Rf_J3;
  ros::Subscriber subs_Rf_J4;
  
  ros::Subscriber subs_Lf_J0;
  ros::Subscriber subs_Lf_J3;
  ros::Subscriber subs_Lf_J4;
  ros::Subscriber subs_Lf_J5;
  ROS_INFO("Subscribers iniciados");

  // definir subscriber fake controller Shadow
  ros::Subscriber subs_fake_controller;
  
  int finger;
  nh.getParam("/get_ik/dedo", finger);
  ROS_INFO("Dedo a usar : %d", finger);
  tip_name = new std::string;
  ROS_INFO("Dedo a usar: %d", finger);
  
  finger = 2;
  // Definir callbacks para cada joint publicada
  switch(finger){
    case 1:
      subs_Th_J1 = nh.subscribe("/sh_thj1_position_controller/state",1000, _j1Callback);
	     subs_Th_J2 = nh.subscribe("/sh_thj2_position_controller/state",1000, _j2Callback);
	     subs_Th_J3 = nh.subscribe("/sh_thj3_position_controller/state",1000, _j3Callback);
	     subs_Th_J4 = nh.subscribe("/sh_thj4_position_controller/state",1000, _j4Callback);
	     subs_Th_J5 = nh.subscribe("/sh_thj5_position_controller/state",1000, _j5Callback);
	break;
    case 2:
      subs_Ff_J0 = nh.subscribe("/sh_ffj0_position_controller/state", 1000, _j0Callback);
      subs_Ff_J3 = nh.subscribe("/sh_ffj3_position_controller/state", 1000, _j3Callback);
      subs_Ff_J4 = nh.subscribe("/sh_ffj4_position_controller/state", 1000, _j4Callback);
	break;
  case 3:  
      subs_Mf_J0 = nh.subscribe("/sh_mfj0_position_controller/state", 1000, _j0Callback);
      subs_Mf_J3 = nh.subscribe("/sh_mfj3_position_controller/state", 1000, _j3Callback);
      subs_Mf_J4 = nh.subscribe("/sh_mfj4_position_controller/state", 1000, _j4Callback);
	break;
  case 4:
      subs_Rf_J0 = nh.subscribe("/sh_rfj0_position_controller/state", 1000, _j0Callback);
      subs_Rf_J3 = nh.subscribe("/sh_rfj3_position_controller/state", 1000, _j3Callback);
      subs_Rf_J4 = nh.subscribe("/sh_rfj4_position_controller/state", 1000, _j4Callback);
	break;
  case 5:
      subs_Lf_J0 = nh.subscribe("/sh_lfj0_position_controller/state", 1000, _j0Callback);
      subs_Lf_J3 = nh.subscribe("/sh_lfj3_position_controller/state", 1000, _j3Callback);
      subs_Lf_J4 = nh.subscribe("/sh_lfj4_position_controller/state", 1000, _j4Callback);
      subs_Lf_J5 = nh.subscribe("/sh_lfj5_position_controller/state", 1000, _j5Callback);
	break;
  }

  subs_fake_controller = nh.subscribe("/move_group/fake_controller_joint_states",1000, fake_control_Callback);

  // Cargar modelo
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
  sharedKinematic_model	= kinematic_model;

  // Definir kinematic state
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  sharedKinematic_state = kinematic_state;
  sharedKinematic_state->setToDefaultValues();
  
  /**
  switch(finger){
    case 1:
	   joint_model = kinematic_model->getJointModelGroup("thumb");
      ROS_INFO("Joint model = thumb");
	   *tip_name = "thtip";
	   break;
    case 2:
    */
	   joint_model = kinematic_model->getJointModelGroup("first_finger");
      /** ROS_INFO("Joint model = first finger");
	   *tip_name = "fftip";
	   break;
    case 3:  
	   joint_model = kinematic_model->getJointModelGroup("middle_finger");
       ROS_INFO("Joint model = middle finger");
	   *tip_name = "mftip";
	   break; 
    case 4:
	   joint_model = kinematic_model->getJointModelGroup("ring_finger");
       ROS_INFO("Joint model = ring finger");
	   *tip_name = "rftip";
	   break;	
    case 5:
	   joint_model = kinematic_model->getJointModelGroup("little_finger");
       ROS_INFO("Joint model = little finger");
	   *tip_name = "lftip";
	   break;
  }
  */
  ros::spin();
  return 0;
}
