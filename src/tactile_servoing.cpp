

/* Author: Angel Delgado
 Organization: Universidad de Alicante
 */

#include <pr2_mechanism_msgs/LoadController.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <pr2_controller_manager/controller_manager.h>

#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "std_msgs/Float64.h"
#include "tekscan_client/GetPressureMap.h"
#include "tactile_servoing_shadow/getFingerJacobianMatrix.h" 

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <control_toolbox/pid.h>

 // MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>

using namespace Eigen;
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tactile_servo");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Definir robot (MoveIt)
  // Cargar modelo
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Moveit Model : ok");
  // Shared robot_model & robot_state 
  // Definir kinematic state
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  ROS_INFO("Kinematic State : ok");
  kinematic_state->setToDefaultValues();
  ROS_INFO("Joint model to default values : ok");
  const robot_state::JointModelGroup* joint_model;
  std::string tip_name;
  joint_model = kinematic_model->getJointModelGroup("first_finger");
  tip_name = "fftip";
  ROS_INFO("Joint model first finger : ok");


  // Clases para transformaciones con tf2
  tf2_ros::Buffer tfBuffer;
  tf::TransformListener tfListener; 
  geometry_msgs::TransformStamped ff_transf;

  // >> SERVICIOS PARA CAMBIAR CONTROLADORES
  // >> pr2_controller_manager -> usado por paquetes Shadow para cargar / arrancar / cambiar controladores
  ros::ServiceClient load_controller_client = node_handle.serviceClient<pr2_mechanism_msgs::LoadController>("pr2_controller_manager/load_controller");
  ros::ServiceClient switch_controller_client = node_handle.serviceClient<pr2_mechanism_msgs::SwitchController>("pr2_controller_manager/switch_controller");   
  
  // >> PRESSURE MAP SERVICE
  ros::ServiceClient pressure_client = node_handle.serviceClient<tekscan_client::GetPressureMap>("GetPressureMap");
  tekscan_client::GetPressureMap srv_pressure;
  srv_pressure.request.num = 0;

  // >> FINGER JACOBIAN SERVICE 
  ros::ServiceClient jacobian_client = node_handle.serviceClient<tactile_servoing_shadow::getFingerJacobianMatrix>("GetJacobianMatrix");
  tactile_servoing_shadow::getFingerJacobianMatrix srv_jacobian;
  srv_jacobian.request.finger_id = 2;
  
  // >> PUBLISHERS PARA CONTROLADORES DE VELOCIDAD
  ros::Publisher vel_ff_j0_pub = node_handle.advertise<std_msgs::Float64>("/sh_ffj0_mixed_position_velocity_controller/command", 1000);
  ros::Publisher vel_ff_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_ffj3_mixed_position_velocity_controller/command", 1000);
  ros::Publisher vel_ff_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_ffj4_mixed_position_velocity_controller/command", 1000);
  
  ros::Publisher vel_mf_j0_pub = node_handle.advertise<std_msgs::Float64>("/sh_mfj0_mixed_position_velocity_controller/command", 1000);
  ros::Publisher vel_mf_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_mfj3_mixed_position_velocity_controller/command", 1000);
  ros::Publisher vel_mf_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_mfj4_mixed_position_velocity_controller/command", 1000);
  
  ros::Publisher vel_rf_j0_pub = node_handle.advertise<std_msgs::Float64>("/sh_rfj0_mixed_position_velocity_controller/command", 1000);
  ros::Publisher vel_rf_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_rfj3_mixed_position_velocity_controller/command", 1000);
  ros::Publisher vel_rf_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_rfj4_mixed_position_velocity_controller/command", 1000);
  
  ros::Publisher vel_lf_j0_pub = node_handle.advertise<std_msgs::Float64>("/sh_lfj0_mixed_position_velocity_controller/command", 1000);
  ros::Publisher vel_lf_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_lfj3_mixed_position_velocity_controller/command", 1000);
  ros::Publisher vel_lf_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_lfj4_mixed_position_velocity_controller/command", 1000);
  ros::Publisher vel_lf_j5_pub = node_handle.advertise<std_msgs::Float64>("/sh_lfj5_mixed_position_velocity_controller/command", 1000);

  ros::Publisher vel_th_j1_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj1_mixed_position_velocity_controller/command", 1000);
  ros::Publisher vel_th_j2_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj2_mixed_position_velocity_controller/command", 1000);
  ros::Publisher vel_th_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj3_mixed_position_velocity_controller/command", 1000);
  ros::Publisher vel_th_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj4_mixed_position_velocity_controller/command", 1000);
  ros::Publisher vel_th_j5_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj5_mixed_position_velocity_controller/command", 1000);
  
  ros::Publisher vel_wr_j1_pub = node_handle.advertise<std_msgs::Float64>("/sh_wrj1_mixed_position_velocity_controller/command", 1000);
  ros::Publisher vel_wr_j2_pub = node_handle.advertise<std_msgs::Float64>("/sh_wrj2_mixed_position_velocity_controller/command", 1000);
  ROS_INFO("Topics : ok");

  /**
   * TACTILE SERVO:
   * Vsensor(ref_sensor) = ProjMatrix(6x6) * PID 
   * PID = (Kp * ef(t) + Ki * intgr(ef(t)) + Kd * (ef(t) - ef(t-1)))
   * ef(t) = [ v(x), v(y), v(f), v(alfa)] ; // variaciones de x,y fuerza y orientacion
   *
   *
   * Velocidades(q) = IK (VSensor(ref_origen))
  */

  control_toolbox::Pid pid_controller;
  MatrixXd projection_matrix(6,6);
  MatrixXd inverse_jacobian(6,4);
  MatrixXd error_matrix(4,1);
  MatrixXd sensor_velocity(6,1);
  Matrix4d sensor_values;

  /**
  * Projection Matrix:
  * Elementos diagonal:
  * (1,1)&(2,2): servoing position
  * (3,3): force servoing
  * (4,4)&(5,5): rolling 
  * (6,6): orientation
  */
  projection_matrix <<  1, 0, 0, 0, 0, 0,
                        0, 1, 0, 0, 0, 0,
                        0, 0, 1, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0;


  /**
  * Jacobiana inversa
  * Definición:
  */
  inverse_jacobian <<   1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 0.005, 0,
                        0, 1, 0, 0,
                        1, 0, 0, 0,
                        0, 0, 0, 1;

 
  bool success;
    
  // >> PARAMETROS DE LOS EXPERIMENTOS
  // >> Obtener numero de dedos que se usan en el experimento -> rosparam
  int num_fingers_exp;
  if (node_handle.getParam("/experimento/numero_dedos", num_fingers_exp))
  {
    ROS_INFO("Numero de dedos para el experimento : %d", num_fingers_exp); 
  }

  /** Iniciar PID */
  /** (p,i,d,i_max, d_max)*/
  pid_controller.initPid(0.8,0.0,0.0,0.0,0.0);
  
  double value_position = 0.8;
  vel_ff_j3_pub.publish(value_position);

  double f_desired = 1.2;
  double f_t_current = 0.0;
  double f_t_previous= 0.0;

  ros::Time last_time = ros::Time::now();
  
  error_matrix(0,0) = 0.0;
  error_matrix(1,0) = 0.0;
  error_matrix(2,0) = 0.0;
  error_matrix(3,0) = 0.0;
  ROS_INFO("Parameters : ok");

  Matrix4d transformation_matrix;
  
  do
  {    
    if (pressure_client.call(srv_pressure))
    {
    
      f_t_current = srv_pressure.response.applied_force[1]; 
      double error_f = f_desired - f_t_current;
      error_matrix(2,0) = error_f;

      // 2  - get pos(x,y) error
      // pos(x,y) desired = [1,2]
      double max_value = srv_pressure.response.ff_values[0];
      int iter = 0;
      int max_position_i,max_position_j = 0;

      for(int i = 0;i < 4; i++)
      {
        for(int j = 0; j < 4; j++)
        {
          sensor_values(i,j) = srv_pressure.response.ff_values[iter]; 
          if (srv_pressure.response.ff_values[iter] > max_value)
          {
            max_position_i = i;
            max_position_j = j;
            max_value = srv_pressure.response.ff_values[iter];
          }
          iter++;
        }
      }

      //double error_x = 1.0 - (double) max_position_i;
      //double error_y = 2.0 - (double) max_position_j;
      // error en X e Y depende de las medidas del sensor. 17mm x 17mm
      // distancia entre cada celdilla -> x = 4mm; y 4mm
      double error_x = (max_position_i - 1) * 0.004;
      double error_y = (max_position_j - 2) * 0.004;
      error_matrix(0,0) = error_x;
      error_matrix(1,0) = error_y;
      ROS_INFO("/Servoing: Centroide presión x = %i, y = %i", max_position_i, max_position_j);
      ROS_INFO("/Servoing: pid value - pre PID x = %f, y = %f, f = %f, o = %f ", error_matrix(0,0), error_matrix(1,0), error_matrix(2,0), error_matrix(3,0));

      // Aplicar PID a errores
      ros::Time current_time = ros::Time::now();
      error_matrix(0,0) = pid_controller.updatePid(error_matrix(0,0), current_time - last_time);
      error_matrix(1,0) = pid_controller.updatePid(error_matrix(1,0), current_time - last_time);
      error_matrix(2,0) = pid_controller.updatePid(error_matrix(2,0), current_time - last_time);
      error_matrix(3,0) = pid_controller.updatePid(3.0, current_time - last_time);
      last_time = current_time;

      // Sensor velocity
      sensor_velocity = projection_matrix*inverse_jacobian*error_matrix; 
      //ROS_INFO("Force error = %f", error_f); 
      ROS_INFO("/Servoing: pid value - final PID x = %f, y = %f, f = %f, o = %f ", error_matrix(0,0), error_matrix(1,0), error_matrix(2,0), error_matrix(3,0));
      ROS_INFO("/Servoing: velocity  x = %f, y = %f, z = %f ", sensor_velocity(0,0), sensor_velocity(1,0), sensor_velocity(2,0));

      // Velocity tip = vel. sensor ??  
      // Transformar velocity sensor -> frame Sensor -> frame tip
      //tfBuffer.waitForTransform("palm","sensor_ff");
      try{
        ros::Time now = ros::Time::now();
        tf::StampedTransform transform;
        tfListener.waitForTransform("palm", "ffsensor", now, ros::Duration(3.0));
        tfListener.lookupTransform("palm", "ffsensor", now, transform);
        ROS_INFO("Transformation  x = %f, y = %f, z = %f)", transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
        tf::Quaternion q = transform.getRotation();
        tf::Matrix3x3 rotation_matrix(q);
        transformation_matrix <<  rotation_matrix[0].getX(), rotation_matrix[0].getY(), rotation_matrix[0].getZ(), transform.getOrigin().getX(), 
                                  rotation_matrix[1].getX(), rotation_matrix[1].getY(), rotation_matrix[1].getZ(), transform.getOrigin().getY(),
                                  rotation_matrix[2].getX(), rotation_matrix[2].getY(), rotation_matrix[2].getZ(), transform.getOrigin().getZ(),
                                  0,0,0,1;

        ROS_INFO("Transformation matrix : \n%f   %f   %f   %f\n%f   %f   %f   %f\n%f   %f   %f   %f\n%f   %f   %f   %f\n", transformation_matrix(0,0), transformation_matrix(0,1), transformation_matrix(0,2), transformation_matrix(0,3),
                                          transformation_matrix(1,0), transformation_matrix(1,1), transformation_matrix(1,2), transformation_matrix(1,3),
                                          transformation_matrix(2,0), transformation_matrix(2,1), transformation_matrix(2,2), transformation_matrix(2,3),
                                          transformation_matrix(3,0), transformation_matrix(3,1), transformation_matrix(3,2), transformation_matrix(3,3));

      }catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }

      // Transformar velocity sensor -> frame tip -> palm
      // Get finger jacobian
      MatrixXd jacobian_matrix(6,4);
      int pos = 1;
      if (jacobian_client.call(srv_jacobian))
      {
        for(int i = 0;i < 6;i++){
          for(int j = 0; j < 4;j++){
            ROS_INFO("Response position = %f", srv_jacobian.response.jacobian[pos]);
            jacobian_matrix(i,j) = srv_jacobian.response.jacobian[pos];
            pos++;
          }
        }    
      }
      else
      {
        ROS_ERROR("Failed to call service pressure");
        return 1;
      }

      // ¨q  =  j (^-1) * V tip   -> obtener qs
      
      sleep(0.5);
    }
    else
    {
      ROS_ERROR("Failed to call service pressure");
      return 1;
    }
     
    
  }while(true);
  
  ros::shutdown();  
  return 0;
}