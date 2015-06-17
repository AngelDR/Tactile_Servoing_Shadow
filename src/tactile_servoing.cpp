

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
#include <sensor_msgs/JointState.h>

using namespace Eigen;
using namespace std;

/**void pinv( MatrixXd& pinvmat) //const
{
   eigen_assert(m_isInitialized && "SVD is not initialized.");
   double  pinvtoler=1.e-6; // choose your tolerance wisely!
   SingularValuesType singularValues_inv=m_singularValues;
   for ( long i=0; i<m_workMatrix.cols(); ++i) {
      if ( m_singularValues(i) > pinvtoler )
         singularValues_inv(i)=1.0/m_singularValues(i);
     else singularValues_inv(i)=0;
   }
   pinvmat= (m_matrixV*singularValues_inv.asDiagonal()*m_matrixU.transpose());
} */

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

  // >> PUBLISHER PARA FAKE CONTROLLER -> SIMULACION
  ros::Publisher fake_controller = node_handle.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states", 1000);
  sensor_msgs::JointState fake_states;
  std::string name_list[24] = {"WRJ2", "WRJ1", "FFJ4", "FFJ3", "FFJ2", "FFJ1", "MFJ4", "MFJ3", "MFJ2", "MFJ1", "RFJ4", "RFJ3", "RFJ2", "RFJ1", "LFJ5", "LFJ4", "LFJ3", "LFJ2", "LFJ1", "THJ5", "THJ4", "THJ3", "THJ2", "THJ1"};
  double position_list[24] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  for(int i = 0; i < 24; i++ ){
    fake_states.name.push_back(name_list[i]);
    fake_states.position.push_back(position_list[i]);
  }
  ROS_INFO("fake_states positions %i", fake_states.position.size());

  
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


  // >> Marker visualization forces
  ros::Publisher vis_pub = node_handle.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
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
  Matrix4d transformation_matrix;
  MatrixXd vector_matrix(1,3);
  Matrix3d mult_matrix; 
  MatrixXd adjoint_matrix(6,6);

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
                        0, 0, 0.005, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0,
                        0, 0, 0, 0, 0, 0;


  /**
  * Jacobiana inversa
  * Definición:
  */
  inverse_jacobian <<   1, 0, 0, 0,
                        0, 1, 0, 0,
                        0, 0, 1, 0,
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
  pid_controller.initPid(-0.5,0.0,0.0,0.0,0.0);
  
  //double value_position = 0.8;
  //vel_ff_j3_pub.publish(value_position);

  double f_desired = 1.2;
  double f_t_current = 0.0;
  double f_t_previous= 0.0;

  ros::Time last_time = ros::Time::now();
  
  error_matrix(0,0) = 0.0;
  error_matrix(1,0) = 0.0;
  error_matrix(2,0) = 0.0;
  error_matrix(3,0) = 0.0;
  ROS_INFO("Parameters : ok");


  // prubas
  double var_pos = 0.1;  
  do
  {    
    if (pressure_client.call(srv_pressure))
    {
    
      f_t_current = srv_pressure.response.applied_force[1]; 
      double error_f = f_desired - f_t_current;
      error_matrix(2,0) =  - error_f;

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
      error_matrix(0,0) = - error_x;
      error_matrix(1,0) = - error_y;
      ROS_INFO("/Servoing: Centroide presión x = %i, y = %i", max_position_i, max_position_j);
      ROS_INFO("/Servoing: Error x = %f, y = %f, f = %f, o = %f ", error_matrix(0,0), error_matrix(1,0), error_matrix(2,0), error_matrix(3,0));

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
      ROS_INFO("/Servoing: Error * [pid] x = %f, y = %f, f = %f, o = %f ", error_matrix(0,0), error_matrix(1,0), error_matrix(2,0), error_matrix(3,0));
      ROS_INFO("/Servoing: velocity  x = %f, y = %f, z = %f, rotx = %f, roty = %f, rotz = %f ", sensor_velocity(0,0), sensor_velocity(1,0), sensor_velocity(2,0), sensor_velocity(3,0), sensor_velocity(4,0), sensor_velocity(5,0));

      // Velocity tip = vel. sensor ??  
      // Transformar velocity sensor -> frame Sensor -> frame tip
      //tfBuffer.waitForTransform("palm","sensor_ff");
      try{
        ros::Time now = ros::Time::now();
        tf::StampedTransform transform;
        tfListener.waitForTransform("palm", "ffsensor", now, ros::Duration(3.0));
        tfListener.lookupTransform("palm", "ffsensor", now, transform);
        //ROS_INFO("Transformation  x = %f, y = %f, z = %f)", transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
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

        // Transformar velocity sensor -> frame tip -> palm
        // construir Adj(T) a partir de rotation matrix & traslation
        vector_matrix << transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ();

        mult_matrix <<   transformation_matrix(0,0), transformation_matrix(0,1), transformation_matrix(0,2),
                                  transformation_matrix(1,0), transformation_matrix(1,1), transformation_matrix(1,2), 
                                  transformation_matrix(2,0), transformation_matrix(2,1), transformation_matrix(2,2);
        mult_matrix = vector_matrix * mult_matrix;
        adjoint_matrix <<  transformation_matrix(0,0), transformation_matrix(0,1), transformation_matrix(0,2), mult_matrix(0,0), mult_matrix(0,1), mult_matrix(0,2),
                                    transformation_matrix(1,0), transformation_matrix(1,1), transformation_matrix(1,2), mult_matrix(1,0), mult_matrix(1,1), mult_matrix(1,2),
                                    transformation_matrix(2,0), transformation_matrix(2,1), transformation_matrix(2,2), mult_matrix(2,0), mult_matrix(2,1), mult_matrix(2,2),
                                    0,0,0,transformation_matrix(0,0), transformation_matrix(0,1), transformation_matrix(0,2),
                                    0,0,0,transformation_matrix(1,0), transformation_matrix(1,1), transformation_matrix(1,2),
                                    0,0,0,transformation_matrix(2,0), transformation_matrix(2,1), transformation_matrix(2,2);

        // Vel sensor -> frame palm ->
        sensor_velocity = adjoint_matrix * sensor_velocity;
      }catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }

      // Get finger jacobian
      MatrixXd jacobian_matrix(6,4);
      MatrixXd ata_matrix(6,6);
      MatrixXd jacobian_matrix_inverse(6,4);
      int pos = 0;
      if (jacobian_client.call(srv_jacobian))
      {
        for(int i = 0;i < 6;i++){
          for(int j = 0; j < 4;j++){
            //ROS_INFO("Response position = %f", srv_jacobian.response.jacobian[pos]);
            jacobian_matrix(i,j) = srv_jacobian.response.jacobian[pos];
            pos++;
          }
        }    
      }
      else
      {
        ROS_ERROR("Failed to call service jacobian");
        return 1;
      }

      // ¨q  =  j (^-1) * V tip   -> obtener qs
      //ata_matrix = jacobian_matrix.traspose();
      ROS_INFO_STREAM("Jacobian: \n" << jacobian_matrix);
      ROS_INFO_STREAM("Jacobian transpose: \n" << jacobian_matrix.transpose());
      ata_matrix = jacobian_matrix * jacobian_matrix.transpose();
      ROS_INFO_STREAM("Jacobian * Jacobian transpose: \n" << ata_matrix);
      ata_matrix = ata_matrix.inverse();
      //pinv(ata_matrix);
      ROS_INFO_STREAM("Inverse (Jacobian * Jacobian transpose): \n" << ata_matrix);
      jacobian_matrix_inverse = jacobian_matrix.transpose() * ata_matrix;
      ROS_INFO_STREAM("Jacobian pseudo inverse: \n" << jacobian_matrix_inverse);


      MatrixXd q_velocities;
      q_velocities = jacobian_matrix_inverse * sensor_velocity; 
      ROS_INFO("Q velocities : \n%f   %f   %f   %f\n", q_velocities(0,0), q_velocities(0,1), q_velocities(0,2), q_velocities(0,3));


      // >> PUBLISH STATES
      // FAKE STATES
      fake_states.header.stamp = ros::Time::now();
      fake_states.header.frame_id = "fake_states";
      ROS_INFO("fake_states header");
      /**int i;
      for(i = 0 ; i < 24 ; i++){
        fake_states.name.push_back(name_list[i]);
        fake_states.position.push_back(position_list[i]);
      }*/
      ROS_INFO("fake_states positions %d", fake_states.position.size());
      
      //fake_states.position.at(2) = 0.0;
      fake_states.position.at(3) = var_pos;
      //fake_states.position.at(4) = 0.2;
      //fake_states.position.at(5) = 0.2;
      var_pos = var_pos + 0.1;
      if(var_pos == 0.5) { var_pos = 0.1;}

      fake_controller.publish(fake_states);
      ROS_INFO("Position finger %f", var_pos);


      // REAL SHADOW HAND

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
        vis_pub.publish( marker );
      }catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }
                                 
      sleep(0.1);
      ros::spinOnce();
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