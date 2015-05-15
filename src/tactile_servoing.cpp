

/* Author: Angel Delgado
 Organization: Universidad de Alicante
 */

#include <pr2_mechanism_msgs/LoadController.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <pr2_controller_manager/controller_manager.h>

#include "std_msgs/Float64.h"
#include "tekscan_client/GetPressureMap.h"

#include <iostream>
#include <fstream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tactile_servo");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  
  // >> SERVICIOS PARA CAMBIAR CONTROLADORES
  // >> pr2_controller_manager -> usado por paquetes Shadow para cargar / arrancar / cambiar controladores
  ros::ServiceClient load_controller_client = node_handle.serviceClient<pr2_mechanism_msgs::LoadController>("pr2_controller_manager/load_controller");
  ros::ServiceClient switch_controller_client = node_handle.serviceClient<pr2_mechanism_msgs::SwitchController>("pr2_controller_manager/switch_controller");
    
  
  // >> PRESSURE MAP SERVICE
  ros::ServiceClient pressure_client = node_handle.serviceClient<tekscan_client::GetPressureMap>("GetPressureMap");
  tekscan_client::GetPressureMap srv_pressure;
  srv_pressure.request.num = 0;
  
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
  
  
  /**
   * TACTILE SERVO:
   * Vsensor(ref_sensor) = ProjMatrix(6x6) * PID 
   * PID = (Kp * ef(t) + Ki * intgr(ef(t)) + Kd * (ef(t) - ef(t-1)))
   * ef(t) = [ v(x), v(y), v(f), v(alfa)] ; // variaciones de x,y fuerza y orientacion
   *
   *
   * Velocidades(q) = IK (VSensor(ref_origen))
  */
  MatrixXf projection_matrix(6,6);
  MatrixXf inverse_jacobian(6,4);
  MatrixXf error_matrix(4,1);
  
  MatrixXf sensor_velocity(6,1);

  Matrix4f sensor_values;

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
                        0, 0, 1, 0,
                        0, 1, 0, 0,
                        1, 0, 0, 0,
                        0, 0, 0, 1;

  /* This sleep is ONLY to allow Rviz to come up */
  sleep(2.0);
  bool success;
    
  // >> PARAMETROS DE LOS EXPERIMENTOS
  // >> Obtener numero de dedos que se usan en el experimento -> rosparam
  int num_fingers_exp;
  if (node_handle.getParam("/experimento/numero_dedos", num_fingers_exp))
  {
    ROS_INFO("Numero de dedos para el experimento : %d", num_fingers_exp); 
  }

  
  
  double value_position = 0.8;
  vel_ff_j3_pub.publish(value_position);

  double f_desired = 1.2;
  double f_t_current = 0.0;
  double f_t_previous= 0.0;
  double error_f,kp,ki,kd;
  error_matrix(0,0) = 0;
  error_matrix(1,0) = 0;
  error_matrix(2,0) = 0;
  error_matrix(3,0) = 0;

  kp = 1;
  
  do
  {
    // Limitar posicionamiento
    //if (value_position < 0.0) value_position = 0;
    //if (value_position > 1.5) value_position = 1.5;
    
    if (pressure_client.call(srv_pressure))
    {
      /**if (srv_pressure.response.applied_force[1] > 0.5 )
      {
	       value_position = value_position - 0.1;
	       vel_ff_j3_pub.publish(value_position);
	       ROS_INFO("/Servoing: pos = %f", value_position);
	       //sleep(2.0);
      }
      else
      {
	       value_position = value_position + 0.1;
	       vel_ff_j3_pub.publish(value_position);
	       ROS_INFO("/Servoing: pos = %f", value_position);
	       //sleep(2.0);
      }*/
      // 1 -  get force error
      f_t_current = srv_pressure.response.applied_force[1]; 
      error_f = f_desired - f_t_current;
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

      int error_x = 1 - max_position_i;
      int error_y = 2 - max_position_j;
      error_matrix(0,0) = error_x;
      error_matrix(1,0) = error_y;

      // Sensor velocity
      sensor_velocity = projection_matrix*inverse_jacobian*((kp*error_matrix)); // + (ki*integral(error_matrix)) + kd(error_matrix(current) - error_matrix(previous)));  
      ROS_INFO("Force error = %f", error_f); 
      ROS_INFO("/Servoing: velocity  x = %f, y = %f, z = %f ", sensor_velocity(0,0), sensor_velocity(1,0), sensor_velocity(2,0));
   
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