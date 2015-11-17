

/* Author: Angel Delgado
 Organization: Universidad de Alicante
 Comentarios: codigo provisional -> hacer modular
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
#include <Eigen/SVD> 
#include <control_toolbox/pid.h>

 // MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <sensor_msgs/JointState.h>

 // Files
#include <iostream>
#include <fstream>

using namespace Eigen;
using namespace std;

/**
Metodo para pseudo inversa. Usado para jac. inversa. 
*/
bool pinv(MatrixXd &orig_matrix, MatrixXd &result, double epsilon = std::numeric_limits<typename MatrixXd::Scalar>::epsilon()) //const
{
  if(orig_matrix.rows()<orig_matrix.cols())
      return false;

  Eigen::JacobiSVD<MatrixXd> svd = orig_matrix.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);

  typename MatrixXd::Scalar tolerance = epsilon * std::max(orig_matrix.cols(), orig_matrix.rows()) * svd.singularValues().array().abs().maxCoeff();
  
  result = svd.matrixV() * MatrixXd(MatrixXd( (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().
      array().inverse(), 0) ).diagonal()) * svd.matrixU().adjoint();
}



/**
Main:  nodo control tactil
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "tactile_servo");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  int id_finger = -1;
  node_handle.getParam("/tactile_servo/id_finger_param", id_finger);
  ROS_INFO("ID FINGER: %d", id_finger);

  // publishers para mano real
   // >> PUBLISHERS PARA CONTROLADORES DE POSICION

  // TODO PARAMETRO -> finger
  /**if (node_handle.getParam("/tactile_reconfiguration/number_fingers", id_finger))
  {
    ROS_INFO("Servo-tactile control for finger : %d", id_finger); 
  }
  else{
      id_finger = 4;
  }*/
  /**ROS_INFO("Argh %s", argv[0]);

  string finger_name = argv[0];

  if(finger_name == "th"){id_finger=0;  ROS_INFO("Servoing thumb");
  }
  else if(finger_name == "ff"){id_finger=1;  ROS_INFO("Servoing first");
  }
  else if(finger_name == "mf"){id_finger=2;  ROS_INFO("Servoing middle");
  }
  else if(finger_name == "rf"){id_finger=3;  ROS_INFO("Servoing ring");
  }
  else if(finger_name == "lf"){id_finger=4;  ROS_INFO("Servoing little");
  }
  else {id_finger=0;  ROS_INFO("No params");
  }*/

  std::string tip_name, group_name, f_desired_param, sensor_frame;
  switch(id_finger){
    case 0:
    {
      ros::Publisher pos_th_j1_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj1_position_controller/command", 1000);
      ros::Publisher pos_th_j2_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj2_position_controller/command", 1000);
      ros::Publisher pos_th_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj3_position_controller/command", 1000);
      ros::Publisher pos_th_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj4_position_controller/command", 1000);
      ros::Publisher pos_th_j5_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj5_position_controller/command", 1000);
      tip_name="thtip";
      group_name="thumb";
      sensor_frame="thsensor";
      f_desired_param = "/tactile_servo_reconfiguration/th_desired_f";
      ROS_INFO("Thumb");
      break;
    }
    case 1:
    {
      ros::Publisher pos_ff_j0_pub = node_handle.advertise<std_msgs::Float64>("/sh_ffj0_position_controller/command", 1000);
      ros::Publisher pos_ff_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_ffj3_position_controller/command", 1000);
      ros::Publisher pos_ff_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_ffj4_position_controller/command", 1000);
      tip_name="fftip";
      group_name="first_finger";
      sensor_frame="ffsensor";
      f_desired_param = "/tactile_servo_reconfiguration/ff_desired_f";
      ROS_INFO("first_finger");
      break;
    }
    case 2:
    {
      ros::Publisher pos_mf_j0_pub = node_handle.advertise<std_msgs::Float64>("/sh_mfj0_position_controller/command", 1000);
      ros::Publisher pos_mf_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_mfj3_position_controller/command", 1000);
      ros::Publisher pos_mf_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_mfj4_position_controller/command", 1000);
      tip_name="mftip";
      group_name="middle_finger";
      sensor_frame="mfsensor";
      f_desired_param = "/tactile_servo_reconfiguration/mf_desired_f";
      ROS_INFO("middle_finger");
      break;
    }
    case 3:
    {
      ros::Publisher pos_rf_j0_pub = node_handle.advertise<std_msgs::Float64>("/sh_rfj0_position_controller/command", 1000);
      ros::Publisher pos_rf_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_rfj3_position_controller/command", 1000);
      ros::Publisher pos_rf_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_rfj4_position_controller/command", 1000); 
      tip_name="rftip";
      group_name="ring_finger";
      sensor_frame="rfsensor";
      f_desired_param = "/tactile_servo_reconfiguration/rf_desired_f";
      ROS_INFO("ring_finger");
      break;
    }
    case 4:
    {
      ros::Publisher pos_lf_j0_pub = node_handle.advertise<std_msgs::Float64>("/sh_lfj0_position_controller/command", 1000);
      ros::Publisher pos_lf_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_lfj3_position_controller/command", 1000);
      ros::Publisher pos_lf_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_lfj4_position_controller/command", 1000);
      ros::Publisher pos_lf_j5_pub = node_handle.advertise<std_msgs::Float64>("/sh_lfj5_position_controller/command", 1000);
      tip_name="lftip";
      group_name="little_finger";
      sensor_frame="lfsensor";
      f_desired_param = "/tactile_servo_reconfiguration/lf_desired_f";
      ROS_INFO("little_finger");
      break;
    }
  }

  // Cargar modelo (MoveIt)
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  kinematic_state->setToDefaultValues();

  // TODO : group and eef -> param
  const robot_state::JointModelGroup* joint_model_group;
  joint_model_group = kinematic_model->getJointModelGroup(group_name);
  ROS_INFO("Joint models: ok");

  // Clases para transformaciones con tf2
  tf2_ros::Buffer tfBuffer;
  tf::TransformListener tfListener; 
  geometry_msgs::TransformStamped _transf;

  // >> SERVICIOS PARA CAMBIAR CONTROLADORES
  // >> pr2_controller_manager -> usado por paquetes Shadow para cargar / arrancar / cambiar controladores
  ros::ServiceClient load_controller_client = node_handle.serviceClient<pr2_mechanism_msgs::LoadController>("pr2_controller_manager/load_controller");
  ros::ServiceClient switch_controller_client = node_handle.serviceClient<pr2_mechanism_msgs::SwitchController>("pr2_controller_manager/switch_controller");   
  
  // >> PRESSURE MAP SERVICE
  ros::ServiceClient pressure_client = node_handle.serviceClient<tekscan_client::GetPressureMap>("GetPressureMap");
  tekscan_client::GetPressureMap srv_pressure;
  srv_pressure.request.num = 0;

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
  MatrixXd sensor_displacement(6,1);
  Matrix4d sensor_values;
  Matrix4d transformation_matrix;
  Vector3d vector_matrix;
  Matrix3d mult_matrix; 
  MatrixXd adjoint_matrix(6,6);
  Vector3d v1;
  Vector3d v2;
  Vector3d v3;
  /**
  * Projection Matrix:
  * Elementos diagonal:
  * (1,1)&(3,3): servoing position  -> corresponder con ejes reales de transformacion en Shadow, para obtener vels. x,y,z
  * (2,2): force servoing
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
  inverse_jacobian <<   0.01, 0, 0, 0,
                        0, 0.01, 0, 0,
                        0, 0.02, 0.01, 0,
                        0, 1, 0, 0,
                        1, 0, 0, 0,
                        0, 0, 0, 1;
  bool success;
    

  /** Iniciar PID */
  /** (p,i,d,i_max, d_max)*/
  int _p_param, _i_param, _d_param;
  if (node_handle.getParam("/tactile_servo_reconfiguration/p", _p_param))
  {
    ROS_INFO("Param P : %d", _p_param); 
  }
  else{
      _p_param = 0.5;
  }

  if (node_handle.getParam("/tactile_servo_reconfiguration/i", _i_param))
  {
    ROS_INFO("Param I : %d", _p_param); 
  }
  else{
    _i_param = 0.5;
  }

  if (node_handle.getParam("/tactile_servo_reconfiguration/d", _d_param))
  {
    ROS_INFO("Param D : %d", _d_param); 
  }
  else{
    _d_param = 0.5;
  }
  pid_controller.initPid(_p_param,_i_param,_d_param,0.0,0.0);

  // PARAM -> f_desired
  double f_desired;   // newtons
  if (node_handle.getParam(f_desired_param, f_desired))
  {
    ROS_INFO("Param desired F : %d", f_desired); 
  }
  else{
    f_desired = 0.3;
  }
  
  double f_t_current = 0.0; 
  double f_t_previous= 0.0;
  ros::Time last_time = ros::Time::now();
  error_matrix(0,0) = 0.0;
  error_matrix(1,0) = 0.0;
  error_matrix(2,0) = 0.0;
  error_matrix(3,0) = 0.0;
  ROS_INFO("Parameters : ok");

  // Inicio bucle Tactle servoing
  int iteration = 0;
  do
  {   
      ROS_INFO("Servoing...");
      if (pressure_client.call(srv_pressure))
      {

        /**
        *  ++++++++++++++              OBTENER ERROR VALORES DESEADOS -> VALORES ACTUALES   +++++++++++++++++++++++++++++++++++++++++++
        */
        f_t_current = srv_pressure.response.applied_force[id_finger]; 
        double error_f = f_desired - f_t_current;
        error_matrix(1,0) = error_f;
        // 2  - get pos(x,y) error
        // pos(x,y) desired = [1,2]

        // VALORES PARA DEDO ACTUAL:
        double finger_force_values[16];
        switch(id_finger)
        {
          case 0:
          {
            for(int pos=0; pos<16; pos++){
              finger_force_values[pos] = srv_pressure.response.th_values[pos];
            }
            break;
          }

          case 1:
          {
            for(int pos=0; pos<16; pos++){
              finger_force_values[pos] = srv_pressure.response.ff_values[pos];
            }
            break;
          }

          case 2:
          {
            for(int pos=0; pos<16; pos++){
              finger_force_values[pos] = srv_pressure.response.mf_values[pos];
            }
            break;
          }

          case 3:
          {
            for(int pos=0; pos<16; pos++){
              finger_force_values[pos] = srv_pressure.response.rf_values[pos];
            }
            break;
          }

          case 4:
          {
            for(int pos=0; pos<16; pos++){
              finger_force_values[pos] = srv_pressure.response.lf_values[pos];
            }
            break;
          }
        }

        double max_value = finger_force_values[0];
        int iter = 0;
        int max_position_i,max_position_j = 0;
        for(int i = 0;i < 4; i++)
        {
          for(int j = 0; j < 4; j++)
          {
            sensor_values(i,j) = finger_force_values[iter]; 
            if (finger_force_values[iter] > max_value)
            {
              max_position_i = i;
              max_position_j = j;
              max_value = finger_force_values[iter];
            }
            iter++;
          }
        }
        // error en X e Y depende de las medidas del sensor. 17mm x 17mm
        // distancia entre cada celdilla -> x = 4mm; y 4mm
        double error_x = (max_position_i - 1) * 0.004;
        double error_y = (max_position_j - 2) * 0.004;
        error_matrix(0,0) = - error_x;
        error_matrix(2,0) = - error_y;


        /** 
        * ++++++++++++++++++++++++++++++          APLICAR PID  -> OBTENER V_SENSOR  ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
        * 
        */
        ros::Time current_time = ros::Time::now();
        error_matrix(0,0) = pid_controller.updatePid(error_matrix(0,0), current_time - last_time);
        error_matrix(1,0) = pid_controller.updatePid(error_matrix(1,0), current_time - last_time);
        error_matrix(2,0) = pid_controller.updatePid(error_matrix(2,0), current_time - last_time);
        error_matrix(3,0) = pid_controller.updatePid(3.0, current_time - last_time);
        last_time = current_time;
        // Sensor velocity
        sensor_velocity = projection_matrix*inverse_jacobian*error_matrix; 
        //ROS_INFO("/Servoing: velocity (sens)  x = %f, y = %f, z = %f, rotx = %f, roty = %f, rotz = %f ", sensor_velocity(0,0), sensor_velocity(1,0), sensor_velocity(2,0), sensor_velocity(3,0), sensor_velocity(4,0), sensor_velocity(5,0));
        //vel_file << sensor_velocity(0,0) << " " << sensor_velocity(1,0) << " " << sensor_velocity(2,0) << " ";
        // Velocity tip = vel. sensor ??  
        // Transformar velocity sensor -> frame Sensor -> frame tip
        //tfBuffer.waitForTransform("palm","sensor_ff");


        /**
        * ++++++++++++++++++++++++++++++++++     TRANSFORMAR V_SENSOR A V_TIP (PALM) +++++++++++++++++++++++++++++++++++++++++++++++++
        *   
        */
        try{
          ros::Time now = ros::Time::now();
          tf::StampedTransform transform_s;
          tfListener.waitForTransform("palm", sensor_frame, now, ros::Duration(8.0));
          tfListener.lookupTransform("palm", sensor_frame, now, transform_s);
          tf::Quaternion q = transform_s.getRotation();
          tf::Matrix3x3 rotation_matrix(q);
          transformation_matrix <<  rotation_matrix[0].getX(), rotation_matrix[0].getY(), rotation_matrix[0].getZ(), transform_s.getOrigin().getX(), 
                                    rotation_matrix[1].getX(), rotation_matrix[1].getY(), rotation_matrix[1].getZ(), transform_s.getOrigin().getY(),
                                    rotation_matrix[2].getX(), rotation_matrix[2].getY(), rotation_matrix[2].getZ(), transform_s.getOrigin().getZ(),
                                    0,0,0,1;
          // Transformar velocity sensor -> frame tip -> palm
          // construir Adj(T) a partir de rotation matrix & traslation
          vector_matrix << transform_s.getOrigin().getX(), transform_s.getOrigin().getY(), transform_s.getOrigin().getZ();
          
          v1 << transformation_matrix(0,0), transformation_matrix(1,0), transformation_matrix(2,0);
          v2 << transformation_matrix(0,1), transformation_matrix(1,1), transformation_matrix(2,1);
          v3 << transformation_matrix(0,2), transformation_matrix(1,2), transformation_matrix(2,2);

          v1 = v1.cross(vector_matrix);
          v2 = v2.cross(vector_matrix);
          v3 = v3.cross(vector_matrix);
          mult_matrix <<   v1(0,0), v2(0,0), v3(0,0),
                          v1(1,0), v2(1,0), v3(1,0), 
                          v1(2,0), v2(2,0), v3(2,0);
          adjoint_matrix <<  transformation_matrix(0,0), transformation_matrix(0,1), transformation_matrix(0,2), mult_matrix(0,0), mult_matrix(0,1), mult_matrix(0,2),
                                      transformation_matrix(1,0), transformation_matrix(1,1), transformation_matrix(1,2), mult_matrix(1,0), mult_matrix(1,1), mult_matrix(1,2),
                                      transformation_matrix(2,0), transformation_matrix(2,1), transformation_matrix(2,2), mult_matrix(2,0), mult_matrix(2,1), mult_matrix(2,2),
                                      0,0,0,transformation_matrix(0,0), transformation_matrix(0,1), transformation_matrix(0,2),
                                      0,0,0,transformation_matrix(1,0), transformation_matrix(1,1), transformation_matrix(1,2),
                                      0,0,0,transformation_matrix(2,0), transformation_matrix(2,1), transformation_matrix(2,2);
          // Vel sensor -> frame palm ->
          sensor_velocity = adjoint_matrix * sensor_velocity;
          //ROS_INFO("/Servoing: velocity (palm):  x = %f, y = %f, z = %f, rotx = %f, roty = %f, rotz = %f ", sensor_velocity(0,0), sensor_velocity(1,0), sensor_velocity(2,0), sensor_velocity(3,0), sensor_velocity(4,0), sensor_velocity(5,0));   
          


          /**
          * ++++++++++++++++++++++++++   OBTENER DESPLAZAMIENTOS ARTICULACION A PARTIR DE V_TIP
          *
          */  
          try{
            double time_int = 1;
            // desplazamiento respecto a /palm
            sensor_displacement <<  sensor_velocity(0,0) * time_int, sensor_velocity(1,0) * time_int,
                                    sensor_velocity(2,0) * time_int, sensor_velocity(3,0) * time_int,
                                    sensor_velocity(4,0) * time_int, sensor_velocity(5,0) * time_int;
            //ROS_INFO_STREAM("Displacement " << sensor_displacement);
            ros::Time now = ros::Time::now();
            tf::StampedTransform transform_sensor_palm;
            tfListener.waitForTransform("forearm", tip_name, now, ros::Duration(3.0));
            tfListener.lookupTransform("forearm", tip_name, now, transform_sensor_palm);
            //ROS_INFO("Posicion original: x=%f, y=%f, z=%f ", transform_sensor_palm.getOrigin().getX(),transform_sensor_palm.getOrigin().getY(),transform_sensor_palm.getOrigin().getZ());

            // Prueba IK
            const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform(tip_name);
            //ROS_INFO(" \n End effector state \n");
            //ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation());
            // 

            // obtener transformacion /palm a /forearm . 
            tf::Vector3 new_origin;
            new_origin.setX((transform_sensor_palm.getOrigin().getX()) + sensor_displacement(0,0));
            new_origin.setY((transform_sensor_palm.getOrigin().getY()) + sensor_displacement(1,0));
            new_origin.setZ((transform_sensor_palm.getOrigin().getZ()) + sensor_displacement(2,0));
            transform_sensor_palm.setOrigin(new_origin);
            //ROS_INFO("Posicion final: x=%f, y=%f, z=%f ",new_origin.getX(),new_origin.getY(),new_origin.getZ());

            //const Eigen::Affine3d &end_effector_state;
            geometry_msgs::Pose sensor_position;
            sensor_position.position.x = new_origin.getX();
            sensor_position.position.y = new_origin.getY();
            sensor_position.position.z = new_origin.getZ();

            sensor_position.orientation.x = transform_sensor_palm.getRotation().getAxis().getX();
            sensor_position.orientation.y = transform_sensor_palm.getRotation().getAxis().getY();
            sensor_position.orientation.z = transform_sensor_palm.getRotation().getAxis().getZ();
            sensor_position.orientation.w = transform_sensor_palm.getRotation().getW();
            
            /**
            std::vector<double> joint_values;
            const std::vector<std::string> &joint_names = joint_model_ff->getJointModelNames();
            //ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation());
            //ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation());

            bool found_ik = kinematic_state->setFromIK(joint_model_ff, sensor_position, 2, 0.1);
            if (found_ik)
            {
              kinematic_state->copyJointGroupPositions(joint_model_ff, joint_values);
              for(std::size_t i=0; i < joint_names.size(); ++i)
              {   
                fake_states.position.at(i+2) = joint_values[i];
                //ROS_INFO("IK para Joint %s: %f", joint_names[i].c_str(), joint_values[i]);  
              }
          
                pos_ff_j3_pub.publish(joint_values[1]);
                sleep(0.1);
                pos_ff_j0_pub.publish(joint_values[2]);
                sleep(0.1);
            }
            else
            {
              ROS_INFO("Did not find IK solution -> joint approximation");
              if(error_f > 0.0){
                fake_states.position.at(3) = fake_states.position.at(3) + 0.05;
                //fake_states.position.at(4) = fake_states.position.at(4) + 0.03;
                //fake_states.position.at(5) = fake_states.position.at(5) + 0.03;
              }
              else{
                fake_states.position.at(3) = fake_states.position.at(3) - 0.05;
                //fake_states.position.at(4) = fake_states.position.at(4) - 0.03;
                //fake_states.position.at(5) = fake_states.position.at(5) - 0.03;
              }

              pos_ff_j3_pub.publish(fake_states.position.at(3));
              sleep(0.1);
              //pos_ff_j0_pub.publish(joint_values[4]);
              //sleep(0.1);
            }

            fake_states.position.at(3) = fake_states.position.at(3) + (error_f * 0.05);
            fake_states.position.at(4) = fake_states.position.at(4) + (error_f * 0.05);

            */
            /**pos_ff_j3_pub.publish(fake_states.position.at(3));
            sleep(0.1); 
            pos_ff_j0_pub.publish(fake_states.position.at(4));
            sleep(0.1); */ 

          }catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
          }

        }catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }

        // >> PUBLISH STATES
        // FAKE STATES
        /**fake_states.header.stamp = ros::Time::now();
        fake_states.header.frame_id = "fake_states";
        fake_controller.publish(fake_states); */
                                   
        //sleep(2.0);
        // Comportamiento task planner:
        ros::spinOnce();

      }
      else
      {
        ROS_ERROR("Failed to call service pressure");
        return 1;
      }
 
    iteration++;
  }while(ros::ok());

  /**err_file.close();
  vel_file.close();
  forces_file.close();
  desired_forces_file.close();*/
  
  ros::shutdown();  
  return 0;
}