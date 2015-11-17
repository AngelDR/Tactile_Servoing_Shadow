

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

  // publishers error
  ros::Publisher error_ff_pub = node_handle.advertise<std_msgs::Float64>("/error/ff", 1000);
  ros::Publisher error_mf_pub = node_handle.advertise<std_msgs::Float64>("/error/mf", 1000);
  ros::Publisher error_rf_pub = node_handle.advertise<std_msgs::Float64>("/error/rf", 1000);
  ros::Publisher error_lf_pub = node_handle.advertise<std_msgs::Float64>("/error/lf", 1000);
  ros::Publisher error_th_pub = node_handle.advertise<std_msgs::Float64>("/error/th", 1000);



   // >> PUBLISHERS PARA CONTROLADORES DE POSICION
  ros::Publisher pos_ff_j0_pub = node_handle.advertise<std_msgs::Float64>("/sh_ffj0_position_controller/command", 1000);
  ros::Publisher pos_ff_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_ffj3_position_controller/command", 1000);
  ros::Publisher pos_ff_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_ffj4_position_controller/command", 1000);
  
  ros::Publisher pos_mf_j0_pub = node_handle.advertise<std_msgs::Float64>("/sh_mfj0_position_controller/command", 1000);
  ros::Publisher pos_mf_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_mfj3_position_controller/command", 1000);
  ros::Publisher pos_mf_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_mfj4_position_controller/command", 1000);
  
  ros::Publisher pos_rf_j0_pub = node_handle.advertise<std_msgs::Float64>("/sh_rfj0_position_controller/command", 1000);
  ros::Publisher pos_rf_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_rfj3_position_controller/command", 1000);
  ros::Publisher pos_rf_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_rfj4_position_controller/command", 1000);
  
  ros::Publisher pos_lf_j0_pub = node_handle.advertise<std_msgs::Float64>("/sh_lfj0_position_controller/command", 1000);
  ros::Publisher pos_lf_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_lfj3_position_controller/command", 1000);
  ros::Publisher pos_lf_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_lfj4_position_controller/command", 1000);
  ros::Publisher pos_lf_j5_pub = node_handle.advertise<std_msgs::Float64>("/sh_lfj5_position_controller/command", 1000);

  ros::Publisher pos_th_j1_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj1_position_controller/command", 1000);
  ros::Publisher pos_th_j2_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj2_position_controller/command", 1000);
  ros::Publisher pos_th_j3_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj3_position_controller/command", 1000);
  ros::Publisher pos_th_j4_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj4_position_controller/command", 1000);
  ros::Publisher pos_th_j5_pub = node_handle.advertise<std_msgs::Float64>("/sh_thj5_position_controller/command", 1000);
  
  ros::Publisher pos_wr_j1_pub = node_handle.advertise<std_msgs::Float64>("/sh_wrj1_position_controller/command", 1000);
  ros::Publisher pos_wr_j2_pub = node_handle.advertise<std_msgs::Float64>("/sh_wrj2_position_controller/command", 1000);

  // Cargar modelo (MoveIt)
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  ROS_INFO("Moveit Model : ok");
  // Definir kinematic state
  robot_state::RobotStatePtr kinematic_state(new robot_state::RobotState(kinematic_model));
  ROS_INFO("Kinematic State : ok");
  kinematic_state->setToDefaultValues();
  ROS_INFO("Joint model to default values : ok");
  const robot_state::JointModelGroup* joint_model_ff;
  const robot_state::JointModelGroup* joint_model_mf;
  const robot_state::JointModelGroup* joint_model_rf;
  const robot_state::JointModelGroup* joint_model_lf;
  const robot_state::JointModelGroup* joint_model_th;
  std::string tip_name_ff,tip_name_mf,tip_name_rf,tip_name_lf,tip_name_th; 
  joint_model_ff = kinematic_model->getJointModelGroup("first_finger");
  tip_name_ff = "fftip";
  joint_model_mf = kinematic_model->getJointModelGroup("middle_finger");
  tip_name_mf = "mftip";
  joint_model_rf = kinematic_model->getJointModelGroup("ring_finger");
  tip_name_rf = "rftip";
  joint_model_lf = kinematic_model->getJointModelGroup("little_finger");
  tip_name_lf = "lftip";
  joint_model_th = kinematic_model->getJointModelGroup("thumb");
  tip_name_th = "thtip";
  ROS_INFO("Joint models: ok");


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
  double position_list[24] = {0.0, 0.0, -0.05, 0.3, 0.3, 0.3, -0.05, 0.3, 0.3, 0.3, -0.05, 0.3, 0.3, 0.3, 0.0, -0.05, 0.3, 0.3, 0.3, 0.8, 0.8, 0.15, -0.69, 0.26};
  double velocity_list[24] = {0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  for(int i = 0; i < 24; i++ ){
    fake_states.name.push_back(name_list[i]);
    fake_states.position.push_back(position_list[i]);
    fake_states.velocity.push_back(velocity_list[i]);
  }
  ROS_INFO("fake_states positions %i", fake_states.position.size());

  // >> posiciones iniciales
  /**pos_ff_j4_pub.publish(-0.05);
  sleep(0.1);
  pos_ff_j3_pub.publish(0.3);
  sleep(0.1);
  pos_ff_j0_pub.publish(0.3);
  sleep(0.1);
  pos_mf_j4_pub.publish(-0.05);
  sleep(0.1);
  pos_mf_j3_pub.publish(0.3);
  sleep(0.1);
  pos_mf_j0_pub.publish(0.3);
  sleep(0.1);
  pos_rf_j4_pub.publish(-0.05);
  sleep(0.1);
  pos_rf_j3_pub.publish(0.3);
  sleep(0.1);
  pos_rf_j0_pub.publish(0.3);
  sleep(0.1);
  pos_lf_j5_pub.publish(0.0);
  sleep(0.1);
  pos_lf_j4_pub.publish(0.0);
  sleep(0.1);
  pos_lf_j3_pub.publish(0.3);
  sleep(0.1);
  pos_lf_j0_pub.publish(0.3);
  sleep(0.1);
  pos_th_j5_pub.publish(0.802);
  sleep(0.1);
  pos_th_j4_pub.publish(1.22);
  sleep(0.1);
  pos_th_j3_pub.publish(0.157);
  sleep(0.1);
  pos_th_j2_pub.publish(-0.698);
  sleep(0.1);
  pos_th_j1_pub.publish(0.261);
  sleep(0.1);*/



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
    
  // >> PARAMETROS DE LOS EXPERIMENTOS
  // >> Obtener numero de dedos que se usan en el experimento -> rosparam
  int num_fingers_exp;
  if (node_handle.getParam("/grasp_reconfiguration/number_fingers", num_fingers_exp))
  {
    ROS_INFO("Numero de dedos para el experimento : %d", num_fingers_exp); 
  }
  else{
      num_fingers_exp = 4;
  }

  /** Iniciar PID */
  /** (p,i,d,i_max, d_max)*/
  pid_controller.initPid(0.5,0.3,0.3,0.0,0.0);
  
  //double value_position = 0.8;
  //vel_ff_j3_pub.publish(value_position);

  double f_desired_ff = 0.3;   // newtons
  double f_desired_mf = 0.3;   // newtons
  double f_desired_rf = 0.3;   // newtons
  double f_desired_lf = 0.3;   // newtons
  double f_desired_th = 0.7;   // newtons
  double f_t_current = 0.0; 
  double f_t_previous= 0.0;

  bool stable = false;
  bool stabilized = false;

  ros::Time last_time = ros::Time::now();
  
  error_matrix(0,0) = 0.0;
  error_matrix(1,0) = 0.0;
  error_matrix(2,0) = 0.0;
  error_matrix(3,0) = 0.0;
  ROS_INFO("Parameters : ok");

  // pruebas
  double var_pos = 0.1;  


  // Archivos Log - plot
  ofstream vel_file, err_file, forces_file, desired_forces_file;
  vel_file.open ("/home/aurova/Desktop/pruebas/resultados/tactile_servo/velocities.txt");
  if(vel_file.is_open())
    ROS_INFO("Archivo de velocidades abierto");

  err_file.open ("/home/aurova/Desktop/pruebas/resultados/tactile_servo/errors.txt");
  if(err_file.is_open())
    ROS_INFO("Archivo de errores abierto");

  forces_file.open ("/home/aurova/Desktop/pruebas/resultados/tactile_servo/forces.txt");
  if(forces_file.is_open())
    ROS_INFO("Archivo de fuerzas abierto");

  desired_forces_file.open ("/home/aurova/Desktop/pruebas/resultados/tactile_servo/forces_des.txt");
  if(desired_forces_file.is_open())
    ROS_INFO("Archivo de fuerzas deseadas abierto");

  // Inicio bucle Tactle servoing
  int iteration = 0;
  do
  {   
    if (node_handle.getParam("/tactile_servo_reconfiguration/ff_desired_f", f_desired_ff))
    {
      ROS_INFO("F deseada ff : %f", f_desired_ff); 
    }

    if (node_handle.getParam("/tactile_servo_reconfiguration/mf_desired_f", f_desired_mf))
    {
      ROS_INFO("F deseada mf : %f", f_desired_mf); 
    }

    if (node_handle.getParam("/tactile_servo_reconfiguration/rf_desired_f", f_desired_rf))
    {
      ROS_INFO("F deseada rf : %f", f_desired_rf); 
    }

    if (node_handle.getParam("/tactile_servo_reconfiguration/lf_desired_f", f_desired_lf))
    {
      ROS_INFO("F deseada lf : %f", f_desired_lf); 
    }

    if (node_handle.getParam("/tactile_servo_reconfiguration/th_desired_f", f_desired_th))
    {
      ROS_INFO("F deseada th : %f", f_desired_th); 
    }

    
      fake_states.header.stamp = ros::Time::now();
      fake_states.header.frame_id = "fake_states";
      fake_controller.publish(fake_states); 

      if (pressure_client.call(srv_pressure))
      {

        /**
        * Escribir results
        */
        forces_file << srv_pressure.response.applied_force[0] << " " << srv_pressure.response.applied_force[1] << " " << srv_pressure.response.applied_force[2] << " " << srv_pressure.response.applied_force[3] << " " << srv_pressure.response.applied_force[4] << " " << iteration << "\n";
        desired_forces_file << f_desired_th << " " << f_desired_ff << " " << f_desired_mf << " " << f_desired_rf << " " << f_desired_lf << " " << iteration << "\n";

        /*** FIRST FINGER
        *
        */
        f_t_current = srv_pressure.response.applied_force[1]; 
        double error_f = f_desired_ff - f_t_current;
        if((-0.1 < error_f) &&  (error_f < 0.1)){stable = true;}
        error_matrix(1,0) = error_f;
        error_ff_pub.publish(error_f);
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
        // error en X e Y depende de las medidas del sensor. 17mm x 17mm
        // distancia entre cada celdilla -> x = 4mm; y 4mm
        double error_x = (max_position_i - 1) * 0.004;
        double error_y = (max_position_j - 2) * 0.004;
        error_matrix(0,0) = - error_x;
        error_matrix(2,0) = - error_y;

        //ROS_INFO("/Servoing: Centroide presión x = %i, y = %i", max_position_i, max_position_j);
        //ROS_INFO("/Servoing: Error x = %f, y = %f, f = %f, o = %f ", error_matrix(0,0), error_matrix(1,0), error_matrix(2,0), error_matrix(3,0));
        // Aplicar PID a errores
        ros::Time current_time = ros::Time::now();
        /**
        * guardar results. 
        */

        err_file << error_matrix(0,0) << " " << error_matrix(1,0) << " " << error_matrix(2,0) << " ";

        error_matrix(0,0) = pid_controller.updatePid(error_matrix(0,0), current_time - last_time);
        error_matrix(1,0) = pid_controller.updatePid(error_matrix(1,0), current_time - last_time);
        error_matrix(2,0) = pid_controller.updatePid(error_matrix(2,0), current_time - last_time);
        error_matrix(3,0) = pid_controller.updatePid(3.0, current_time - last_time);
        last_time = current_time;
        // Sensor velocity
        sensor_velocity = projection_matrix*inverse_jacobian*error_matrix; 
        //ROS_INFO("Force error = %f", error_f); 
        //ROS_INFO("/Servoing: Error * [pid] x = %f, y = %f, f = %f, o = %f ", error_matrix(0,0), error_matrix(1,0), error_matrix(2,0), error_matrix(3,0));
        ROS_INFO("/Servoing: velocity (sens)  x = %f, y = %f, z = %f, rotx = %f, roty = %f, rotz = %f ", sensor_velocity(0,0), sensor_velocity(1,0), sensor_velocity(2,0), sensor_velocity(3,0), sensor_velocity(4,0), sensor_velocity(5,0));
        vel_file << sensor_velocity(0,0) << " " << sensor_velocity(1,0) << " " << sensor_velocity(2,0) << " ";
        // Velocity tip = vel. sensor ??  
        // Transformar velocity sensor -> frame Sensor -> frame tip
        //tfBuffer.waitForTransform("palm","sensor_ff");
        try{
          ros::Time now = ros::Time::now();
          tf::StampedTransform transform_s;
          tfListener.waitForTransform("palm", "ffsensor", now, ros::Duration(8.0));
          tfListener.lookupTransform("palm", "ffsensor", now, transform_s);
          //ROS_INFO("Transformation  x = %f, y = %f, z = %f)", transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
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
          //mult_matrix = vector_matrix * mult_matrix;
          adjoint_matrix <<  transformation_matrix(0,0), transformation_matrix(0,1), transformation_matrix(0,2), mult_matrix(0,0), mult_matrix(0,1), mult_matrix(0,2),
                                      transformation_matrix(1,0), transformation_matrix(1,1), transformation_matrix(1,2), mult_matrix(1,0), mult_matrix(1,1), mult_matrix(1,2),
                                      transformation_matrix(2,0), transformation_matrix(2,1), transformation_matrix(2,2), mult_matrix(2,0), mult_matrix(2,1), mult_matrix(2,2),
                                      0,0,0,transformation_matrix(0,0), transformation_matrix(0,1), transformation_matrix(0,2),
                                      0,0,0,transformation_matrix(1,0), transformation_matrix(1,1), transformation_matrix(1,2),
                                      0,0,0,transformation_matrix(2,0), transformation_matrix(2,1), transformation_matrix(2,2);
          //ROS_INFO_STREAM("/Adjoint matrix: \n" << adjoint_matrix);
          // Vel sensor -> frame palm ->
          sensor_velocity = adjoint_matrix * sensor_velocity;
          ROS_INFO("/Servoing: velocity (palm):  x = %f, y = %f, z = %f, rotx = %f, roty = %f, rotz = %f ", sensor_velocity(0,0), sensor_velocity(1,0), sensor_velocity(2,0), sensor_velocity(3,0), sensor_velocity(4,0), sensor_velocity(5,0));   
          /*****************************************************************************
              Prueba con IK -> enviar posicion 
              interpolar posicion para tiempo = 0.5 seg. A partir de velocidad del sensor
          **********************************************************************************/
          try{
            double time_int = 1;
            // desplazamiento respecto a /palm
            sensor_displacement <<  sensor_velocity(0,0) * time_int, sensor_velocity(1,0) * time_int,
                                    sensor_velocity(2,0) * time_int, sensor_velocity(3,0) * time_int,
                                    sensor_velocity(4,0) * time_int, sensor_velocity(5,0) * time_int;
            ROS_INFO_STREAM("Displacement " << sensor_displacement);
            ros::Time now = ros::Time::now();
            tf::StampedTransform transform_sensor_palm;
            tfListener.waitForTransform("forearm", "fftip", now, ros::Duration(3.0));
            tfListener.lookupTransform("forearm", "fftip", now, transform_sensor_palm);

            ROS_INFO("Posicion original: x=%f, y=%f, z=%f ", transform_sensor_palm.getOrigin().getX(),transform_sensor_palm.getOrigin().getY(),transform_sensor_palm.getOrigin().getZ());

            // Prueba IK
            const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("fftip");
            ROS_INFO(" \n End effector state \n");
            ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation());
            // pruab

            // obtener transformacion /palm a /forearm . 
            tf::Vector3 new_origin;
            new_origin.setX((transform_sensor_palm.getOrigin().getX()) + sensor_displacement(0,0));
            new_origin.setY((transform_sensor_palm.getOrigin().getY()) + sensor_displacement(1,0));
            new_origin.setZ((transform_sensor_palm.getOrigin().getZ()) + sensor_displacement(2,0));
            transform_sensor_palm.setOrigin(new_origin);
            ROS_INFO("Posicion final: x=%f, y=%f, z=%f ",new_origin.getX(),new_origin.getY(),new_origin.getZ());

            //const Eigen::Affine3d &end_effector_state;
            geometry_msgs::Pose sensor_position;
            sensor_position.position.x = new_origin.getX();
            sensor_position.position.y = new_origin.getY();
            sensor_position.position.z = new_origin.getZ();

            sensor_position.orientation.x = transform_sensor_palm.getRotation().getAxis().getX();
            sensor_position.orientation.y = transform_sensor_palm.getRotation().getAxis().getY();
            sensor_position.orientation.z = transform_sensor_palm.getRotation().getAxis().getZ();
            sensor_position.orientation.w = transform_sensor_palm.getRotation().getW();
            
            std::vector<double> joint_values;
            const std::vector<std::string> &joint_names = joint_model_ff->getJointModelNames();
            //ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation());
            //ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation());

            /**bool found_ik = kinematic_state->setFromIK(joint_model_ff, sensor_position, 2, 0.1);
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
            }*/
            /**if(error_f < 0.0){
                fake_states.position.at(3) = fake_states.position.at(3) - (error_f * 0.05);
                fake_states.position.at(4) = fake_states.position.at(4) - (error_f * 0.05);
                //fake_states.position.at(5) = fake_states.position.at(5) + 0.03;
            }
            else{
                fake_states.position.at(3) = fake_states.position.at(3) + (error_f * 0.05);
                fake_states.position.at(4) = fake_states.position.at(4) + (error_f * 0.05);
                //fake_states.position.at(5) = fake_states.position.at(5) - 0.03;
            } */

            fake_states.position.at(3) = fake_states.position.at(3) + (error_f * 0.5);
            fake_states.position.at(4) = fake_states.position.at(4) + (error_f * 0.2);


            pos_ff_j3_pub.publish(fake_states.position.at(3));
            sleep(0.1); 
            pos_ff_j0_pub.publish(fake_states.position.at(4));
            sleep(0.1);  

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
        /**
        * --- FIRST FINGER
        */




        /*** MIDDLE FINGER
        *
        *
        */
        f_t_current = srv_pressure.response.applied_force[2]; 
        error_f = f_desired_mf - f_t_current;
        if((-0.1 < error_f) &&  (error_f < 0.1)){stable = true;}
        error_matrix(1,0) = error_f;
        error_mf_pub.publish(error_f);
        // 2  - get pos(x,y) error
        // pos(x,y) desired = [1,2]
        max_value = srv_pressure.response.mf_values[0];
        iter = 0;
        max_position_i,max_position_j = 0;
        for(int i = 0;i < 4; i++)
        {
          for(int j = 0; j < 4; j++)
          {
            sensor_values(i,j) = srv_pressure.response.mf_values[iter]; 
            if (srv_pressure.response.mf_values[iter] > max_value)
            {
              max_position_i = i;
              max_position_j = j;
              max_value = srv_pressure.response.mf_values[iter];
            }
            iter++;
          }
        }
        // error en X e Y depende de las medidas del sensor. 17mm x 17mm
        // distancia entre cada celdilla -> x = 4mm; y 4mm
        error_x = (max_position_i - 1) * 0.004;
        error_y = (max_position_j - 2) * 0.004;
        error_matrix(0,0) = - error_x;
        error_matrix(2,0) = - error_y;
        //ROS_INFO("/Servoing: Centroide presión x = %i, y = %i", max_position_i, max_position_j);
        //ROS_INFO("/Servoing: Error x = %f, y = %f, f = %f, o = %f ", error_matrix(0,0), error_matrix(1,0), error_matrix(2,0), error_matrix(3,0));
        // Aplicar PID a errores
        current_time = ros::Time::now();

        /**
        * guardar results. 
        */
        err_file << error_matrix(0,0) << " " << error_matrix(1,0) << " " << error_matrix(2,0) << " ";

        error_matrix(0,0) = pid_controller.updatePid(error_matrix(0,0), current_time - last_time);
        error_matrix(1,0) = pid_controller.updatePid(error_matrix(1,0), current_time - last_time);
        error_matrix(2,0) = pid_controller.updatePid(error_matrix(2,0), current_time - last_time);
        error_matrix(3,0) = pid_controller.updatePid(3.0, current_time - last_time);
        last_time = current_time;
        // Sensor velocity
        sensor_velocity = projection_matrix*inverse_jacobian*error_matrix; 
        //ROS_INFO("Force error = %f", error_f); 
        //ROS_INFO("/Servoing: Error * [pid] x = %f, y = %f, f = %f, o = %f ", error_matrix(0,0), error_matrix(1,0), error_matrix(2,0), error_matrix(3,0));
        ROS_INFO("/Servoing: velocity (sens)  x = %f, y = %f, z = %f, rotx = %f, roty = %f, rotz = %f ", sensor_velocity(0,0), sensor_velocity(1,0), sensor_velocity(2,0), sensor_velocity(3,0), sensor_velocity(4,0), sensor_velocity(5,0));
        vel_file << sensor_velocity(0,0) << " " << sensor_velocity(1,0) << " " << sensor_velocity(2,0) << " ";
        // Velocity tip = vel. sensor ??  
        // Transformar velocity sensor -> frame Sensor -> frame tip
        //tfBuffer.waitForTransform("palm","sensor_ff");
        try{
          ros::Time now = ros::Time::now();
          tf::StampedTransform transform_s;
          tfListener.waitForTransform("palm", "mfsensor", now, ros::Duration(3.0));
          tfListener.lookupTransform("palm", "mfsensor", now, transform_s);
          //ROS_INFO("Transformation  x = %f, y = %f, z = %f)", transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
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
          //mult_matrix = vector_matrix * mult_matrix;
          adjoint_matrix <<  transformation_matrix(0,0), transformation_matrix(0,1), transformation_matrix(0,2), mult_matrix(0,0), mult_matrix(0,1), mult_matrix(0,2),
                                      transformation_matrix(1,0), transformation_matrix(1,1), transformation_matrix(1,2), mult_matrix(1,0), mult_matrix(1,1), mult_matrix(1,2),
                                      transformation_matrix(2,0), transformation_matrix(2,1), transformation_matrix(2,2), mult_matrix(2,0), mult_matrix(2,1), mult_matrix(2,2),
                                      0,0,0,transformation_matrix(0,0), transformation_matrix(0,1), transformation_matrix(0,2),
                                      0,0,0,transformation_matrix(1,0), transformation_matrix(1,1), transformation_matrix(1,2),
                                      0,0,0,transformation_matrix(2,0), transformation_matrix(2,1), transformation_matrix(2,2);
          //ROS_INFO_STREAM("/Adjoint matrix: \n" << adjoint_matrix);

          // Vel sensor -> frame palm ->
          sensor_velocity = adjoint_matrix * sensor_velocity;
          ROS_INFO("/Servoing: velocity (palm):  x = %f, y = %f, z = %f, rotx = %f, roty = %f, rotz = %f ", sensor_velocity(0,0), sensor_velocity(1,0), sensor_velocity(2,0), sensor_velocity(3,0), sensor_velocity(4,0), sensor_velocity(5,0));

          /*****************************************************************************
              Prueba con IK -> enviar posicion 
              interpolar posicion para tiempo = 0.5 seg. A partir de velocidad del sensor
          **********************************************************************************/
          try{
            double time_int = 1;
            // desplazamiento respecto a /palm
            sensor_displacement <<  sensor_velocity(0,0) * time_int, sensor_velocity(1,0) * time_int,
                                    sensor_velocity(2,0) * time_int, sensor_velocity(3,0) * time_int,
                                    sensor_velocity(4,0) * time_int, sensor_velocity(5,0) * time_int;
            ROS_INFO_STREAM("Displacement " << sensor_displacement);
            ros::Time now = ros::Time::now();
            tf::StampedTransform transform_sensor_palm;
            tfListener.waitForTransform("forearm", "mftip", now, ros::Duration(3.0));
            tfListener.lookupTransform("forearm", "mftip", now, transform_sensor_palm);

            ROS_INFO("Posicion original: x=%f, y=%f, z=%f ", transform_sensor_palm.getOrigin().getX(),transform_sensor_palm.getOrigin().getY(),transform_sensor_palm.getOrigin().getZ());


            // Prueba IK
            const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("mftip");
            ROS_INFO(" \n End effector state \n");
            ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation());
            // pruab

            // obtener transformacion /palm a /forearm . 

            tf::Vector3 new_origin;
            new_origin.setX((transform_sensor_palm.getOrigin().getX()) + sensor_displacement(0,0));
            new_origin.setY((transform_sensor_palm.getOrigin().getY()) + sensor_displacement(1,0));
            new_origin.setZ((transform_sensor_palm.getOrigin().getZ()) + sensor_displacement(2,0));
            transform_sensor_palm.setOrigin(new_origin);
            ROS_INFO("Posicion final: x=%f, y=%f, z=%f ",new_origin.getX(),new_origin.getY(),new_origin.getZ());

            //const Eigen::Affine3d &end_effector_state;
            geometry_msgs::Pose sensor_position;
            sensor_position.position.x = new_origin.getX();
            sensor_position.position.y = new_origin.getY();
            sensor_position.position.z = new_origin.getZ();

            sensor_position.orientation.x = transform_sensor_palm.getRotation().getAxis().getX();
            sensor_position.orientation.y = transform_sensor_palm.getRotation().getAxis().getY();
            sensor_position.orientation.z = transform_sensor_palm.getRotation().getAxis().getZ();
            sensor_position.orientation.w = transform_sensor_palm.getRotation().getW();
            
            std::vector<double> joint_values;
            const std::vector<std::string> &joint_names = joint_model_mf->getJointModelNames();
            //ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation());
            //ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation());

            /**bool found_ik = kinematic_state->setFromIK(joint_model_mf, sensor_position, 2, 0.1);
            if (found_ik)
            {
              kinematic_state->copyJointGroupPositions(joint_model_mf, joint_values);
              for(std::size_t i=0; i < joint_names.size(); ++i)
              {   
                fake_states.position.at(i+6) = joint_values[i];
                ROS_INFO("IK para Joint %s: %f", joint_names[i].c_str(), joint_values[i]);  
              }
              pos_mf_j3_pub.publish(joint_values[0]);
              sleep(0.1);
              pos_mf_j0_pub.publish(joint_values[2]);
              sleep(0.1);
            }
            else
            {
              ROS_INFO("Did not find IK solution");
              ROS_INFO("Did not find IK solution -> joint approximation");
              if(error_f > 0.0){
                fake_states.position.at(7) = fake_states.position.at(7) + 0.05;
                //fake_states.position.at(8) = fake_states.position.at(8) + 0.03;
                //fake_states.position.at(9) = fake_states.position.at(9) + 0.03;
              }
              else{
                fake_states.position.at(7) = fake_states.position.at(7) - 0.05;
                //fake_states.position.at(8) = fake_states.position.at(8) - 0.03;
                //fake_states.position.at(9) = fake_states.position.at(9) - 0.03;
              }
              pos_mf_j3_pub.publish(fake_states.position.at(7));
              sleep(0.1);
              //pos_mf_j0_pub.publish(fake_states.position.at(8));
              //sleep(0.1);
            }*/

            /**if(error_f < 0.0){
                fake_states.position.at(7) = fake_states.position.at(7) - (error_f * 0.05);
                fake_states.position.at(8) = fake_states.position.at(8) - (error_f * 0.05);
                //fake_states.position.at(5) = fake_states.position.at(5) + 0.03;
            }
            else{
                fake_states.position.at(7) = fake_states.position.at(7) + (error_f * 0.05);
                fake_states.position.at(8) = fake_states.position.at(8) + (error_f * 0.05);
                //fake_states.position.at(5) = fake_states.position.at(5) - 0.03;
            } */

            fake_states.position.at(7) = fake_states.position.at(7) + (error_f * 0.5);
            fake_states.position.at(8) = fake_states.position.at(8) + (error_f * 0.2);


            pos_mf_j3_pub.publish(fake_states.position.at(7));
            sleep(0.1);
            pos_mf_j0_pub.publish(fake_states.position.at(8));
            sleep(0.1);  

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
          
        /**
        * --- MIDDLE FINGER
        */




        /*** RING FINGER
        *
        *
        *
        */
        f_t_current = srv_pressure.response.applied_force[3]; 
        error_f = f_desired_rf - f_t_current;
        //if(-0.05 < error_f < 0.05) {stable = true;}
        error_matrix(1,0) = error_f;
        error_rf_pub.publish(error_f);
        // 2  - get pos(x,y) error
        // pos(x,y) desired = [1,2]
        max_value = srv_pressure.response.rf_values[0];
        iter = 0;
        max_position_i,max_position_j = 0;
        for(int i = 0;i < 4; i++)
        {
          for(int j = 0; j < 4; j++)
          {
            sensor_values(i,j) = srv_pressure.response.rf_values[iter]; 
            if (srv_pressure.response.rf_values[iter] > max_value)
            {
              max_position_i = i;
              max_position_j = j;
              max_value = srv_pressure.response.rf_values[iter];
            }
            iter++;
          }
        }
        // error en X e Y depende de las medidas del sensor. 17mm x 17mm
        // distancia entre cada celdilla -> x = 4mm; y 4mm
        error_x = (max_position_i - 1) * 0.004;
        error_y = (max_position_j - 2) * 0.004;
        error_matrix(0,0) = - error_x;
        error_matrix(2,0) = - error_y;
        //ROS_INFO("/Servoing: Centroide presión x = %i, y = %i", max_position_i, max_position_j);
        //ROS_INFO("/Servoing: Error x = %f, y = %f, f = %f, o = %f ", error_matrix(0,0), error_matrix(1,0), error_matrix(2,0), error_matrix(3,0));

        // Aplicar PID a errores
        current_time = ros::Time::now();
        /**
        * guardar results. 
        */
        err_file << error_matrix(0,0) << " " << error_matrix(1,0) << " " << error_matrix(2,0) << " ";

        error_matrix(0,0) = pid_controller.updatePid(error_matrix(0,0), current_time - last_time);
        error_matrix(1,0) = pid_controller.updatePid(error_matrix(1,0), current_time - last_time);
        error_matrix(2,0) = pid_controller.updatePid(error_matrix(2,0), current_time - last_time);
        error_matrix(3,0) = pid_controller.updatePid(3.0, current_time - last_time);
        last_time = current_time;

        // Sensor velocity
        sensor_velocity = projection_matrix*inverse_jacobian*error_matrix; 
        //ROS_INFO("Force error = %f", error_f); 
        //ROS_INFO("/Servoing: Error * [pid] x = %f, y = %f, f = %f, o = %f ", error_matrix(0,0), error_matrix(1,0), error_matrix(2,0), error_matrix(3,0));
        ROS_INFO("/Servoing: velocity (sens)  x = %f, y = %f, z = %f, rotx = %f, roty = %f, rotz = %f ", sensor_velocity(0,0), sensor_velocity(1,0), sensor_velocity(2,0), sensor_velocity(3,0), sensor_velocity(4,0), sensor_velocity(5,0));
        vel_file << sensor_velocity(0,0) << " " << sensor_velocity(1,0) << " " << sensor_velocity(2,0) << " ";

        // Velocity tip = vel. sensor ??  
        // Transformar velocity sensor -> frame Sensor -> frame tip
        //tfBuffer.waitForTransform("palm","sensor_ff");
        try{
          ros::Time now = ros::Time::now();
          tf::StampedTransform transform_s;
          tfListener.waitForTransform("palm", "rfsensor", now, ros::Duration(3.0));
          tfListener.lookupTransform("palm", "rfsensor", now, transform_s);
          //ROS_INFO("Transformation  x = %f, y = %f, z = %f)", transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
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
          //mult_matrix = vector_matrix * mult_matrix;
          adjoint_matrix <<  transformation_matrix(0,0), transformation_matrix(0,1), transformation_matrix(0,2), mult_matrix(0,0), mult_matrix(0,1), mult_matrix(0,2),
                                      transformation_matrix(1,0), transformation_matrix(1,1), transformation_matrix(1,2), mult_matrix(1,0), mult_matrix(1,1), mult_matrix(1,2),
                                      transformation_matrix(2,0), transformation_matrix(2,1), transformation_matrix(2,2), mult_matrix(2,0), mult_matrix(2,1), mult_matrix(2,2),
                                      0,0,0,transformation_matrix(0,0), transformation_matrix(0,1), transformation_matrix(0,2),
                                      0,0,0,transformation_matrix(1,0), transformation_matrix(1,1), transformation_matrix(1,2),
                                      0,0,0,transformation_matrix(2,0), transformation_matrix(2,1), transformation_matrix(2,2);
          //ROS_INFO_STREAM("/Adjoint matrix: \n" << adjoint_matrix);

          // Vel sensor -> frame palm ->
          sensor_velocity = adjoint_matrix * sensor_velocity;
          ROS_INFO("/Servoing: velocity (palm):  x = %f, y = %f, z = %f, rotx = %f, roty = %f, rotz = %f ", sensor_velocity(0,0), sensor_velocity(1,0), sensor_velocity(2,0), sensor_velocity(3,0), sensor_velocity(4,0), sensor_velocity(5,0));
  
          /*****************************************************************************
              Prueba con IK -> enviar posicion 
              interpolar posicion para tiempo = 0.5 seg. A partir de velocidad del sensor
          **********************************************************************************/
          try{
            double time_int = 1;
            // desplazamiento respecto a /palm
            sensor_displacement <<  sensor_velocity(0,0) * time_int, sensor_velocity(1,0) * time_int,
                                    sensor_velocity(2,0) * time_int, sensor_velocity(3,0) * time_int,
                                    sensor_velocity(4,0) * time_int, sensor_velocity(5,0) * time_int;
            ROS_INFO_STREAM("Displacement " << sensor_displacement);
            ros::Time now = ros::Time::now();
            tf::StampedTransform transform_sensor_palm;
            tfListener.waitForTransform("forearm", "rftip", now, ros::Duration(3.0));
            tfListener.lookupTransform("forearm", "rftip", now, transform_sensor_palm);

            ROS_INFO("Posicion original: x=%f, y=%f, z=%f ", transform_sensor_palm.getOrigin().getX(),transform_sensor_palm.getOrigin().getY(),transform_sensor_palm.getOrigin().getZ());


            // Prueba IK
            const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("rftip");
            ROS_INFO(" \n End effector state \n");
            ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation());
            // pruab

            // obtener transformacion /palm a /forearm . 

            tf::Vector3 new_origin;
            new_origin.setX((transform_sensor_palm.getOrigin().getX()) + sensor_displacement(0,0));
            new_origin.setY((transform_sensor_palm.getOrigin().getY()) + sensor_displacement(1,0));
            new_origin.setZ((transform_sensor_palm.getOrigin().getZ()) + sensor_displacement(2,0));
            transform_sensor_palm.setOrigin(new_origin);
            ROS_INFO("Posicion final: x=%f, y=%f, z=%f ",new_origin.getX(),new_origin.getY(),new_origin.getZ());

            //const Eigen::Affine3d &end_effector_state;
            geometry_msgs::Pose sensor_position;
            sensor_position.position.x = new_origin.getX();
            sensor_position.position.y = new_origin.getY();
            sensor_position.position.z = new_origin.getZ();

            sensor_position.orientation.x = transform_sensor_palm.getRotation().getAxis().getX();
            sensor_position.orientation.y = transform_sensor_palm.getRotation().getAxis().getY();
            sensor_position.orientation.z = transform_sensor_palm.getRotation().getAxis().getZ();
            sensor_position.orientation.w = transform_sensor_palm.getRotation().getW();
            
            std::vector<double> joint_values;
            const std::vector<std::string> &joint_names = joint_model_mf->getJointModelNames();
            //ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation());
            //ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation());

            /**bool found_ik = kinematic_state->setFromIK(joint_model_rf, sensor_position, 2, 0.1);
            if (found_ik)
            {
              kinematic_state->copyJointGroupPositions(joint_model_rf, joint_values);
              for(std::size_t i=0; i < joint_names.size(); ++i)
              {   
                fake_states.position.at(i+10) = joint_values[i];
                ROS_INFO("IK para Joint %s: %f", joint_names[i].c_str(), joint_values[i]);  
              }
            }
            else
            {
              ROS_INFO("Did not find IK solution");
              ROS_INFO("Did not find IK solution -> joint approximation");
              if(error_f > 0.0){
                fake_states.position.at(11) = fake_states.position.at(11) + 0.05;
                //fake_states.position.at(12) = fake_states.position.at(12) + 0.05;
                //fake_states.position.at(13) = fake_states.position.at(13) + 0.05;
              }
              else{
                fake_states.position.at(11) = fake_states.position.at(11) - 0.05;
                //fake_states.position.at(12) = fake_states.position.at(12) - 0.05;
                //fake_states.position.at(13) = fake_states.position.at(13) - 0.05;
              }
            }*/

            fake_states.position.at(11) = fake_states.position.at(11) + (error_f * 0.5);
            fake_states.position.at(12) = fake_states.position.at(12) + (error_f * 0.2);
            pos_rf_j3_pub.publish(fake_states.position.at(11));
            sleep(0.1);
            pos_rf_j0_pub.publish(fake_states.position.at(12));
            sleep(0.1); 

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
        /**
        * --- RING FINGER
        */



        /*** LITTLE FINGER
        *
        *
        *
        */
        f_t_current = srv_pressure.response.applied_force[4]; 
        error_f = f_desired_lf - f_t_current;
        //if(-0.05 < error_f < 0.05) {stable = true;}
        error_matrix(1,0) = error_f;
        error_lf_pub.publish(error_f);
        // 2  - get pos(x,y) error
        // pos(x,y) desired = [1,2]
        max_value = srv_pressure.response.lf_values[0];
        iter = 0;
        max_position_i,max_position_j = 0;
        for(int i = 0;i < 4; i++)
        {
          for(int j = 0; j < 4; j++)
          {
            sensor_values(i,j) = srv_pressure.response.lf_values[iter]; 
            if (srv_pressure.response.lf_values[iter] > max_value)
            {
              max_position_i = i;
              max_position_j = j;
              max_value = srv_pressure.response.lf_values[iter];
            }
            iter++;
          }
        }
        // error en X e Y depende de las medidas del sensor. 17mm x 17mm
        // distancia entre cada celdilla -> x = 4mm; y 4mm
        error_x = (max_position_i - 1) * 0.004;
        error_y = (max_position_j - 2) * 0.004;
        error_matrix(0,0) = - error_x;
        error_matrix(2,0) = - error_y;
        //ROS_INFO("/Servoing: Centroide presión x = %i, y = %i", max_position_i, max_position_j);
        //ROS_INFO("/Servoing: Error x = %f, y = %f, f = %f, o = %f ", error_matrix(0,0), error_matrix(1,0), error_matrix(2,0), error_matrix(3,0));

        // Aplicar PID a errores
        current_time = ros::Time::now();
        /**
        * guardar results. 
        */
        err_file << error_matrix(0,0) << " " << error_matrix(1,0) << " " << error_matrix(2,0) << " ";

        error_matrix(0,0) = pid_controller.updatePid(error_matrix(0,0), current_time - last_time);
        error_matrix(1,0) = pid_controller.updatePid(error_matrix(1,0), current_time - last_time);
        error_matrix(2,0) = pid_controller.updatePid(error_matrix(2,0), current_time - last_time);
        error_matrix(3,0) = pid_controller.updatePid(3.0, current_time - last_time);
        last_time = current_time;

        // Sensor velocity
        sensor_velocity = projection_matrix*inverse_jacobian*error_matrix; 
        //ROS_INFO("Force error = %f", error_f); 
        //ROS_INFO("/Servoing: Error * [pid] x = %f, y = %f, f = %f, o = %f ", error_matrix(0,0), error_matrix(1,0), error_matrix(2,0), error_matrix(3,0));
        ROS_INFO("/Servoing: velocity (sens)  x = %f, y = %f, z = %f, rotx = %f, roty = %f, rotz = %f ", sensor_velocity(0,0), sensor_velocity(1,0), sensor_velocity(2,0), sensor_velocity(3,0), sensor_velocity(4,0), sensor_velocity(5,0));
        vel_file << sensor_velocity(0,0) << " " << sensor_velocity(1,0) << " " << sensor_velocity(2,0) << " ";
        // Velocity tip = vel. sensor ??  
        // Transformar velocity sensor -> frame Sensor -> frame tip
        //tfBuffer.waitForTransform("palm","sensor_ff");
        try{
          ros::Time now = ros::Time::now();
          tf::StampedTransform transform_s;
          tfListener.waitForTransform("palm", "lfsensor", now, ros::Duration(3.0));
          tfListener.lookupTransform("palm", "lfsensor", now, transform_s);
          //ROS_INFO("Transformation  x = %f, y = %f, z = %f)", transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
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
          //mult_matrix = vector_matrix * mult_matrix;
          adjoint_matrix <<  transformation_matrix(0,0), transformation_matrix(0,1), transformation_matrix(0,2), mult_matrix(0,0), mult_matrix(0,1), mult_matrix(0,2),
                                      transformation_matrix(1,0), transformation_matrix(1,1), transformation_matrix(1,2), mult_matrix(1,0), mult_matrix(1,1), mult_matrix(1,2),
                                      transformation_matrix(2,0), transformation_matrix(2,1), transformation_matrix(2,2), mult_matrix(2,0), mult_matrix(2,1), mult_matrix(2,2),
                                      0,0,0,transformation_matrix(0,0), transformation_matrix(0,1), transformation_matrix(0,2),
                                      0,0,0,transformation_matrix(1,0), transformation_matrix(1,1), transformation_matrix(1,2),
                                      0,0,0,transformation_matrix(2,0), transformation_matrix(2,1), transformation_matrix(2,2);
          //ROS_INFO_STREAM("/Adjoint matrix: \n" << adjoint_matrix);

          // Vel sensor -> frame palm ->
          sensor_velocity = adjoint_matrix * sensor_velocity;
          ROS_INFO("/Servoing: velocity (palm):  x = %f, y = %f, z = %f, rotx = %f, roty = %f, rotz = %f ", sensor_velocity(0,0), sensor_velocity(1,0), sensor_velocity(2,0), sensor_velocity(3,0), sensor_velocity(4,0), sensor_velocity(5,0));

          /*****************************************************************************
              Prueba con IK -> enviar posicion 
              interpolar posicion para tiempo = 0.5 seg. A partir de velocidad del sensor
          **********************************************************************************/
          try{
            double time_int = 1;
            // desplazamiento respecto a /palm
            sensor_displacement <<  sensor_velocity(0,0) * time_int, sensor_velocity(1,0) * time_int,
                                    sensor_velocity(2,0) * time_int, sensor_velocity(3,0) * time_int,
                                    sensor_velocity(4,0) * time_int, sensor_velocity(5,0) * time_int;
            ROS_INFO_STREAM("Displacement " << sensor_displacement);
            ros::Time now = ros::Time::now();
            tf::StampedTransform transform_sensor_palm;
            tfListener.waitForTransform("forearm", "lftip", now, ros::Duration(3.0));
            tfListener.lookupTransform("forearm", "lftip", now, transform_sensor_palm);

            ROS_INFO("Posicion original: x=%f, y=%f, z=%f ", transform_sensor_palm.getOrigin().getX(),transform_sensor_palm.getOrigin().getY(),transform_sensor_palm.getOrigin().getZ());


            // Prueba IK
            const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("lftip");
            ROS_INFO(" \n End effector state \n");
            ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation());
            // pruab

            // obtener transformacion /palm a /forearm . 

            tf::Vector3 new_origin;
            new_origin.setX((transform_sensor_palm.getOrigin().getX()) + sensor_displacement(0,0));
            new_origin.setY((transform_sensor_palm.getOrigin().getY()) + sensor_displacement(1,0));
            new_origin.setZ((transform_sensor_palm.getOrigin().getZ()) + sensor_displacement(2,0));
            transform_sensor_palm.setOrigin(new_origin);
            ROS_INFO("Posicion final: x=%f, y=%f, z=%f ",new_origin.getX(),new_origin.getY(),new_origin.getZ());

            //const Eigen::Affine3d &end_effector_state;
            geometry_msgs::Pose sensor_position;
            sensor_position.position.x = new_origin.getX();
            sensor_position.position.y = new_origin.getY();
            sensor_position.position.z = new_origin.getZ();

            sensor_position.orientation.x = transform_sensor_palm.getRotation().getAxis().getX();
            sensor_position.orientation.y = transform_sensor_palm.getRotation().getAxis().getY();
            sensor_position.orientation.z = transform_sensor_palm.getRotation().getAxis().getZ();
            sensor_position.orientation.w = transform_sensor_palm.getRotation().getW();
            
            std::vector<double> joint_values;
            const std::vector<std::string> &joint_names = joint_model_lf->getJointModelNames();
            //ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation());
            //ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation());

            /**
            bool found_ik = kinematic_state->setFromIK(joint_model_lf, sensor_position, 2, 0.1);
            if (found_ik)
            {
              kinematic_state->copyJointGroupPositions(joint_model_lf, joint_values);
              for(std::size_t i=0; i < joint_names.size(); ++i)
              {   
                fake_states.position.at(i+14) = joint_values[i];
                ROS_INFO("IK para Joint %s: %f", joint_names[i].c_str(), joint_values[i]);  
              }
            }
            else
            {
              ROS_INFO("Did not find IK solution");
              ROS_INFO("Did not find IK solution -> joint approximation");
              if(error_f > 0.0){
                fake_states.position.at(16) = fake_states.position.at(16) + (error_f * 0.05);
                //fake_states.position.at(17) = fake_states.position.at(17) + 0.05;
                //fake_states.position.at(18) = fake_states.position.at(18) + 0.05;

              }
              else{
                fake_states.position.at(16) = fake_states.position.at(16) - (error_f * 0.05);
                //fake_states.position.at(17) = fake_states.position.at(17) - 0.05;
                //fake_states.position.at(18) = fake_states.position.at(18) - 0.05;
              }
            }*/
            fake_states.position.at(16) = fake_states.position.at(16) + (error_f * 0.5);
            fake_states.position.at(17) = fake_states.position.at(17) + (error_f * 0.2);
            pos_lf_j3_pub.publish(fake_states.position.at(16));
            sleep(0.1);
            pos_lf_j0_pub.publish(fake_states.position.at(17));
            sleep(0.1); 

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
        /**
        * --- LITTLE FINGER
        */



        /*** THUMB        *
        */
        f_t_current = srv_pressure.response.applied_force[0]; 
        error_f = f_desired_th - f_t_current;
        if((-0.1 < error_f) &&  (error_f < 0.1)){stable = true;}
        error_matrix(1,0) = error_f;
        error_th_pub.publish(error_f);
        // 2  - get pos(x,y) error
        // pos(x,y) desired = [1,2]
        max_value = srv_pressure.response.th_values[0];
        iter = 0;
        max_position_i,max_position_j = 0;
        for(int i = 0;i < 4; i++)
        {
          for(int j = 0; j < 4; j++)
          {
            sensor_values(i,j) = srv_pressure.response.th_values[iter]; 
            if (srv_pressure.response.th_values[iter] > max_value)
            {
              max_position_i = i;
              max_position_j = j;
              max_value = srv_pressure.response.th_values[iter];
            }
            iter++;
          }
        }
        // error en X e Y depende de las medidas del sensor. 17mm x 17mm
        // distancia entre cada celdilla -> x = 4mm; y 4mm
        error_x = (max_position_i - 1) * 0.004;
        error_y = (max_position_j - 2) * 0.004;
        error_matrix(0,0) = - error_x;
        error_matrix(2,0) = - error_y;
        //ROS_INFO("/Servoing: Centroide presión x = %i, y = %i", max_position_i, max_position_j);
        //ROS_INFO("/Servoing: Error x = %f, y = %f, f = %f, o = %f ", error_matrix(0,0), error_matrix(1,0), error_matrix(2,0), error_matrix(3,0));
        // Aplicar PID a errores
        current_time = ros::Time::now();
        /**
        * guardar results. 
        */
        err_file << error_matrix(0,0) << " " << error_matrix(1,0) << " " << error_matrix(2,0) << " " << iteration << "\n";

        error_matrix(0,0) = pid_controller.updatePid(error_matrix(0,0), current_time - last_time);
        error_matrix(1,0) = pid_controller.updatePid(error_matrix(1,0), current_time - last_time);
        error_matrix(2,0) = pid_controller.updatePid(error_matrix(2,0), current_time - last_time);
        error_matrix(3,0) = pid_controller.updatePid(3.0, current_time - last_time);
        last_time = current_time;
        // Sensor velocity
        sensor_velocity = projection_matrix*inverse_jacobian*error_matrix; 
        sensor_velocity(1,0) = sensor_velocity(1,0) * (-1);
        sensor_velocity(1,0) = sensor_velocity(2,0) * (-1); 
        //ROS_INFO("Force error = %f", error_f); 
        //ROS_INFO("/Servoing: Error * [pid] x = %f, y = %f, f = %f, o = %f ", error_matrix(0,0), error_matrix(1,0), error_matrix(2,0), error_matrix(3,0));
        ROS_INFO("/Servoing: velocity (sens)  x = %f, y = %f, z = %f, rotx = %f, roty = %f, rotz = %f ", sensor_velocity(0,0), sensor_velocity(1,0), sensor_velocity(2,0), sensor_velocity(3,0), sensor_velocity(4,0), sensor_velocity(5,0));
        vel_file << sensor_velocity(0,0) << " " << sensor_velocity(1,0) << " " << sensor_velocity(2,0) << " " << iteration << "\n";
        // Velocity tip = vel. sensor ??  
        // Transformar velocity sensor -> frame Sensor -> frame tip
        //tfBuffer.waitForTransform("palm","sensor_ff");
        try{
          ros::Time now = ros::Time::now();
          tf::StampedTransform transform_s;
          tfListener.waitForTransform("palm", "thsensor", now, ros::Duration(3.0));
          tfListener.lookupTransform("palm", "thsensor", now, transform_s);
          //ROS_INFO("Transformation  x = %f, y = %f, z = %f)", transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
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
          //mult_matrix = vector_matrix * mult_matrix;
          adjoint_matrix <<  transformation_matrix(0,0), transformation_matrix(0,1), transformation_matrix(0,2), mult_matrix(0,0), mult_matrix(0,1), mult_matrix(0,2),
                                      transformation_matrix(1,0), transformation_matrix(1,1), transformation_matrix(1,2), mult_matrix(1,0), mult_matrix(1,1), mult_matrix(1,2),
                                      transformation_matrix(2,0), transformation_matrix(2,1), transformation_matrix(2,2), mult_matrix(2,0), mult_matrix(2,1), mult_matrix(2,2),
                                      0,0,0,transformation_matrix(0,0), transformation_matrix(0,1), transformation_matrix(0,2),
                                      0,0,0,transformation_matrix(1,0), transformation_matrix(1,1), transformation_matrix(1,2),
                                      0,0,0,transformation_matrix(2,0), transformation_matrix(2,1), transformation_matrix(2,2);
          //ROS_INFO_STREAM("/Adjoint matrix: \n" << adjoint_matrix);

          // Vel sensor -> frame palm ->
          sensor_velocity = adjoint_matrix * sensor_velocity;
          ROS_INFO("/Servoing: velocity (palm):  x = %f, y = %f, z = %f, rotx = %f, roty = %f, rotz = %f ", sensor_velocity(0,0), sensor_velocity(1,0), sensor_velocity(2,0), sensor_velocity(3,0), sensor_velocity(4,0), sensor_velocity(5,0));


           
          /*****************************************************************************
              Prueba con IK -> enviar posicion 
              interpolar posicion para tiempo = 0.5 seg. A partir de velocidad del sensor
          **********************************************************************************/
          try{
            double time_int = 1;
            // desplazamiento respecto a /palm
            sensor_displacement <<  sensor_velocity(0,0) * time_int, sensor_velocity(1,0) * time_int,
                                    sensor_velocity(2,0) * time_int, sensor_velocity(3,0) * time_int,
                                    sensor_velocity(4,0) * time_int, sensor_velocity(5,0) * time_int;
            ROS_INFO_STREAM("Displacement " << sensor_displacement);
            ros::Time now = ros::Time::now();
            tf::StampedTransform transform_sensor_palm;
            tfListener.waitForTransform("forearm", "thtip", now, ros::Duration(3.0));
            tfListener.lookupTransform("forearm", "thtip", now, transform_sensor_palm);

            ROS_INFO("Posicion original: x=%f, y=%f, z=%f ", transform_sensor_palm.getOrigin().getX(),transform_sensor_palm.getOrigin().getY(),transform_sensor_palm.getOrigin().getZ());


            // Prueba IK
            const Eigen::Affine3d &end_effector_state = kinematic_state->getGlobalLinkTransform("thtip");
            ROS_INFO(" \n End effector state \n");
            ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation());
            // pruab

            // obtener transformacion /palm a /forearm . 

            tf::Vector3 new_origin;
            new_origin.setX((transform_sensor_palm.getOrigin().getX()) + sensor_displacement(0,0));
            new_origin.setY((transform_sensor_palm.getOrigin().getY()) + sensor_displacement(1,0));
            new_origin.setZ((transform_sensor_palm.getOrigin().getZ()) + sensor_displacement(2,0));
            transform_sensor_palm.setOrigin(new_origin);
            ROS_INFO("Posicion final: x=%f, y=%f, z=%f ",new_origin.getX(),new_origin.getY(),new_origin.getZ());

            //const Eigen::Affine3d &end_effector_state;
            geometry_msgs::Pose sensor_position;
            sensor_position.position.x = new_origin.getX();
            sensor_position.position.y = new_origin.getY();
            sensor_position.position.z = new_origin.getZ();

            sensor_position.orientation.x = transform_sensor_palm.getRotation().getAxis().getX();
            sensor_position.orientation.y = transform_sensor_palm.getRotation().getAxis().getY();
            sensor_position.orientation.z = transform_sensor_palm.getRotation().getAxis().getZ();
            sensor_position.orientation.w = transform_sensor_palm.getRotation().getW();
            
            std::vector<double> joint_values;
            const std::vector<std::string> &joint_names = joint_model_mf->getJointModelNames();
            //ROS_INFO_STREAM("Translation: \n" << end_effector_state.translation());
            //ROS_INFO_STREAM("Rotation: \n" << end_effector_state.rotation());

            /**bool found_ik = kinematic_state->setFromIK(joint_model_th, sensor_position, 2, 0.1);
            if (found_ik)
            {
              kinematic_state->copyJointGroupPositions(joint_model_th, joint_values);
              for(std::size_t i=0; i < joint_names.size(); ++i)
              {   
                fake_states.position.at(i+19) = joint_values[i];
                ROS_INFO("IK para Joint %s: %f", joint_names[i].c_str(), joint_values[i]);  
              }
            }
            else
            {
              ROS_INFO("Did not find IK solution");
              if(error_f > 0.0){
                //fake_states.position.at(21) = fake_states.position.at(21) + 0.05;
                //fake_states.position.at(22) = fake_states.position.at(22) + 0.05;
                fake_states.position.at(23) = fake_states.position.at(23) + 0.05;

              }
              else{
                //fake_states.position.at(21) = fake_states.position.at(21) - 0.05;
                //fake_states.position.at(22) = fake_states.position.at(22) - 0.05;
                fake_states.position.at(23) = fake_states.position.at(23) - 0.05;
              }
            }*/
            /*if(error_f < 0.0){
                fake_states.position.at(23) = fake_states.position.at(23) - (error_f * 0.05);
                //fake_states.position.at(4) = fake_states.position.at(4) + 0.03;
                //fake_states.position.at(5) = fake_states.position.at(5) + 0.03;
            }
            else{
                fake_states.position.at(23) = fake_states.position.at(23) + (error_f * 0.05);
                //fake_states.position.at(4) = fake_states.position.at(4) - 0.03;
                //fake_states.position.at(5) = fake_states.position.at(5) - 0.03;
            }*/

            fake_states.position.at(23) = fake_states.position.at(23) + (error_f * 0.2);
            fake_states.position.at(22) = fake_states.position.at(22) + (error_f * 0.5);

            pos_th_j1_pub.publish(fake_states.position.at(23));
            sleep(0.1);
            pos_th_j2_pub.publish(fake_states.position.at(22));
            sleep(0.1);  

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
          
        /**
        * --- THUMB FINGER
        */


        // >> PUBLISH STATES
        // FAKE STATES
        fake_states.header.stamp = ros::Time::now();
        fake_states.header.frame_id = "fake_states";
        fake_controller.publish(fake_states);
                                   
        //sleep(2.0);
        // Comportamiento task planner:
        ros::spinOnce();
        
        /**if(stable && !stabilized)
        {
          if(srv_pressure.response.applied_force[1] == 0){ f_desired_ff = 0.3;}
          else{f_desired_ff = srv_pressure.response.applied_force[1];}

          if(srv_pressure.response.applied_force[2] == 0){ f_desired_mf = 0.3;}
          else{f_desired_mf = srv_pressure.response.applied_force[2];}

          if(srv_pressure.response.applied_force[0] == 0){ f_desired_th = 0.3;}
          else{f_desired_th = srv_pressure.response.applied_force[0];} 

          //if(srv_pressure.response.applied_force[3] == 0){ f_desired_rf = 0.3;}
          //else{f_desired_rf = srv_pressure.response.applied_force[3];} 
          stabilized = true;
        }*/

      }
      else
      {
        ROS_ERROR("Failed to call service pressure");
        return 1;
      }
 
    iteration++;
  }while(ros::ok());

  err_file.close();
  vel_file.close();
  forces_file.close();
  desired_forces_file.close();
  
  ros::shutdown();  
  return 0;
}