

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
//#include "tactile_servoing_shadow/getFingerJacobianMatrix.h" 

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <Eigen/SVD> 
#include <control_toolbox/pid.h>
#include <math.h> 

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

#define NUM_TACTELS 19
#define NUM_NEIGHBORS 5
#define ZERO_VALUE 3300
#define IMAGE_RANK 255

class NodeClass {

private:

public:
  ros::NodeHandle n;
  ros::Subscriber sub;
  double virtual_image_height;
  double radio_cylinder;
  double pixel_size;
  double virtual_image_width;
  double zero_value;
  double pixels_vertical;
  double pixels_horizontal;
  double number_neighbors;
  MatrixXf tactel_cartesian_coordinates;
  MatrixXf tactel_cylindrical_coordinates;
  MatrixXf tactel_plane_coordinates;
  MatrixXf tactel_pixels_positions;
  MatrixXf virtual_image;
  MatrixXf mixture_gaussian_image;
  MatrixXf nearest_neighbors_positions;
  MatrixXf nearest_neighbors_values;
  MatrixXf deviations_array;




  //Constructor
  NodeClass()
  {
    // Establecer tamaño imagen tactil, correspondiente al semicilindro virtual
    virtual_image_height = 0.025;
    radio_cylinder = 0.007;
    pixel_size = 0.0025;
    virtual_image_width = 3.1415 * radio_cylinder;
    zero_value = 3300;

    // CARTESIAN COORDINATES
    // Matriz con posiciones de los electrodos:
    // TODO: cargar desde archivo
    tactel_cartesian_coordinates(NUM_TACTELS,6);
    tactel_cylindrical_coordinates = ArrayXXf::Zero(NUM_TACTELS, 3);
    tactel_plane_coordinates = ArrayXXf::Zero(NUM_TACTELS, 2);

    tactel_cartesian_coordinates << 
    0.993, -4.855, -1.116, 0.196, -0.956, -0.220,
    -2.700, -3.513, -3.670, 0.000, -0.692, -0.722,
    -6.200, -3.513, -3.670, 0.000, -0.692, -0.722,
    -8.000, -4.956, -1.116, 0.000, -0.976, -0.220,
    -10.500, -3.513, -3.670, 0.000, -0.692, -0.722,
    -13.400, -4.956, -1.116, 0.000, -0.976, -0.220,
    4.763, 0.000, -2.330, 0.500, 0.000, -0.866,
    3.031, -1.950, -3.330, 0.500, 0.000, -0.866,
    3.031, 1.950, -3.330, 0.500, 0.000, -0.866,
    1.299, 0.000, -4.330, 0.500, 0.000, -0.866,
    0.993, 4.855, -1.116, 0.196, 0.956, -0.220,
    -2.700, 3.513, -3.670, 0.000, 0.692, -0.722,
    -6.200, 3.513, -3.670, 0.000, 0.692, -0.722,
    -8.000, 4.956, -1.116, 0.000, 0.976, -0.220,
    -10.500, 3.513, -3.670, 0.000, 0.692, -0.722,
    -13.400, 4.956, -1.116, 0.000, 0.976, -0.220,
    -2.800, 0.000, -5.080, 0.000, 0.000, -1.000,
    -9.800, 0.000, -5.080, 0.000, 0.000, -1.000,
    -13.600, 0.000, -5.080, 0.000, 0.000, -1.000;

    // Escalar valores: mm -> m 
    tactel_cartesian_coordinates = tactel_cartesian_coordinates * 0.001;
    // Transladar posciones en x, para usar como referencia extremo del dedo;
    // trasladar origen a finger_tip x+0.005
    for(int i=0; i<NUM_TACTELS; i++){
      tactel_cartesian_coordinates(i,0) -= 0.005;
    }
    // Cambiar de signo coordenada z, para facilitar conversion a coord. cilindricas
    tactel_cartesian_coordinates.col(2) = tactel_cartesian_coordinates.col(2) * -1.0;


    // CYLINDER COORDINATES
    // radio_cilindro - angulo -  posicion x
    //posicion x:
    tactel_cylindrical_coordinates.col(2) = tactel_cartesian_coordinates.col(0);
    double sin_value;
    double angle;
    for(int i=0; i<NUM_TACTELS; i++){
      // radio:
      tactel_cylindrical_coordinates(i,0) = radio_cylinder;

      // angulo:
      sin_value = tactel_cartesian_coordinates(i,2) / sqrt( pow(tactel_cartesian_coordinates(i,1),2) + pow(tactel_cartesian_coordinates(i,2),2) );
      angle = asin(sin_value);

      if (tactel_cartesian_coordinates(i,1) > 0)
        tactel_cylindrical_coordinates(i,1) = 3.1415 - angle;
      else
        tactel_cylindrical_coordinates(i,1) = 3.1415 - angle;
    }


    // PLANE COORDINATES
    /**
    pasar de proyeccion cilindro a imagen plana:
    0->0; 2*radio->image_width
    posicion x = new_posicion_x
    posicion_y = (proyected_position(:,2) * virtual_image width) / 2*radio;
    */

    tactel_plane_coordinates.col(0) = tactel_cylindrical_coordinates.col(2);
    tactel_plane_coordinates.col(1) = tactel_cylindrical_coordinates.col(1) * (virtual_image_width/3.1415);

    
    // INCIALIZAR IMAGEN VIRTUAL DEPENDIENDO DEL TAMANHO DE PIXEL
    pixels_vertical = round(virtual_image_height/pixel_size)+1;
    pixels_horizontal = round(virtual_image_width/pixel_size)+1;
    virtual_image =  ArrayXXf::Zero(pixels_vertical,pixels_horizontal);  

    // MAPEAR POSICIONES ELECTRODOS A POSICIONES PIXELS
    tactel_pixels_positions =  ArrayXXf::Zero(NUM_TACTELS,2);
    double pos_x, pos_y;
    for(int it=1; it<NUM_TACTELS; it++){ 

        pos_x = round( (tactel_plane_coordinates(it,0) * (pixels_vertical+1)) / -virtual_image_height);
        pos_y = round( (tactel_plane_coordinates(it,1) * (pixels_horizontal+1)) / -virtual_image_width);
        if( (pos_x == 0) || (pos_x < 0) )
            pos_x = 1;
        if( (pos_y == 0) || (pos_y < 0) )
            pos_y = 1;
        tactel_pixels_positions(it,0) = pos_x+1;
        tactel_pixels_positions(it,1) = pos_y;
    }

    // CREAR MATRIZ DE KNN: [[indice_electrodo_vecino,...]  [distancia_electrodo_vecino,...] ]
    nearest_neighbors_positions =  ArrayXXf::Zero(NUM_TACTELS,2*NUM_NEIGHBORS);  
    MatrixXf partial_neighbors_matrix;

    for(int i=0; i<NUM_TACTELS; i++){
      // Inicializar vector_indices y vector_distancias = max_distance
      MatrixXf vector_indices = ArrayXXf::Zero(1,NUM_NEIGHBORS);
      MatrixXf vector_distances = ArrayXXf::Zero(1,NUM_NEIGHBORS);
      double max_distance = 2 * (sqrt(  (pow(pixels_vertical,2)) + (pow(pixels_horizontal,2)) ));
      for(int j=0;j<NUM_NEIGHBORS;j++){   vector_distances(0,j) = max_distance; }

      // recorrer tactels candidatos y obtener lista vecinos para cada tactel
      double distance;
      for(int candidate_tactel=0; candidate_tactel<NUM_TACTELS; candidate_tactel++){
        distance = sqrt(  pow(tactel_pixels_positions(i,0)-tactel_pixels_positions(candidate_tactel,0),2) +
                            pow(tactel_pixels_positions(i,1)-tactel_pixels_positions(candidate_tactel,1),2)  );
        // Si candidato != actual  && vector_vecinos no lleno o distancia < distancia maxima actual -> añadir este candidadto
        if( (i!=candidate_tactel) && ( (vector_indices(0,NUM_NEIGHBORS-1)==0) || (distance<max_distance) )  ){
          vector_indices(0,NUM_NEIGHBORS-1) = candidate_tactel+1;
          vector_distances(0,NUM_NEIGHBORS-1) = distance;
          partial_neighbors_matrix = orderVectors(vector_indices,vector_distances);
          vector_indices = partial_neighbors_matrix.row(0);
          vector_distances = partial_neighbors_matrix.row(1);
        }
      }

      // Insertar en matriz global de vecinos
      for(int j=0; j<NUM_NEIGHBORS; j++){
        nearest_neighbors_positions(i,j)= vector_indices(0,j);
        nearest_neighbors_positions(i,j+NUM_NEIGHBORS) = vector_distances(0,j);      
      }
    }

  } // End constructor node


  // Funcion para ordenar matriz de vecinos
  MatrixXf orderVectors(MatrixXf input_vector_indices, MatrixXf input_vector_distances  ){
    
    MatrixXf old_vector_distances = input_vector_distances;
    MatrixXf old_vector_indices = input_vector_indices;
    MatrixXf output_matrix =  ArrayXXf::Zero(2,NUM_NEIGHBORS);
 
    double minimum_value = old_vector_distances(0,0);
    int iteration;
    double current_size;
    int position_of_minimum = 0;

    for(int i=0; i<NUM_NEIGHBORS; i++){
      current_size = old_vector_indices.cols();
      for(int position=0; position<current_size; position++){
        if(old_vector_distances(position) < minimum_value){
          position_of_minimum = position;
          minimum_value = old_vector_distances(position);
        }
      }

      // insertar en salida
      output_matrix(0,iteration) = old_vector_indices(0,position_of_minimum);
      output_matrix(1,iteration) = old_vector_distances(0,position_of_minimum);
      iteration++;

      // Borrar de old vector
      removeColumn(old_vector_indices,position_of_minimum);
      removeColumn(old_vector_distances,position_of_minimum);
    }

    return output_matrix;
  }

  // Function eliminar columna de matriz
  void removeColumn(MatrixXf& matrix, unsigned int colToRemove)
  {
    unsigned int numRows = matrix.rows();
    unsigned int numCols = matrix.cols()-1;

    if( colToRemove < numCols )
      matrix.block(0,colToRemove,numRows,numCols-colToRemove) = matrix.block(0,colToRemove+1,numRows,numCols-colToRemove);

    matrix.conservativeResize(numRows,numCols);
  }

  // Funcion para obetener indice tactil segun posicion pixels
  int getIndexTactel(int pos_x, int pos_y){
    int index_tactel = 0;
    for(int i=0; i < NUM_TACTELS; i++){
      if((tactel_pixels_positions(i,0)==pos_x) && (tactel_pixels_positions(i,1)==pos_y))
        index_tactel = i;
    }
    return index_tactel;
  }


}; // End class node



/** CLASE FINGER: encapsular topics joints y valor tactil
*
*
*/

class Finger {

private:

public:
  ros::Publisher j0_pub;
  ros::Publisher j1_pub;
  ros::Publisher j2_pub;
  ros::Publisher j3_pub;
  ros::Publisher j4_pub;
  ros::Publisher j5_pub;

  std::vector<double> tactile_values;



  //Constructor
  Finger(string j0_name,string j1_name,string j2_name,string j3_name,string j4_name,string j5_name,string tactile_param, NodeClass* a)
  {
    if(j0_name.compare("-") != 0)
      j0_pub = a->n.advertise<std_msgs::Float64>(j0_name, 1000);
    if(j1_name.compare("-") != 0)
      j1_pub = a->n.advertise<std_msgs::Float64>(j1_name, 1000);
    if(j2_name.compare("-") != 0)
      j2_pub = a->n.advertise<std_msgs::Float64>(j2_name, 1000);
    if(j3_name.compare("-") != 0)
      j3_pub = a->n.advertise<std_msgs::Float64>(j3_name, 1000);
    if(j4_name.compare("-") != 0)
      j4_pub = a->n.advertise<std_msgs::Float64>(j4_name, 1000);
    if(j5_name.compare("-") != 0)
      j5_pub = a->n.advertise<std_msgs::Float64>(j5_name, 1000);

    a->n.getParam(tactile_param,tactile_values);
  }

};




/**
Main:  nodo control tactil
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "tactile_servo_biotac");
  NodeClass* a = new NodeClass();
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Clases para transformaciones con tf2
  tf2_ros::Buffer tfBuffer;
  tf::TransformListener tfListener; 
  geometry_msgs::TransformStamped ff_transf;  
  

  /**
  *  BUCLE OBTENCION IMAGEN  -  TACTILE CONTROL
  *
  */

  do{

    Finger* current_finger;

    for(int finger = 0; finger < 5; finger++){

      switch(finger){
        // 0: thumb
        case 0: 
          {current_finger = new Finger("-","/sh_thj1_position_controller/command","/sh_thj2_position_controller/command",
                    "/sh_thj3_position_controller/command","/sh_thj4_position_controller/command","/sh_thj5_position_controller/command",
                    "/biotac_electrodes_th",a);
          break;}

        // 1: first
        case 1: 
          {current_finger = new Finger("/sh_ffj0_position_controller/command","-","-",
                    "/sh_ffj3_position_controller/command","/sh_ffj4_position_controller/command","-",
                    "/biotac_electrodes_ff",a);
          break;}

        // 2: middle
        case 2: 
          {current_finger = new Finger("/sh_mfj0_position_controller/command","-","-",
                    "/sh_mfj3_position_controller/command","/sh_mfj4_position_controller/command","-",
                    "/biotac_electrodes_mf",a);
          break;}

        // 3: ring
        case 3: 
          {current_finger = new Finger("/sh_rfj0_position_controller/command","-","-",
                    "/sh_rfj3_position_controller/command","/sh_rfj4_position_controller/command","-",
                    "/biotac_electrodes_rf",a);
          break;}

        // 4: little
        case 4: 
          {current_finger = new Finger("/sh_lfj0_position_controller/command","-","-",
                    "/sh_lfj3_position_controller/command","/sh_lfj4_position_controller/command","/sh_lfj5_position_controller/command",
                    "/biotac_electrodes_lf",a);
          break;}
      }



      /**
      *  INICIALIZAR VALORES DE IMAGEN TACTIL
      */
      for(int i=0; i<NUM_TACTELS; i++){ 
        a->virtual_image(a->tactel_pixels_positions(i,0),a->tactel_pixels_positions(i,0)) = current_finger->tactile_values[i];
      }

      /**
      * - NORMALIZAR IMAGEN -> gray (0,255)
      * - OBTENER IMAGEN COMPLEMENTARIA
      */
      a->virtual_image *= IMAGE_RANK;
      a->virtual_image /= ZERO_VALUE;
      for(int i=0; i < a->virtual_image.rows();i++){
        for(int j=0; j < a->virtual_image.cols();j++){
          a->virtual_image(i,j) = 255 - a->virtual_image(i,j); 
        }
      }


      /**
      * CREAR ARRAY DE VALORES DE LO KNN
      * size: num_tactels x num_neigh
      */
      a->nearest_neighbors_values =  ArrayXXf::Zero(NUM_TACTELS,NUM_NEIGHBORS);
      for(int i=0; i < NUM_TACTELS; i++){
        for(int j=0; j < NUM_NEIGHBORS; j++){
          // Asignar valores de los vecinos. En nearest_neighbor_positions guardo el indice (1-19) -> en vector (0-18)
          a->nearest_neighbors_values(i,j) = current_finger->tactile_values[(a->nearest_neighbors_positions(i,j))-1]; 
        }
      }  


      /**
      * OBTENER VALORES DE DESVIACION PARA CADA GAUSSIANA DEPENDIENTES DE LOS VECINOS   
      * max_distance = sqrt((pixels_horiz.^2)   +  (pixels_vertical.^2)  );
      * obtener valor gaussiana para cada electrodo. Inversamente
      * proporcional a la distanci y directamente proporcional al valor de
      * cada uno de los electrodos incluidos dentro de los k-nn. Media de los
      * valores.
      *
      */
      double max_distance = sqrt((pow(a->pixels_horizontal,2))   +  (pow(a->pixels_vertical,2))  );
      a->deviations_array = ArrayXXf::Zero(NUM_TACTELS,1);
      double gaussian_deviation_value, distance_factor, gray_normalized_value;
      for(int i=0; i < NUM_TACTELS; i++){
        gaussian_deviation_value = 0;
        for(int j=0; j<NUM_NEIGHBORS; j++){
          distance_factor = a->nearest_neighbors_positions(i,NUM_NEIGHBORS+j) / max_distance;
          // normalizar valor del pixel
          gray_normalized_value = (a->nearest_neighbors_values(i,j) * a->pixels_vertical) / 255;
          gaussian_deviation_value = gaussian_deviation_value + (gray_normalized_value * distance_factor);
        }
        a->deviations_array(i,0) = gaussian_deviation_value / NUM_NEIGHBORS;
      }


      /**
      * 
      *
      * MIXTURE OF GAUSSIAN METHOD
      */
      a->mixture_gaussian_image = ArrayXXf::Zero(a->pixels_vertical, a->pixels_horizontal);
      double value_i_j;
      int index_tactel;

      for(int i=0; i<a->pixels_vertical; i++){
        for(int j=0; j<a->pixels_horizontal; j++){
          value_i_j = 0;

          // Recorrer imagen dando valores a cada pixel dependiendo de la suma de gaussianas dinamica en cada punto. Valores 0, descartados
          for(int ui=0; ui<a->pixels_vertical; ui++){
            for(int uj=0; uj<a->pixels_horizontal; uj++){
              if(a->virtual_image(ui,uj) != 0){
                index_tactel = a->getIndexTactel(ui,uj);
                if(index_tactel!=0)
                  value_i_j += a->virtual_image(ui,uj) * ( 
                                    exp(
                                      (-1) *
                                      ( (pow(i-ui,2)) + (pow(j-uj,2)) )
                                          /
                                      (2 * pow(a->deviations_array(index_tactel),2) )
                                      )
                                                        );                          
              }

            }
          } // End for interior

          a->mixture_gaussian_image(i,j) = value_i_j;
        }
      }// End for exterior



      /**
      *
      *  BUCLE DE CONTROL:
      * error = mixture_gaussian_image - desired_gaussian_image
      * v = Jacobian * error
      *
      */


    }

  }while(true);

}

