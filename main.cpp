#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <gazebo_msgs/ModelStates.h>
#include <sensor_msgs/Imu.h>

#include <image_transport/image_transport.h>
//#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

static const std::string OPENCV_WINDOW = "Image window";

const bool debug = true;

void mostrarImagenEnVentana(cv::Mat image);
void verCamaraFrontal(const sensor_msgs::ImageConstPtr& msg);
void verCamaraIzquierda(const sensor_msgs::ImageConstPtr& msg);
void verCamaraDerecha(const sensor_msgs::ImageConstPtr& msg);
void verCamaraFrontalNormalizada(const sensor_msgs::ImageConstPtr& msg);

class RobotDriver
{
private:
  double forwardVel;
  double rotateVel;
  double valueIzN, valueDeN;
  double valueIzExtAnt, valueDeExtAnt;
  double valueI, valueF, valueD, valueIzExt, valueDeExt;
  int contI, contF, contD, contIE, contDE;

  // Orientacion del robot real
  double orientacion;
  // Orientacion del robot antes de dar la vuelta
  double orientacionGiro;
  // Posicion robot en int para usarla con la matriz interna del mapa
  struct posicion{
    int x, y;
  }posRobot;

  bool camIncorrecto;
  bool esquinaIzquierda, esquinaDerecha;

  // El mapa mide 20x20
  int mapaCarrera[20][20];
  
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_vel_pub_;
  ros::Publisher scan_pub;

  ros::Subscriber laserSub;
  ros::Subscriber ms;
  ros::Subscriber imu;

  ros::Subscriber frontRGBSub;
  ros::Subscriber rearRGB1Sub;
  ros::Subscriber rearRGB2Sub;

  cv_bridge::CvImagePtr cv_ptr_izq;
  cv_bridge::CvImagePtr cv_ptr_der;


public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    // Inicializacion de las variables
    forwardVel = 0.0;
    rotateVel = 0.0;

    valueIzN = 1.5; valueDeN = 4.5;
    valueIzExtAnt = 10; valueDeExtAnt = 10;
    valueI = 0.1; valueF = 0.1; valueD = 0.1; valueIzExt = 0.1; valueDeExt = 0.1;
    contI = 1; contF = 1; contD = 1; contIE = 1; contDE = 1;
    
    esquinaIzquierda = false;
    esquinaDerecha = false;
    camIncorrecto = false;

    // Inicializacion del mapa
    for(unsigned i=0;i<20;++i){
      for(unsigned j=0;j<20;++j){
        mapaCarrera[i][j] = 0;
      }
    }
    
    nh_ = nh;

    //ultimoGiro = ' ';
    
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot2/commands/velocity", 1);
    
    // Para usar el imu descomentar
    // imu = nh.subscribe("/robot2/sensors/imu_data", 1, &RobotDriver::commandImu, this);

    // ModelStatesMessage
    ms = nh.subscribe("/gazebo/model_states", 1, &RobotDriver::mapa, this);
    // Para la navegacion
    laserSub = nh.subscribe("/robot2/scan", 1, &RobotDriver::procesaDatosLaser, this);
    // Para sacar el tamaño del robot en el laser
    //laserSub = nh.subscribe("/robot2/scan", 1, &RobotDriver::compruebaRobot, this);

    frontRGBSub = nh.subscribe("/robot2/camera/rgb/image_raw", 1, &RobotDriver::procesaDatosMonofocal, this);
    rearRGB1Sub = nh.subscribe("/robot2/trasera1/trasera1/rgb/image_raw", 1, &RobotDriver::procesaDatosBifocalIzq, this);
    rearRGB2Sub = nh.subscribe("/robot2/trasera2/trasera2/rgb/image_raw", 1, &RobotDriver::procesaDatosBifocalDer, this);
  }
  
  //sensor_msgs/Imu
  void commandImu(const sensor_msgs::Imu::ConstPtr& msg){
    std::cout << "IMU" << std::endl;
    //ROS_INFO_STREAM(msg->orientation);
    double prueba = atan2(2*(msg->orientation.x*msg->orientation.y + msg->orientation.w*msg->orientation.z), 
        msg->orientation.w*msg->orientation.w +
        msg->orientation.x*msg->orientation.x -
        msg->orientation.y*msg->orientation.y - 
        msg->orientation.z*msg->orientation.z);

    std::cout << "IMU_Orientacion despues del calculo: " << prueba << std::endl << std::endl;
    std::cout << "IMU_linear_aceleration: " << msg->linear_acceleration << std::endl;

    std::cout << std::endl;
  }

  // El robot 2 esta en la i = 53
  void mapa(const gazebo_msgs::ModelStates::ConstPtr& msg){
    unsigned suma = 0;
    ROS_INFO_STREAM("Nombre: " << msg->name[53]);
    ROS_INFO_STREAM("Posicion: \n" << msg->pose[53].position);
    posRobot.x = msg->pose[53].position.x+10;
    posRobot.y = msg->pose[53].position.y+10;

    //ROS_INFO_STREAM("Orientacion: \n" << msg->pose[53].orientation);
    orientacion = atan2(2*(msg->pose[53].orientation.x*msg->pose[53].orientation.y + msg->pose[53].orientation.w*msg->pose[53].orientation.z), 
        msg->pose[53].orientation.w*msg->pose[53].orientation.w +
        msg->pose[53].orientation.x*msg->pose[53].orientation.x -
        msg->pose[53].orientation.y*msg->pose[53].orientation.y - 
        msg->pose[53].orientation.z*msg->pose[53].orientation.z);

    std::cout << "Orientacion despues del calculo: " << orientacion << std::endl << std::endl;

    // Comprobar si nos hemos dado la vuelta o no
    if(orientacion > -0.75 && orientacion < 0.75){
      // Avanzas hacia el sur
      // Pon la posicion del mapa interno a 1 dnd estas y en tu linea
      mapaCarrera[posRobot.x][posRobot.y-1] = 1;
      mapaCarrera[posRobot.x][posRobot.y] = 1;
      mapaCarrera[posRobot.x][posRobot.y+1] = 1;
      
      // comprueba si ya has ido hacia esa posicion
      suma = mapaCarrera[posRobot.x+2][posRobot.y-1] + mapaCarrera[posRobot.x+2][posRobot.y] + mapaCarrera[posRobot.x+2][posRobot.y+1];
    } else if(orientacion < -0.75 && orientacion > -2.25){
      // Avanzas hacia el oeste
      // Pon la posicion del mapa interno a 1 dnd estas y en tu linea
      mapaCarrera[posRobot.x-1][posRobot.y] = 1;
      mapaCarrera[posRobot.x][posRobot.y] = 1;
      mapaCarrera[posRobot.x+1][posRobot.y] = 1;
      
      // comprueba si ya has ido hacia esa posicion
      suma = mapaCarrera[posRobot.x-1][posRobot.y-2] + mapaCarrera[posRobot.x][posRobot.y-2] + mapaCarrera[posRobot.x+1][posRobot.y-2];

    } else if(orientacion < -2.25 && orientacion < 2.25){
      // Avanzas hacia el norte
      // Pon la posicion del mapa interno a 1 dnd estas y en tu linea
      mapaCarrera[posRobot.x][posRobot.y-1] = 1;
      mapaCarrera[posRobot.x][posRobot.y] = 1;
      mapaCarrera[posRobot.x][posRobot.y+1] = 1;
      
      // comprueba si ya has ido hacia esa posicion
      suma = mapaCarrera[posRobot.x-2][posRobot.y-1] + mapaCarrera[posRobot.x-2][posRobot.y] + mapaCarrera[posRobot.x-2][posRobot.y+1];
    
    } else {
      // Avanzas hacia el este
      // Pon la posicion del mapa interno a 1 dnd estas y en tu linea
      mapaCarrera[posRobot.x-1][posRobot.y] = 1;
      mapaCarrera[posRobot.x][posRobot.y] = 1;
      mapaCarrera[posRobot.x+1][posRobot.y] = 1;
      
      // comprueba si ya has ido hacia esa posicion
      suma = mapaCarrera[posRobot.x-1][posRobot.y+2] + mapaCarrera[posRobot.x][posRobot.y+2] + mapaCarrera[posRobot.x+1][posRobot.y+2];
    }

    // Si la suma es > 2 y el camino es el correcto
    if(suma > 2 && !camIncorrecto){
      camIncorrecto = true;
      if(orientacion>0)
        orientacionGiro = orientacion-3;
      else
        orientacionGiro = orientacion+3;
      std::cout << "OJO QUE ESTAS YENDO AL REVES" <<std::endl;
    }
  }


  void procesaDatosMonofocal(const sensor_msgs::ImageConstPtr& msg){
    if (debug) {
        //verCamaraFrontal(msg);
        verCamaraFrontalNormalizada(msg);
        //verPanoramica(msg);
      }
    }

  void verPanoramica() {
    if (cv_ptr_izq!=0 && cv_ptr_der!=0)
      procesaDatosBifocal();
  }

  //void procesaDatosBifocal(const sensor_msgs::ImageConstPtr& msg){
  void procesaDatosBifocal(){

/*    cv_bridge::CvImagePtr cv_ptr_izq;
    cv_bridge::CvImagePtr cv_ptr_der;

    // 1.- Obtener 2 imágenes
    // 2.- Calcular KeyPoints (2 imágenes)
    // 3.- Calcular Descriptores (2A Key Points)
    // 4.- Calcular Correspondencias (Filtrar)
    // 5.- Obtener Homografía
    // 6.- Aplicar transformación + Unir

    // 1.- Obtener 2 imágenes
    // 2.- Calcular KeyPoints (2 imágenes)
    SurfFeatureDetector detector(400);
    vector<KeyPoint> keypoints1;
    detector.detect(img1, keypoints1);
    // 3.- Calcular Descriptores (2A Key Points)
    SurfDescriptorExtractor extractor;
    Mat descriptors1;
    extractor.compute(img1, keypoints1, descriptors1);
    // 4.- Calcular Correspondencias (Filtrar)
    BFMatcher matcher(NORM_L2);
    vector<DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);
    // 5.- Obtener Homografía
    // 6.- Aplicar transformación + Unir
    void FeatureDetector::detect(const Mat& image, vector<KeyPoint>& keypoints, const Mat& mask=Mat() ) const;


    SurfFeatureDetector detector(400);
    vector<KeyPoint> keypoints1;
    detector.detect(img1, keypoints1);
    // 3.- Calcular Descriptores (2A Key Points)
    SurfDescriptorExtractor extractor;
    Mat descriptors1;
    extractor.compute(img1, keypoints1, descriptors1);
    // 4.- Calcular Correspondencias (Filtrar)
    BFMatcher matcher(NORM_L2);
    vector<DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches);
    // 5.- Obtener Homografía
    // 6.- Aplicar transformación + Unir
    void FeatureDetector::detect(const Mat& image, vector<KeyPoint>& keypoints, const Mat& mask=Mat() ) const;
*/ 

    std::cout << "Iniciando Panoramica!!" << std::endl;

    // 1.- Obtener 2 imágenes
    //cv::Mat image1 = cv_ptr_der->image;
    //cv::Mat image2 = cv_ptr_der->image;

    // 2.- Calcular KeyPoints (2 imágenes)
    //-- Step 1: Detect the keypoints using SURF Detector
    int minHessian = 800;

    cv::SurfFeatureDetector detector( minHessian );
    std::vector< cv::KeyPoint > keypoints_object, keypoints_scene;

    detector.detect( cv_ptr_der->image, keypoints_object );
    detector.detect( cv_ptr_izq->image, keypoints_scene );

    //-- Step 2: Calculate descriptors (feature vectors)
    cv::SurfDescriptorExtractor extractor;

    cv::Mat descriptors_object, descriptors_scene;
    extractor.compute( cv_ptr_der->image, keypoints_object, descriptors_object );
    extractor.compute( cv_ptr_izq->image, keypoints_scene, descriptors_scene );

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    cv::FlannBasedMatcher matcher;
    std::vector< cv::DMatch > matches;
    matcher.match( descriptors_object, descriptors_scene, matches );
    double max_dist = 0; 
    double min_dist = 100;

    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < descriptors_object.rows; i++ ) { 
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }
    printf("-- Max dist : %f \n", max_dist );
    printf("-- Min dist : %f \n", min_dist );

    //-- Use only "good" matches (i.e. whose distance is less than 3*min_dist )
    std::vector< cv::DMatch > good_matches;

    for( int i = 0; i < descriptors_object.rows; i++ ) { 
        if( matches[i].distance < 3*min_dist )
            good_matches.push_back( matches[i]);
    }
    std::vector< cv::Point2f > obj;
    std::vector< cv::Point2f > scene;
    for( int i = 0; i < good_matches.size(); i++ ){
        //-- Get the keypoints from the good matches
        obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
        scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
    }
    // Find the Homography Matrix
    try {
      std::cout << "--Homografia--" << std::endl;
      cv::Mat H = findHomography( obj, scene, CV_RANSAC );
      // Use the Homography Matrix to warp the images
      cv::Mat result;
      cv::warpPerspective(cv_ptr_izq->image,result,H,cv::Size(cv_ptr_izq->image.cols+cv_ptr_der->image.cols,cv_ptr_izq->image.rows));
      cv::Mat half(result,cv::Rect(0,0,cv_ptr_der->image.cols,cv_ptr_der->image.rows));
      cv_ptr_der->image.copyTo(half);
      cv::namedWindow("Result");
      cv::startWindowThread();
      cv::imshow( "Result", result );

      cv::waitKey(500);
    } catch (cv::Exception e) {
      std::cerr << "Se ha detectado una exception al realizar la Homografía: " + e.err << std::endl;
    }

    //A partir de aqui
    //Se fusionan las imagenes de cv_ptr_izq y cv_ptr_der.
    //http://stackoverflow.com/questions/11134667/some-problems-on-image-stitching-homography?lq=1
    //FindContours
    //http://docs.opencv.org/3.1.0/d3/dc0/group__imgproc__shape.html#ga17ed9f5d79ae97bd4c7cf18403e1689a&gsc.tab=0
  }

   void procesaDatosBifocalIzq(const sensor_msgs::ImageConstPtr& msg){
      try {
        if (debug)
          verCamaraIzquierda(msg);
        cv_ptr_izq = cv_bridge::toCvCopy(msg, msg->encoding);
      }
      catch (cv_bridge::Exception& e){
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
      }
   }

   void procesaDatosBifocalDer(const sensor_msgs::ImageConstPtr& msg){
      try {
        if (debug)
          verCamaraDerecha(msg);
        cv_ptr_der = cv_bridge::toCvCopy(msg, msg->encoding);
      }
      catch (cv_bridge::Exception& e){
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
      }
   }

  void giroEsquinaDerecha(){
    std::cout << "GIRANDO POR LA FUNCION ESQUINA DERECHA!!" << std::endl;
    if(valueF<5 && valueF>1.5){
      rotateVel = -0.4;
      forwardVel = 0.2;
    }
    else{
      esquinaDerecha = false;
    }
  }

  void giroEsquinaIzquierda(){
    std::cout << "GIRANDO POR LA FUNCION ESQUINA IZQUIERDA!!" << std::endl;
    if(valueF<5 && valueF>1.5){
      rotateVel = 0.4;
      forwardVel = 0.2;
    }
    else{
      esquinaIzquierda = false;
    }
  }

  void darVueltaRobot(){
    std::cout << "MI ORIENTACION ACTUAL ES: " << orientacion << ", Y LA ORIENTACION A LA QUE TENGO QUE LLEGAR ES: " << orientacionGiro << "\n";
    if(orientacion<orientacionGiro+0.2 && orientacion>orientacionGiro-0.2){
      camIncorrecto = false;
    }
    else{
      rotateVel = 0.4;
      forwardVel = 0;
    }

  }

  void procesaDatosLaser(const sensor_msgs::LaserScan::ConstPtr& msg){
    if(debug){
      // Mínimo valor angular del láser -0.521568
      ROS_INFO_STREAM("AngleMin: " << msg->angle_min);
      // Máximo valor angular del láser 0.524276
      ROS_INFO_STREAM("AngleMax: " << msg->angle_max);
      // Incremento angular entre dos beams 0.00163669
      ROS_INFO_STREAM("AngleIncrement: " << msg->angle_increment); 
      // Mínimo valor que devuelve el láser 0.45
      ROS_INFO_STREAM("RangeMin: " << msg->range_min); 
      // Máximo valor que devuelve el láser. Valores por debajo y 
      // por encima de estos rangos no deben ser tenidos en cuenta 10
      ROS_INFO_STREAM("RangeMax: " << msg->range_max); 
    }
    // Me esta dando 639 valores del laser
    int totalValues = ceil((msg->angle_max-msg->angle_min)/msg->angle_increment); // Total de valores que devuelve el láser

    valueI = 0.1; valueF = 0.1; valueD = 0.1; valueIzExt = 0.1; valueDeExt = 0.1;
    contI = 1; contF = 1; contD = 1; contIE = 1; contDE = 1;
    
    // 4
    for(int i=0;i<totalValues;++i){
      if(i>=0 && i<3){
        if(!std::isnan(msg->ranges[i])){
          valueDeExt += msg->ranges[i];
          contDE++;
        }
      }
      // 210
      if(i>=0 && i<210){
        if(!std::isnan(msg->ranges[i])){
          valueD += msg->ranges[i];
          contD++;
        }
      }
      // 150
      if(i>=244 && i<394){
        if(!std::isnan(msg->ranges[i])){
          valueF += msg->ranges[i];
          contF++;
        }
      }
      // 210
      if(i>=429 && i<639){
        if(!std::isnan(msg->ranges[i])){
          valueI += msg->ranges[i];
          contI++;
        }
      }
      // 4
      if(i>=636 && i<639){
        if(!std::isnan(msg->ranges[i])){
          valueIzExt += msg->ranges[i];
          contIE++;
        }
      }
      //std::cout << "i " << i << ": " << msg->ranges[i] << std::endl;
    }
    valueDeExt /= contDE;
    ROS_INFO_STREAM("valueDeExt:" << valueDeExt << "; " << contDE); // Acceso a los valores de rango
    valueD /= contD;
    ROS_INFO_STREAM("ValueD:" << valueD << "; " << contD); // Acceso a los valores de rango
    valueF /= contF;
    ROS_INFO_STREAM("ValueF:" << valueF << "; " << contF); // Acceso a los valores de rango
    valueI /= contI;
    ROS_INFO_STREAM("ValueI:" << valueI << "; " << contI); // Acceso a los valores de rango
    valueIzExt /= contIE;
    std::cout << "valueIzExtAnt: " << valueIzExtAnt << std::endl;
    ROS_INFO_STREAM("valueIzExt:" << valueIzExt << "; " << contIE); // Acceso a los valores de rango
    std::cout << "valueDeExtAnt: " << valueDeExtAnt << std::endl;
    ROS_INFO_STREAM("valueDeExt:" << valueDeExt << "; " << contDE); // Acceso a los valores de rango
    std::cout << std::endl;

    // rotateVel positivo gira a la izquierda
    // rotateVel negativo gira a la derecha

    // Pruebas de la navegacion
    if(camIncorrecto){
      darVueltaRobot();
    }else{
      if(valueIzExt > valueIzExtAnt+0.5){
        esquinaIzquierda = true;
        std::cout << "ESQUINAAAAAAAAAAAAAA IZQUIERDA" << std::endl;
        // Sacar la orientacion y no parar hasta que haya girado 90º??
      }
      /**else if(valueDeExt > valueDeExtAnt+0.5){
        esquinaDerecha = true;
        std::cout << "ESQUINAAAAAAAAAAAAA DERECHA" << std::endl;
      }*/
      if(!esquinaIzquierda && !esquinaDerecha){
        if((valueI < valueIzN+0.5 && valueI > valueIzN-0.5) && valueF > 1.1){
          std::cout << "SIGO RECTO" << std::endl;
          forwardVel = 0.5;
          rotateVel = 0;
        }
        else if(valueI > valueIzN+0.4 && valueF > 1.1){
          std::cout << "AVANZO PERO AJUSTANDO A LA IZQUIERDA" << std::endl;
          forwardVel = 0.5;
          rotateVel = 0.3;
        }
        else if(valueI < valueIzN-0.4 && valueF > 1.1){
          std::cout << "AVANZO PERO AJUSTANDO A LA DERECHA" << std::endl;
          forwardVel = 0.5;
          rotateVel = -0.3;
        }
        // Hasta aqui es para que siga la pared izquierda
        // Para que si esta muy cerca de la pared o de un obstaculo intente evitarlo
        
        else if(valueF < 1.1){
          forwardVel = 0.1;
          if(valueI < valueD){
            std::cout << "ENTRO PARA GIRAR A LA DERECHA" << std::endl;
            rotateVel = -0.4;
          }
          else{
            rotateVel = 0.4;
            std::cout << "ENTRO PARA GIRAR A LA IZQUIERDA" << std::endl;
          }
        }
        else forwardVel = 0;
      }
      else if(esquinaIzquierda){
        giroEsquinaIzquierda();
      }
      /**
      else if(esquinaDerecha){
        giroEsquinaDerecha();
      }
      */
      else{
        std::cout << "NO ESTOY ACTUALIZANDO NADA" << std::endl;
      }

      valueIzExtAnt = valueIzExt;
      valueDeExtAnt = valueDeExt;
    }
  }

  void bucle(){
    ros::Rate rate(10);
    std::cout << "Iniciando bucle para siempre" << std::endl;
    while (ros::ok()){
      geometry_msgs::Twist base_cmd; // Este mensaje es el que se publicara para decir las velocidades linear y angular del robot
      base_cmd.linear.x = forwardVel; // Velocidad linear que tendra el colega
      base_cmd.angular.z = rotateVel; // Velocidad angular que tendra el colega

      //publish the assembled command
      cmd_vel_pub_.publish(base_cmd);
      ros::spinOnce(); // Se procesaran todas las llamadas que queden pendientes (como procesaDatosLaser)
      rate.sleep(); // Con esto esperara a que acabe el ciclo
      if (debug)
        verPanoramica();
    }

    if (debug) {
       ros::spin();
       ros::shutdown();
       cv::destroyWindow("view");
    }
  }

};

void verCamaraFrontal(const sensor_msgs::ImageConstPtr& msg){
   cv::namedWindow("view");
   cv::startWindowThread();
   try { 
      cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
      cv::waitKey(30);
   }
   catch (cv_bridge::Exception& e) {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
   }
}

void verCamaraDerecha(const sensor_msgs::ImageConstPtr& msg){
   cv::namedWindow("derecha");
   cv::startWindowThread();
   try { 
      cv::imshow("derecha", cv_bridge::toCvShare(msg, "bgr8")->image);
      cv::waitKey(30);
   }
   catch (cv_bridge::Exception& e) {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
   }
}

void verCamaraIzquierda(const sensor_msgs::ImageConstPtr& msg){
   cv::namedWindow("izquierda");
   cv::startWindowThread();
   try { 
      cv::imshow("izquierda", cv_bridge::toCvShare(msg, "bgr8")->image);
      cv::waitKey(30);
   }
   catch (cv_bridge::Exception& e) {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
   }
}

void verCamaraFrontalNormalizada(const sensor_msgs::ImageConstPtr& msg){
   cv::namedWindow("view");
   cv::startWindowThread();
   try {
      cv_bridge::CvImageConstPtr cv_ptr;
      cv_ptr = cv_bridge::toCvShare(msg);
      //cv_bridge::toCvCopy(msg, msg->encoding);

      // imshow expects a float value to lie in [0,1], so we need to normalize
      // for visualization purposes.
      double max = 0.0;
      cv::minMaxLoc(cv_ptr->image, 0, &max, 0, 0);
      cv::Mat normalized;
      cv_ptr->image.convertTo(normalized, CV_32F, 1.0/max, 0);

      cv::imshow("view", normalized);
      cv::waitKey(1);
   } catch (const cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
   }
}

/*
void mostrarImagenEnVentana(cv::Mat image) {
  cv::namedWindow(OPENCV_WINDOW);
  cv::startWindowThread();
   try {
    cv::imshow(OPENCV_WINDOW, image);
    cv::waitKey(2000);
    cv::destroyWindow(OPENCV_WINDOW);
  } catch (const cv::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}
*/
/*
void mostrarImagenEnVentana(cv::Mat image) {
  cv_bridge::CvImage out_msg;
  out_msg.header   = in_msg->header; // Same timestamp and tf frame as input image
  out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
  out_msg.image    = image; // Your cv::Mat
  cout<<"EOOOOOOOOOOO"<<endl;
  cv::namedWindow(OPENCV_WINDOW);
  cv::startWindowThread();
   try {

    cv::imshow(OPENCV_WINDOW, cv_bridge::toCvShare(out_msg, "bgr8")->image););
    cv::waitKey(2000);
    cv::destroyWindow(OPENCV_WINDOW);
  } catch (const cv::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}
*/

int main(int argc, char** argv){   //init the ROS node
   ros::init(argc, argv, "robot_driver");
   ros::NodeHandle nh;

   RobotDriver driver(nh);

   //if (debug) {
      //cv::namedWindow("view");
      //cv::startWindowThread();
      //image_transport::ImageTransport it(nh);
      //image_transport::Subscriber subAux = it.subscribe("robot1/camera/rgb/image_raw", 1, verCamaraFrontal);
   //}

   //driver.driveKeyboard();
   driver.bucle();
   return 0;
}
