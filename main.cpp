#include <iostream>
#include <stdio.h>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <gazebo_msgs/ModelStates.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sys/resource.h>

cv::Mat normalizaImagen(const cv::Mat& m);
void mostrarImagenEnVentana(cv::Mat image);
void verCamaraFrontal(const sensor_msgs::ImageConstPtr& msg);
void verCamaraIzquierda(const sensor_msgs::ImageConstPtr& msg);
void verCamaraDerecha(const sensor_msgs::ImageConstPtr& msg);

using namespace cv;

static const std::string OPENCV_WINDOW = "Image window";

const bool debug = false;
const bool panoramica = true;

class RobotDriver
{
private:
  double forwardVel;
  double rotateVel;
  double valueIzN, valueDeN;
  double valueIzExtAnt, valueDeExtAnt;
  double valueI, valueF, valueD, valueIzExt, valueDeExt, valueFExt;
  int contI, contF, contD, contIE, contDE, contFExt;

  long double tiempoTurboActivado;
  long double tiempoTurboDesactivado;

  // Orientacion del robot real
  double orientacion;
  // Orientacion del robot antes de dar la vuelta
  double orientacionGiro;
  // Posicion robot en int para usarla con la matriz interna del mapa
  struct posicion{
    int x, y;
  } posRobot;

  bool camIncorrecto;
  bool esquinaIzquierda;
  bool turbo;
  bool inicio;

  // El mapa mide 20x20
  int mapaCarrera[20][20];

  Mat descriptorsBack, descriptorsFront, descriptorsLeft, descriptorsRight;
  
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_vel_pub_;
  ros::Publisher scan_pub;

  ros::Subscriber laserSub;
  ros::Subscriber ms;

  ros::Subscriber frontRGBSub;
  ros::Subscriber rearRGB1Sub;
  ros::Subscriber rearRGB2Sub;

  ros::Subscriber inicioCarrera;

  cv_bridge::CvImagePtr cv_ptr_frontal;
  cv_bridge::CvImagePtr cv_ptr_izq;
  cv_bridge::CvImagePtr cv_ptr_der;


public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    // Inicializacion de las variables
    forwardVel = 0.0;
    rotateVel = 0.0;

    valueIzN = 1.8; valueDeN = 4.2;
    valueIzExtAnt = 10; valueDeExtAnt = 10;
    valueI = 0.1; valueF = 0.1; valueD = 0.1; valueIzExt = 0.1; valueDeExt = 0.1, valueFExt = 0.1;
    contI = 1; contF = 1; contD = 1; contIE = 1; contDE = 1, contFExt = 1;

    tiempoTurboActivado = 5;
    tiempoTurboDesactivado = 10;
    
    esquinaIzquierda = false;
    camIncorrecto = false;
    turbo = false;
    inicio = true;

    // Inicializacion del mapa
    for(unsigned i=0;i<20;++i){
      for(unsigned j=0;j<20;++j){
        mapaCarrera[i][j] = 0;
      }
    }

    Mat img_back  = imread( "./robotSolitario1.png", CV_LOAD_IMAGE_GRAYSCALE );
    Mat img_front = imread( "./robotSolitarioGiradoFrontal.png", CV_LOAD_IMAGE_GRAYSCALE );
    Mat img_left  = imread( "./robotSolitarioGiradoIzq.png", CV_LOAD_IMAGE_GRAYSCALE );
    Mat img_right = imread( "./robotSolitarioGiradoDerecha.png", CV_LOAD_IMAGE_GRAYSCALE );

    if( !img_back.data || !img_front.data || !img_left.data || !img_right.data)
      { std::cout<< " --(!) Error reading images " << std::endl; }

    std::vector<KeyPoint> keypoints_1, keypoints_2, keypoints_3, keypoints_4;
    
    SurfFeatureDetector detector(900);
    detector.detect(img_back, keypoints_1);
    detector.detect(img_front, keypoints_2);
    detector.detect(img_left, keypoints_3);
    detector.detect(img_right, keypoints_4);

    SurfDescriptorExtractor extractor;
    extractor.compute(img_back,keypoints_1,descriptorsBack);
    extractor.compute(img_front,keypoints_1,descriptorsFront);
    extractor.compute(img_left,keypoints_1,descriptorsLeft);
    extractor.compute(img_right,keypoints_1,descriptorsRight);

    nh_ = nh;

    //ultimoGiro = ' ';
    
    frontRGBSub = nh.subscribe("/robot2/camera/rgb/image_raw", 1, &RobotDriver::procesaDatosMonofocal, this);
    rearRGB1Sub = nh.subscribe("/robot2/trasera1/trasera1/rgb/image_raw", 1, &RobotDriver::procesaDatosBifocalIzq, this);
    rearRGB2Sub = nh.subscribe("/robot2/trasera2/trasera2/rgb/image_raw", 1, &RobotDriver::procesaDatosBifocalDer, this);

    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot2/commands/velocity", 1);
    
    // Para usar el imu descomentar
    // imu = nh.subscribe("/robot2/sensors/imu_data", 1, &RobotDriver::commandImu, this);

    // Para el inicio de la carrera
    //inicioCarrera = nh.subscribe("/control_tower/race_state", 1, &RobotDriver::iniciaCarrera, this);

    // ModelStatesMessage
    ms = nh.subscribe("/gazebo/model_states", 1, &RobotDriver::mapa, this);
    // Para la navegacion
    //laserSub = nh.subscribe("/robot2/scan", 1, &RobotDriver::procesaDatosLaser, this);
    // Para sacar el tamaño del robot en el laser
    //laserSub = nh.subscribe("/robot2/scan", 1, &RobotDriver::compruebaRobot, this);
  }
  
  /**
  void iniciaCarrera(const std_msgs::String::ConstPtr& msg){
    if(msg->data=="GO")
      inicio = true;
  }
  */

  // El robot 2 esta en la i = 53
  void mapa(const gazebo_msgs::ModelStates::ConstPtr& msg){
    unsigned suma = 0;
    //ROS_INFO_STREAM("Nombre: " << msg->name[53]);
    //ROS_INFO_STREAM("Posicion: \n" << msg->pose[53].position);
    posRobot.x = msg->pose[53].position.x+9;
    posRobot.y = msg->pose[53].position.y+9;
    //std::cout << "Posicion X en la matriz: " << posRobot.x << std::endl;
    //std::cout << "Posicion Y en la matriz: " << posRobot.y << std::endl;

    //ROS_INFO_STREAM("Orientacion: \n" << msg->pose[53].orientation);
    orientacion = atan2(2*(msg->pose[53].orientation.x*msg->pose[53].orientation.y + msg->pose[53].orientation.w*msg->pose[53].orientation.z), 
        msg->pose[53].orientation.w*msg->pose[53].orientation.w +
        msg->pose[53].orientation.x*msg->pose[53].orientation.x -
        msg->pose[53].orientation.y*msg->pose[53].orientation.y - 
        msg->pose[53].orientation.z*msg->pose[53].orientation.z);

    //std::cout << "Orientacion despues del calculo: " << orientacion << std::endl << std::endl;

    // Comprobar si nos hemos dado la vuelta o no
    if(orientacion > -0.8 && orientacion < 0.8 && posRobot.x+2<19 && posRobot.x-2>0){
      // Avanzas hacia el sur
      // Pon la posicion del mapa interno a 1 dnd estas y en tu linea
      mapaCarrera[posRobot.x][posRobot.y-1] = 1;
      mapaCarrera[posRobot.x][posRobot.y] = 1;
      mapaCarrera[posRobot.x][posRobot.y+1] = 1;
      
      // comprueba si ya has ido hacia esa posicion
      suma = mapaCarrera[posRobot.x+2][posRobot.y-1] + mapaCarrera[posRobot.x+2][posRobot.y] + mapaCarrera[posRobot.x+2][posRobot.y+1];
      //std::cout << "suma: " << mapaCarrera[posRobot.x+2][posRobot.y-1] << "+" << mapaCarrera[posRobot.x+2][posRobot.y] << "+" << mapaCarrera[posRobot.x+2][posRobot.y+1] << "=" << suma << std::endl;
    } else if(orientacion < -0.75 && orientacion > -2.25 && posRobot.y+2<19 && posRobot.y-2>0){
      // Avanzas hacia el oeste
      // Pon la posicion del mapa interno a 1 dnd estas y en tu linea
      mapaCarrera[posRobot.x-1][posRobot.y] = 1;
      mapaCarrera[posRobot.x][posRobot.y] = 1;
      mapaCarrera[posRobot.x+1][posRobot.y] = 1;
      
      // comprueba si ya has ido hacia esa posicion
      suma = mapaCarrera[posRobot.x-1][posRobot.y-2] + mapaCarrera[posRobot.x][posRobot.y-2] + mapaCarrera[posRobot.x+1][posRobot.y-2];
      //std::cout << "suma: " << mapaCarrera[posRobot.x-1][posRobot.y-2] << "+" << mapaCarrera[posRobot.x][posRobot.y-2] << "+" << mapaCarrera[posRobot.x+1][posRobot.y-2] << "=" << suma << std::endl;
    } else if(orientacion < -2.25 && orientacion < 2.25 && posRobot.x+2<19 && posRobot.x-2>0){
      // Avanzas hacia el norte
      // Pon la posicion del mapa interno a 1 dnd estas y en tu linea
      mapaCarrera[posRobot.x][posRobot.y-1] = 1;
      mapaCarrera[posRobot.x][posRobot.y] = 1;
      mapaCarrera[posRobot.x][posRobot.y+1] = 1;
      
      // comprueba si ya has ido hacia esa posicion
      suma = mapaCarrera[posRobot.x-2][posRobot.y-1] + mapaCarrera[posRobot.x-2][posRobot.y] + mapaCarrera[posRobot.x-2][posRobot.y+1];
      //std::cout << "suma: " << mapaCarrera[posRobot.x-2][posRobot.y-1] << "+" << mapaCarrera[posRobot.x-2][posRobot.y] << "+" << mapaCarrera[posRobot.x-2][posRobot.y+1] << "=" << suma << std::endl;
    } else if(orientacion > 0.75 && orientacion < 2.25 && posRobot.y+2<19 && posRobot.y-2>0){
      // Avanzas hacia el este
      // Pon la posicion del mapa interno a 1 dnd estas y en tu linea
      mapaCarrera[posRobot.x-1][posRobot.y] = 1;
      mapaCarrera[posRobot.x][posRobot.y] = 1;
      mapaCarrera[posRobot.x+1][posRobot.y] = 1;
      
      // comprueba si ya has ido hacia esa posicion
      suma = mapaCarrera[posRobot.x-1][posRobot.y+2] + mapaCarrera[posRobot.x][posRobot.y+2] + mapaCarrera[posRobot.x+1][posRobot.y+2];
      if(suma>3)
        suma = 0;
      //std::cout << "suma: " << mapaCarrera[posRobot.x-1][posRobot.y+2] << "+" << mapaCarrera[posRobot.x][posRobot.y+2] << "+" << mapaCarrera[posRobot.x+1][posRobot.y+2] << "=" << suma << std::endl;
    }

    // Si la suma es > 2 y el camino es el correcto
    if(suma > 2 && !camIncorrecto){
      camIncorrecto = true;
      if(orientacion>0)
        orientacionGiro = orientacion-2.25;
      else
        orientacionGiro = orientacion+2.25;
      //std::cout << "OJO QUE ESTAS YENDO AL REVES" <<std::endl;
    }
  }


  void procesaDatosMonofocal(const sensor_msgs::ImageConstPtr& msg){
    try{
      if (debug) {
          verCamaraFrontal(msg);
          //verCamaraFrontalNormalizada(msg);
          //verPanoramica(msg);
        }
        cv_ptr_frontal = cv_bridge::toCvCopy(msg, msg->encoding);
    }
    catch (cv_bridge::Exception& e){
       ROS_ERROR("cv_bridge exception: %s", e.what());
       return;
    }
  }

  void verPanoramica() {
    if (panoramica) {
      if (cv_ptr_izq!=0 && cv_ptr_der!=0)
        procesaDatosBifocal();
    }
  }

    
// cv_bridge::CvImagePtr cv_ptr_izq;
// cv_bridge::CvImagePtr cv_ptr_der;

// // 1.- Obtener 2 imágenes
// // 2.- Calcular KeyPoints (2 imágenes)
// // 3.- Calcular Descriptores (2A Key Points)
// // 4.- Calcular Correspondencias (Filtrar)
// // 5.- Obtener Homografía
// // 6.- Aplicar transformación + Unir

// // 1.- Obtener 2 imágenes
// // 2.- Calcular KeyPoints (2 imágenes)
// SurfFeatureDetector detector(400);
// vector<KeyPoint> keypoints1;
// detector.detect(img1, keypoints1);
// // 3.- Calcular Descriptores (2A Key Points)
// SurfDescriptorExtractor extractor;
// Mat descriptors1;
// extractor.compute(img1, keypoints1, descriptors1);
// // 4.- Calcular Correspondencias (Filtrar)
// BFMatcher matcher(NORM_L2);
// vector<DMatch> matches;
// matcher.match(descriptors1, descriptors2, matches);
// // 5.- Obtener Homografía
// // 6.- Aplicar transformación + Unir
// void FeatureDetector::detect(const Mat& image, vector<KeyPoint>& keypoints, const Mat& mask=Mat() ) const;

//A partir de aqui
//Se fusionan las imagenes de cv_ptr_izq y cv_ptr_der.
//http://stackoverflow.com/questions/11134667/some-problems-on-image-stitching-homography?lq=1
//http://docs.opencv.org/2.4/doc/tutorials/introduction/linux_gcc_cmake/linux_gcc_cmake.html#linux-gcc-usage
//FindContours
//http://docs.opencv.org/3.1.0/d3/dc0/group__imgproc__shape.html#ga17ed9f5d79ae97bd4c7cf18403e1689a&gsc.tab=0
void procesaDatosBifocal(){
  //http://docs.opencv.org/2.4/doc/tutorials/features2d/feature_homography/feature_homography.html
  std::cout << "Iniciando Panoramica!!" << std::endl;

  // 1.- Obtener 2 imágenes
  // cv::Mat image1 = normalizaImagen(cv_ptr_der->image);
  // cv::Mat image2 = normalizaImagen(cv_ptr_izq->image);
  cv::Mat image2 = cv_ptr_der->image;
  cv::Mat image1 = cv_ptr_izq->image;
  // cv::namedWindow("Prueba");
  // cv::startWindowThread();
  // cv::imshow( "Prueba", image2 );
  // cv::namedWindow("Prueba1");
  // cv::startWindowThread();
  // cv::imshow( "Prueba1", image1 );

  // 2.- Calcular KeyPoints (2 imágenes)
  //-- Step 1: Detect the keypoints using SURF Detector
  int minHessian = 400;

  cv::OrbFeatureDetector detector;
  std::vector< cv::KeyPoint > keypoints_object, keypoints_scene;

  detector.detect( image1, keypoints_object );
  detector.detect( image2, keypoints_scene  );

  // 3.- Calcular Descriptores (2A Key Points)
  //-- Step 2: Calculate descriptors (feature vectors)
  cv::BriefDescriptorExtractor extractor;

  cv::Mat descriptors_object, descriptors_scene;
  extractor.compute( image1, keypoints_object, descriptors_object );
  extractor.compute( image2, keypoints_scene,  descriptors_scene  );

  // 4.- Calcular Correspondencias (Filtrar)
  //-- Step 3: Matching descriptor vectors using FLANN matcher
  cv::BFMatcher matcher;
  std::vector< cv::DMatch > matchesBack;
  try{
    matcher.match( descriptors_object, descriptors_scene, matchesBack );
  } catch (cv::Exception e) {
    std::cerr << "Se ha detectado una exception al realizar el match: " + e.err << std::endl;
    return;
  }
  double max_dist = 0; 
  double min_dist = 100;

  //-- Quick calculation of max and min distances between keypoints
  for( int i = 0; i < descriptors_object.rows; i++ ) { 
      double dist = matchesBack[i].distance;
      if( dist < min_dist ) min_dist = dist;
      if( dist > max_dist ) max_dist = dist;
  }
  printf("-- Max dist : %f \n", max_dist );
  printf("-- Min dist : %f \n", min_dist );

  //-- Use only "good" matchesBack (i.e. whose distance is less than 3*min_dist )
  std::vector< cv::DMatch > good_matchesBack;

  int numMatches = 0;

  for( int i = 0; i < descriptors_object.rows; i++ ) { 
    if( matchesBack[i].distance < 2.5*min_dist ) {
        good_matchesBack.push_back( matchesBack[i]);
        numMatches++;
    }
  }

  if (numMatches < 24) {
    for( int i = 0; i < descriptors_object.rows; i++ ) { 
      if( matchesBack[i].distance < 0.45*max_dist ) {
          good_matchesBack.push_back( matchesBack[i]);
          numMatches++;
      }
    }
  }

  Mat img_matches;
  cv::drawMatches( image1, keypoints_object, image2, keypoints_scene,
  good_matchesBack, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
  std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

  cv::namedWindow("Resultado good_matches");
  cv::startWindowThread();
  cv::imshow( "Resultado good_matches", img_matches );

  std::vector< cv::Point2f > obj, scene;
  for( int i = 0; i < good_matchesBack.size(); i++ ){
      //-- Get the keypoints from the good matchesBack
      obj.push_back( keypoints_object[ good_matchesBack[i].queryIdx ].pt );
      scene.push_back( keypoints_scene[ good_matchesBack[i].trainIdx ].pt );
  }
  try {
    // 5.- Obtener Homografía
    std::cout << "--Homografia--" << std::endl;
    // Find the Homography Matrix
    cv::Mat H = findHomography( obj, scene, CV_RANSAC );
    // 6.- Aplicar transformación + Unir
    // Use the Homography Matrix to warp the images
    cv::Mat result;
    cv::warpPerspective(image1,result,H,cv::Size(image1.cols+image2.cols,image1.rows));
    cv::Mat half(result,cv::Rect(0,0,image2.cols,image2.rows));
    image2.copyTo(half);
    // 7.- Ver resultado
    cv::namedWindow("Resultado panoramica");
    cv::startWindowThread();
    cv::imshow( "Resultado panoramica", result );

    cv::waitKey(100);
  } catch (cv::Exception e) {
    std::cerr << "Se ha detectado una exception al realizar la Homografía: " + e.err << std::endl;
    return ;
  }
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
    if (panoramica)
      verCamaraDerecha(msg);
    cv_ptr_der = cv_bridge::toCvCopy(msg, msg->encoding);
  }
  catch (cv_bridge::Exception& e){
     ROS_ERROR("cv_bridge exception: %s", e.what());
     return;
  }
}

void giroEsquinaIzquierda(){
  //std::cout << "GIRANDO POR LA FUNCION ESQUINA IZQUIERDA!!" << std::endl;
  if(valueF<5 && valueF>1.5){
    rotateVel = 0.4;
    forwardVel = 0.25;
  }
  else{
    esquinaIzquierda = false;
  }
}

void darVueltaRobot(){
  //std::cout << "MI ORIENTACION ACTUAL ES: " << orientacion << ", Y LA ORIENTACION A LA QUE TENGO QUE LLEGAR ES: " << orientacionGiro << "\n";
  if(orientacion<orientacionGiro+0.2 && orientacion>orientacionGiro-0.2){
    camIncorrecto = false;
  }
  else{
    rotateVel = 0.4;
    forwardVel = 0;
  }
}

bool buscaCoincidencias(Mat descriptorRobot, Mat descriptorImagen){
  BFMatcher matcher(NORM_L2);
  vector<DMatch> matches;

  matcher.match(descriptorRobot, descriptorImagen, matches);

  double max_dist = 0; double min_dist = 100;

  std::vector< DMatch > good_matches;

  for( int i = 0; i < descriptorRobot.rows; i++ )
  { 
    double dist = matches[i].distance;
    if( dist < min_dist ) min_dist = dist;
    if( dist > max_dist ) max_dist = dist;
  }

  //printf("-- Max dist : %f \n", max_dist );
  //printf("-- Min dist : %f \n", min_dist );

  for( int i = 0; i < descriptorRobot.rows; i++ )
  { if( matches[i].distance < 3*min_dist && matches[i].distance < 0.125 )
    { good_matches.push_back( matches[i]); }
  }
  std::cout << "ESTAS BUSCANDO ESTOOOO!!!" << std::endl;
  std::cout << "good_matches: " << good_matches.size() << std::endl;
  if(good_matches.size()>4){
    //std::cout << "ES UN FUCKING ROBOT" << std::endl;
    std::cout << "good_matches: " << good_matches.size() << std::endl;
    return true;
  }
  else{
    return false;
  }
}

bool encuentraRobot(){
  // Detectar los keypoints
  SurfFeatureDetector detector(900);
  std::vector<KeyPoint> keypoints_2;
  //std::cout <<"PASO 1" << std::endl;
  detector.detect(cv_ptr_frontal->image, keypoints_2);
  //std::cout <<"PASO 2" << std::endl;
  // Calcular los descriptores de los keypoints
  if(keypoints_2.size()!=0){
    SurfDescriptorExtractor extractor;
    Mat descriptors2;
    //extractor.compute(img_back,keypoints_1,descriptorsBack);
    //std::cout <<"PASO 3" << std::endl;
    extractor.compute(cv_ptr_frontal->image,keypoints_2,descriptors2);
    //std::cout <<"PASO 4" << std::endl;

    // Buscar coincidencias en los descriptores
    BFMatcher matcher(NORM_L2);
    vector<DMatch> matchesBack, matchesLeft, matchesRight, matchesFront;

    if(buscaCoincidencias(descriptorsBack, descriptors2))
      return true;
    else if(buscaCoincidencias(descriptorsRight, descriptors2))
      return true;
    else if(buscaCoincidencias(descriptorsLeft, descriptors2))
      return true;
    else if(buscaCoincidencias(descriptorsFront, descriptors2))
      return true;
    else
      return false;
  }
  else{
    //std::cout << "BLABLABLABLABLABLABLABLABLABLA" << std::endl;
    return false;
  }
}

void procesaDatosLaser(const sensor_msgs::LaserScan::ConstPtr& msg){
  //std::cout << "HAGO procesaDatosLaser" << std::endl;
  if(debug){
    // Mínimo valor angular del láser -0.521568
    //ROS_INFO_STREAM("AngleMin: " << msg->angle_min);
    // Máximo valor angular del láser 0.524276
   // ROS_INFO_STREAM("AngleMax: " << msg->angle_max);
    // Incremento angular entre dos beams 0.00163669
    //ROS_INFO_STREAM("AngleIncrement: " << msg->angle_increment); 
    // Mínimo valor que devuelve el láser 0.45
   //ROS_INFO_STREAM("RangeMin: " << msg->range_min); 
    // Máximo valor que devuelve el láser. Valores por debajo y 
    // por encima de estos rangos no deben ser tenidos en cuenta 10
    //ROS_INFO_STREAM("RangeMax: " << msg->range_max); 
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
    if(i>=219 && i<419){
      if(!std::isnan(msg->ranges[i])){
        valueF += msg->ranges[i];
        contF++;
      }
    }
    // 50
    if(i>=354 && i<404){
      if(!std::isnan(msg->ranges[i])){
        valueFExt += msg->ranges[i];
        contFExt++;
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
  //ROS_INFO_STREAM("valueDeExt:" << valueDeExt << "; " << contDE); // Acceso a los valores de rango
  valueD /= contD;
  ROS_INFO_STREAM("ValueD:" << valueD << "; " << contD); // Acceso a los valores de rango
  valueF /= contF;
  ROS_INFO_STREAM("ValueF:" << valueF << "; " << contF); // Acceso a los valores de rango
  valueI /= contI;
  ROS_INFO_STREAM("ValueI:" << valueI << "; " << contI); // Acceso a los valores de rango
  valueIzExt /= contIE;
  valueFExt /= contFExt;
  std::cout << "valueFExtremo: " << valueFExt << std::endl;
  //std::cout << "valueIzExtAnt: " << valueIzExtAnt << std::endl;
  //ROS_INFO_STREAM("valueIzExt:" << valueIzExt << "; " << contIE); // Acceso a los valores de rango
  //std::cout << "valueDeExtAnt: " << valueDeExtAnt << std::endl;
  //ROS_INFO_STREAM("valueDeExt:" << valueDeExt << "; " << contDE); // Acceso a los valores de rango
  //std::cout << std::endl;

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
   
    if(!esquinaIzquierda){
      if(valueF==0.1)
        forwardVel = -0.5;
      else if(valueFExt==0.1)
        forwardVel = -0.3;
      else if((valueI < valueIzN+0.3 && valueI > valueIzN-0.3) && valueF > 1.1){
        std::cout << "SIGO RECTO" << std::endl;

        if(!turbo && tiempoTurboActivado>0 && tiempoTurboActivado<5)
          forwardVel = 0.9;
        else if(turbo && tiempoTurboActivado>0)
          forwardVel = 0.9;
        else{
          forwardVel = 0.5;
          turbo = false;
        }
        /**
        if(!turbo)
          forwardVel = 0.5;
        else
          forwardVel = 0.9;
          */
        rotateVel = 0;
      }
      else if(valueI > valueIzN+0.3 && valueF > 1.1){
        std::cout << "AVANZO PERO AJUSTANDO A LA IZQUIERDA" << std::endl;
        if(!turbo && tiempoTurboActivado>0 && tiempoTurboActivado<5)
          forwardVel = 0.9;
        else if(turbo && tiempoTurboActivado>0)
          forwardVel = 0.9;
        else{
          forwardVel = 0.5;
          turbo = false;
        }
        /**
        if(!turbo)
          forwardVel = 0.5;
        else
          forwardVel = 0.9;
        */
        rotateVel = 0.3;
      }
      else if(valueI < valueIzN-0.3 && valueF > 1.1){
        std::cout << "AVANZO PERO AJUSTANDO A LA DERECHA" << std::endl;
        if(!turbo && tiempoTurboActivado>0 && tiempoTurboActivado<5)
          forwardVel = 0.9;
        else if(turbo && tiempoTurboActivado>0)
          forwardVel = 0.9;
        else{
          forwardVel = 0.5;
          turbo = false;
        }
        /**
        if(!turbo)
          forwardVel = 0.5;
        else
          forwardVel = 0.9;
        */
        rotateVel = -0.3;
      }
      // Hasta aqui es para que siga la pared izquierda
      // Para que si esta muy cerca de la pared o de un obstaculo intente evitarlo
      
      else if(valueF < 1.1){
        if(cv_ptr_frontal != 0){
          if(tiempoTurboDesactivado > 5){
            if(encuentraRobot()){
              turbo = true;
              std::cout << "ESTOY VIENDO A UN ROBOT DELANTE A MENOS DE 1 METRO!!" << std::endl;
            }
          }
          else{
            turbo = false;
            forwardVel = 0.5;
          }
        }
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
      else forwardVel = 0.5;
    }
    else if(esquinaIzquierda){
      giroEsquinaIzquierda();
    }
    /**
    else if(turbo){
      navegacionTurbo();
    }
    */
    else{
      //std::cout << "NO ESTOY ACTUALIZANDO NADA" << std::endl;
    }
    valueIzExtAnt = valueIzExt;
    valueDeExtAnt = valueDeExt;
  }
}

void bucle(){
  ros::Rate rate(10);
  
  
  //std::cout << "Iniciando bucle para siempre" << std::endl;
  while (ros::ok()){
    if(inicio){
      long double tiemp = getTickCount();
      geometry_msgs::Twist base_cmd; // Este mensaje es el que se publicara para decir las velocidades linear y angular del robot
      base_cmd.linear.x = forwardVel; // Velocidad linear que tendra el colega
      base_cmd.angular.z = rotateVel; // Velocidad angular que tendra el colega

      //publish the assembled command
      cmd_vel_pub_.publish(base_cmd);

      ros::spinOnce(); // Se procesaran todas las llamadas que queden pendientes (como procesaDatosLaser)
      rate.sleep(); // Con esto esperara a que acabe el ciclo
      long double tFinal = (getTickCount()-tiemp)/getTickFrequency();

      if((tiempoTurboDesactivado>0 && tiempoTurboDesactivado<10) || turbo){
        tiempoTurboActivado -= tFinal;
        tiempoTurboDesactivado -= tFinal;
      }else if(tiempoTurboDesactivado<0){
        tiempoTurboDesactivado = 10;
        tiempoTurboActivado = 5;
      }
      std::cout << "tiempo turbo activado: " << tiempoTurboActivado << std::endl;
      std::cout << "tiempo turbo desactivado: " << tiempoTurboDesactivado << std::endl;
      std::cout << "Velocidad: " << forwardVel << std::endl;
      std::cout << "turbo: " << turbo << std::endl;
      std::cout << std::endl;


      //std::cout << "TIEMPOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO: " << tFinal << std::endl;
      if (panoramica)
        verPanoramica();
    }
  }

  if (debug) {
     ros::spin();
     ros::shutdown();
     cv::destroyWindow("view");
  }
}

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
  std::cout << "HAGO PRIMERO verCamaraFrontalNormalizada" << std::endl;
   cv::namedWindow("view");
   cv::startWindowThread();
   try {    
      //cv_ptr_frontal = cv_bridge::toCvShare(msg);
      cv_ptr_frontal = cv_bridge::toCvCopy(msg, msg->encoding);

      // imshow expects a float value to lie in [0,1], so we need to normalize
      // for visualization purposes.
      double max = 0.0;
      cv::minMaxLoc(cv_ptr_frontal->image, 0, &max, 0, 0);
      cv::Mat normalized;
      cv_ptr_frontal->image.convertTo(normalized, CV_32F, 1.0/max, 0);

      //cv::imshow("view", normalized);
      cv::waitKey(1);
   } catch (const cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
   }
}

};
/*

cv::Mat normalizaImagen(const cv::Mat& m){
  cv::Mat mat(m);
  double max = 0.0;
  cv::minMaxLoc(mat, 0, &max, 0, 0);
  cv::Mat normalized;
  mat.convertTo(normalized, CV_32F, 1.0/max, 0);
  normalized.convertTo(normalized, CV_8UC1);
  return normalized;
}
*/

cv::Mat normalizaImagen(const cv::Mat& m){
  cv::Mat mat;
  cv::normalize(m, mat, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  return mat;
}

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
