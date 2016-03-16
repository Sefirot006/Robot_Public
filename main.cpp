#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

const bool debug = true;

void verCamaraFrontal(const sensor_msgs::ImageConstPtr& msg);
void verCamaraFrontalNormalizada(const sensor_msgs::ImageConstPtr& msg);

class RobotDriver
{
private:

  double forwardVel;
  double rotateVel;
  double closestRange;
  double valNormalIz;
  double valNormalDe;
  char ultimoGiro;
    
  bool brecha;
  bool esquina;
  
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_vel_pub_;
  ros::Publisher scan_pub;

  ros::Subscriber laserSub;
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
    forwardVel = 0.0;
    rotateVel = 0.0;
    valNormalIz = 1.5;
    valNormalDe = 3;
    brecha = false;
    esquina = false;
    ultimoGiro = ' ';
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot2/commands/velocity", 1);
    //laserSub = nh.subscribe("base_scan", 1, &Wander::commandCallback, this);
    //scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 50);
    // Susctibe el metodo procesaDatosLaser al topico scan del robot1(que sera el laser(creo))
    // Este metodo sera llamado cada vez que el emisor publique datos
    laserSub = nh.subscribe("/robot2/scan", 1, &RobotDriver::procesaDatosLaser, this);
    frontRGBSub = nh.subscribe("/robot2/camera/rgb/image_raw", 1, &RobotDriver::procesaDatosMonofocal, this);
    rearRGB1Sub = nh.subscribe("/robot2/trasera1/trasera1/rgb/image_raw", 1, &RobotDriver::procesaDatosBifocalIzq, this);
    rearRGB2Sub = nh.subscribe("/robot2/trasera2/trasera2/rgb/image_raw", 1, &RobotDriver::procesaDatosBifocalDer, this);
    //imu = nh.subscribe("/robot2/imu_data", 1, &RobotDriver::commandImu, this);
  }
//sensor_msgs/Imu
  void commandImu(const sensor_msgs::Imu::ConstPtr& msg){
    std::cout << "IMU" << std::endl;
    //ROS_INFO_STREAM("Odometry x: " << msg->pose.pose.position.x); 
    //ROS_INFO_STREAM("Odometry y: " << msg->pose.pose.position.y); 
    //ROS_INFO_STREAM("Odometry angz: " << 2*atan2(msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
  }


  void procesaDatosMonofocal(const sensor_msgs::ImageConstPtr& msg){
      if (debug) {
         //verCamaraFrontal(msg);
         verCamaraFrontalNormalizada(msg);
      }
   }

   void procesaDatosBifocal(){
      //A partir de aqui
      //Se fusionan las imagenes de cv_ptr_izq y cv_ptr_der.
      //http://stackoverflow.com/questions/11134667/some-problems-on-image-stitching-homography?lq=1
      //FindContours
      //http://docs.opencv.org/3.1.0/d3/dc0/group__imgproc__shape.html#ga17ed9f5d79ae97bd4c7cf18403e1689a&gsc.tab=0
   }

   void procesaDatosBifocalIzq(const sensor_msgs::ImageConstPtr& msg){
      try {
         cv_ptr_izq = cv_bridge::toCvCopy(msg, msg->encoding);
      }
      catch (cv_bridge::Exception& e){
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
      }
   }

   void procesaDatosBifocalDer(const sensor_msgs::ImageConstPtr& msg){
      try {
         cv_ptr_der = cv_bridge::toCvCopy(msg, msg->encoding);
      }
      catch (cv_bridge::Exception& e){
         ROS_ERROR("cv_bridge exception: %s", e.what());
         return;
      }
   }

  void detectaBrecha(const double valueI,const double valueF, const double valueD){
    std::cout << "DETECTANDO BRECHA" << std::endl;
    forwardVel = 0.5;
    rotateVel = 0.15;
    if(valueF < 2 && valueI < 3)
      brecha = false;
  }

  void detectaObstaculo(const double valueI,const double valueF, const double valueD){
    forwardVel = 0.3;
    if(valueI < valueD){
      rotateVel = -0.5;
      ultimoGiro = 'i';
    }
    else{
      rotateVel = 0.5;
      ultimoGiro = 'd';
    }
  }

  void salirEsquina(const double valueI,const double valueF, const double valueD){
    if(ultimoGiro == 'i')
      rotateVel = -0.5;
    else
      rotateVel = 0.5;
    if(valueF > 1.5)
      esquina = false;
  }

  void procesaDatosLaser(const sensor_msgs::LaserScan::ConstPtr& msg){
    //std::cout << "probando" << std::endl;

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
    
    //std::cout << "AAAAAAAAAAAAAA: " << totalValues << std::endl;
    double valueI = 0.1, valueF = 0.1, valueD = 0.1;
    int contI = 1, contF = 1, contD = 1;
    for(int i=0;i<totalValues;++i){
      // 200
      if(i>=5 && i<205){
        if(!std::isnan(msg->ranges[i])){
          valueD += msg->ranges[i];
          contD++;
        }
      }
      // 130
      if(i>=228 && i<358){
        if(!std::isnan(msg->ranges[i])){
          valueF += msg->ranges[i];
          contF++;
        }
      }
      // 200
      if(i>=434 && i<634){
        if(!std::isnan(msg->ranges[i])){
          valueI += msg->ranges[i];
          contI++;
        }
      }
      //std::cout << "i " << i << ": " << msg->ranges[i] << std::endl;
    }
    valueD /= contD;
    ROS_INFO_STREAM("ValueD:" << valueD << "; " << contD); // Acceso a los valores de rango
    valueF /= contF;
    ROS_INFO_STREAM("ValueF:" << valueF << "; " << contF); // Acceso a los valores de rango
    valueI /= contI;
    ROS_INFO_STREAM("ValueI:" << valueI << "; " << contI); // Acceso a los valores de rango

    // rotateVel positivo gira a la izquierda
    // rotateVel negativo gira a la derecha

    //Prueba de dar media vuelta
    /**if(vuelta>0){
      rotateVel = 0.5;
      vuelta--;
    }
    else{
      rotateVel = 0;
    }*/

    // Pruebas de la navegacion
    //Intentando ir siempre en el centro del camino
    if(!esquina && !brecha){
      if(valueF==0.1 && valueI==0.1 && valueD==0.1){
        esquina = true;
        forwardVel = -0.4;
      }
      else if(valueF==0.1 && contF == 1 && contI != 1 && contD != 1)
        forwardVel = 0.5;
      else if(valueF > 2){
        forwardVel = 0.5;
        if(valueI < 2.6 && valueD > 3.2)
          rotateVel = -0.1;
        else if(valueD < 2.6 && valueI > 3.2)
          rotateVel = 0.1;
        else
          rotateVel = 0;
        if(valueI > 6)
          brecha = true;        
      }
      else{
        detectaObstaculo(valueI, valueF,valueD);
      }
    }
    else if(esquina)
      salirEsquina(valueI,valueF,valueD);
    else
      detectaBrecha(valueI, valueF,valueD);
  }

/**
  //! Loop forever while sending drive commands based on keyboard input
  bool driveKeyboard()
  {
    std::cout << "Type a command and then press enter.  "
      "Use '+' to move forward, 'l' to turn left, "
      "'r' to turn right, '.' to exit.\n";

    //we will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;

    char cmd[50];
    while(nh_.ok()){

      std::cin.getline(cmd, 50);
      if(cmd[0]!='+' && cmd[0]!='l' && cmd[0]!='r' && cmd[0]!='.')
      {
        std::cout << "unknown command:" << cmd << "\n";
        continue;
      }

      base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;   
      //move forward
      if(cmd[0]=='+'){
        base_cmd.linear.x = 0.25;
      } 
      //turn left (yaw) and drive forward at the same time
      else if(cmd[0]=='l'){
        base_cmd.angular.z = 0.75;
        base_cmd.linear.x = 0.25;
      } 
      //turn right (yaw) and drive forward at the same time
      else if(cmd[0]=='r'){
        base_cmd.angular.z = -0.75;
        base_cmd.linear.x = 0.25;
      } 
      //quit
      else if(cmd[0]=='.'){
        break;
      }

      //publish the assembled command
      cmd_vel_pub_.publish(base_cmd);
    }
    return true;
  }
*/

  void bucle(){
    ros::Rate rate(10);
    while (ros::ok()){
      geometry_msgs::Twist base_cmd; // Este mensaje es el que se publicara para decir las velocidades linear y angular del robot
      base_cmd.linear.x = forwardVel; // Velocidad linear que tendra el colega
      base_cmd.angular.z = rotateVel; // Velocidad angular que tendra el colega

      //publish the assembled command
      cmd_vel_pub_.publish(base_cmd);
      ros::spinOnce(); // Se procesaran todas las llamadas que queden pendientes (como procesaDatosLaser)
      rate.sleep(); // Con esto esperara a que acabe el ciclo
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
