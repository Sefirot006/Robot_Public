#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

const bool debug = true;

void verCamaraFrontal(const sensor_msgs::ImageConstPtr& msg);

class RobotDriver
{
private:
   double forwardVel;
   double rotateVel;
   double closestRange;
   double valNormalIz;
   double valNormalDe;
   bool brecha;

   //! The node handle we'll be using
   ros::NodeHandle nh_;
   //! We will be publishing to the "/base_controller/command" topic to issue commands
   ros::Publisher cmd_vel_pub_;

   ros::Publisher scan_pub;

   ros::Subscriber laserSub;
   ros::Subscriber frontRGBSub;
   ros::Subscriber rearRGB1Sub;
   ros::Subscriber rearRGB2Sub;

public:
   //! ROS node initialization
   RobotDriver(ros::NodeHandle &nh){
      nh_ = nh;
      forwardVel = 0.0;
      rotateVel = 0.0;
      valNormalIz = 1.5;
      valNormalDe = 3;
      brecha = false;
      // Set up the publisher for the cmd_vel topic
      cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot2/commands/velocity", 1);
      // Susctibe el metodo procesaDatosLaser al topico scan del robot1(que sera el laser(creo))
      // Este metodo sera llamado cada vez que el emisor publique datos
      laserSub = nh.subscribe("/robot2/scan", 1, &RobotDriver::procesaDatosLaser, this);
      frontRGBSub = nh.subscribe("/robot2/camera/rgb/image_raw", 1, &RobotDriver::procesaDatosMonofocal, this);
      rearRGB1Sub = nh.subscribe("/robot2/trasera1/trasera1/rgb/image_raw", 1, &RobotDriver::procesaDatosBifocal, this);
      rearRGB2Sub = nh.subscribe("/robot2/trasera2/trasera2/rgb/image_raw", 1, &RobotDriver::procesaDatosBifocal, this);
   }

   void giraI(){
      rotateVel = 0.5;
      brecha = false;
   }

   void giraD(){
      rotateVel = -0.5;
      brecha = false;
   }

   void brechaDetectada(const double valueI,const double valueF, const double valueD){
      if(valueI<valNormalIz && valueD<valNormalDe){
         if(valueI < valueD)
            giraI();
         else
            giraD();
      }
   }

   void procesaDatosMonofocal(const sensor_msgs::ImageConstPtr& msg){
      if (debug)
         verCamaraFrontal(msg);

      
   }



   void procesaDatosBifocal(const sensor_msgs::ImageConstPtr& msg){
      
   }

   void procesaDatosLaser(const sensor_msgs::LaserScan::ConstPtr& msg){
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
      // Me esta dando 639 valores del laser
      int totalValues = ceil((msg->angle_max-msg->angle_min)/msg->angle_increment); // Total de valores que devuelve el láser
      
      //std::cout << "totalValues: " << totalValues << std::endl;
      double valueI = 0.1, valueF = 0.1, valueD = 0.1;
      int contI = 1, contF = 1, contD = 1;
      for(int i=0;i<totalValues;++i){
         if(i>=18 && i<190){
            if(!std::isnan(msg->ranges[i])){
               valueD += msg->ranges[i];
               contD++;
            }
         }
         if(i>=218 && i<318){
            if(!std::isnan(msg->ranges[i])){
               valueF += msg->ranges[i];
               contF++;
            }
         }
         if(i>=346 && i<518){
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
      
      // Pruebas de la navegacion
      // Intento de que se enderece siempre hacia adelante
      // Si no hay nada delante avanza sin problemas
      if(valueF==0.1 && contF == 1)
         forwardVel = 0.5;
      else if(valueF > 2){
         forwardVel = 0.5;
         if(valueD < 5.5 && valueD > 3.5)
            rotateVel = -0.1;
         if(valueI < 1.3 && valueI > 0.8)
            rotateVel = 0.1;
      }

      /**
      if(!brecha){
         // Si no hay nada delante avanza sin problemas
         if(valueF==0.1 && contF == 1)
            forwardVel = 0.5;
         else if(valueF > 2){
            forwardVel = 0.5;
            if(valueI < 1.7)
               rotateVel = 0.25;
            else if(valueD < 6.3)
               rotateVel = -0.25;
            // Estariamos detectando una brecha en el muro
            else if(valueI < valNormalIz || valueD < valNormalDe){
               brecha = true;
               brechaDetectada(valueI,valueF,valueD);
            }
            else
               rotateVel = 0;
         }
         // Tenemos un obstaculo por delante
         else
            forwardVel = 0;
      }
      else
         brechaDetectada(valueI,valueF,valueD);
      
      */
      // Codigo del wander aplicado a esto
      /**
      if(valueF < 0.4){
         forwardVel = -0.2;
         if(valueD < 1.2)
            rotateVel = -0.25;
         else if(valueI < 1.2)
            rotateVel = 0.25;
      }
      else if(valueF < 0.5)
         forwardVel = 0.3;
      else{
         forwardVel = 0.5;
      }
      if(valueD < 1.2){
         rotateVel = 0.25;
         if(valueI < 0.5)
            rotateVel = -0.25;
      }
      else if(valueI < 1.2){
         rotateVel = -0.25;
         if(valueD < 0.5)
            rotateVel = 0.25;
      }
      else rotateVel = 0;
      */
   }

/**
   //! Loop forever while sending drive commands based on keyboard input
   bool driveKeyboard()
   {
      std::cout << "Type a command and then press enter.   "
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
   try { 
      cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
      cv::waitKey(30);
   }
   catch (cv_bridge::Exception& e) {
      ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
   }
}

int main(int argc, char** argv){   //init the ROS node
   ros::init(argc, argv, "robot_driver");
   ros::NodeHandle nh;

   RobotDriver driver(nh);

   if (debug) {
      cv::namedWindow("view");
      cv::startWindowThread();
      image_transport::ImageTransport it(nh);
      image_transport::Subscriber subAux = it.subscribe("robot1/camera/rgb/image_raw", 1, verCamaraFrontal);
   }

   //driver.driveKeyboard();
   driver.bucle();
   return 0;
}
