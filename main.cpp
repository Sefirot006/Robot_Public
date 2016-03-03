#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

class RobotDriver
{
private:

  double forwardVel;
  double rotateVel;
  double closestRange;
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue commands
  ros::Publisher cmd_vel_pub_;

  ros::Publisher scan_pub;

  ros::Subscriber laserSub;

public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh)
  {
    forwardVel = 0.5;
    rotateVel = 0.0;
    nh_ = nh;
    //set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/robot1/commands/velocity", 1);
    //laserSub = nh.subscribe("base_scan", 1, &Wander::commandCallback, this);
    //scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 50);
    // Susctibe el metodo procesaDatosLaser al topico scan del robot1(que sera el laser(creo))
    // Este metodo sera llamado cada vez que el emisor publique datos
    laserSub = nh.subscribe("/robot1/scan", 1, &RobotDriver::procesaDatosLaser, this);
  }

  void procesaDatosLaser(const sensor_msgs::LaserScan::ConstPtr& msg){
    //std::cout << "probando" << std::endl;

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
    
    // TODO A partir de aqui hay que cambiarlo
    /**double value1 = 0,value2 = 0,value3 = 0;
    for (int i=0; i< totalValues; i++) {
      if(i<440 && i>=100)
        value3 += msg->ranges[i];
      if(i<600 && i>=400)
        value2 += msg->ranges[i];
      if(i>=600 && i<940)
        value1 += msg->ranges[i];
      //ROS_INFO_STREAM("Values[" << i << "]:" << msg->ranges[i]); // Acceso a los valores de rango
    }
    value1 /= 340;
    ROS_INFO_STREAM("Value1:" << value1); // Acceso a los valores de rango
    value2 /= 200;
    ROS_INFO_STREAM("Value2:" << value2); // Acceso a los valores de rango
    value3 /= 340;
    ROS_INFO_STREAM("Value3:" << value3); // Acceso a los valores de rango

    if(value2 < 1.5){
      forwardVel = -0.2;
      if(value1 < 2){
        rotateVel = 0.4;
      }
      else if(value3 < 2){
        rotateVel = -0.4;
      }
    }
    else if(value2 < 2){
      forwardVel = 0.5;
    }
    else{
      forwardVel = 0.5;
    }
    if(value1 < 2){
      rotateVel = -0.4;
      if(value3 < 0.5){
        rotateVel = 0.4;
      }
    }
    else if(value2 < 2){
      rotateVel = 0.4;
      if(value1 < 0.5){
        rotateVel = -0.4;
      }
    }
    else{
      rotateVel = 0;
    }
    */
  }

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
  }

};

int main(int argc, char** argv)
{
  //init the ROS node
  ros::init(argc, argv, "robot_driver");
  ros::NodeHandle nh;

  RobotDriver driver(nh);
  //driver.driveKeyboard();
  driver.bucle();
  return 0;
}
