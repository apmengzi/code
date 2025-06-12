#include "bot_serial.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    turn_on_robot Robot_Control;//Instantiate an object //实例化一�?对象
    Robot_Control.Control();
    rclcpp::shutdown();
    return 0;
}
void turn_on_robot::Control()
{ 
    _Last_Time=rclcpp::Node::now();
    while(rclcpp::ok())
    {
        try
        {
             _Now = rclcpp::Node::now();
             Sampling_Time = (_Now - _Last_Time).seconds(); 
             if (true == Get_Sensor_Data_New()) 
             {
                printf("new!");
                 Robot_Pos.X+=(Robot_Vel.X * cos(Robot_Pos.Z) - Robot_Vel.Y * sin(Robot_Pos.Z)) * Sampling_Time;
            //     // 计算X方向的位移，单位：m
                 Robot_Pos.Y+=(Robot_Vel.X * sin(Robot_Pos.Z) + Robot_Vel.Y * cos(Robot_Pos.Z)) * Sampling_Time; 
            //    //计算Y方向的位移，单位：m
                 Robot_Pos.Z+=Robot_Vel.Z * Sampling_Time; 
            //     //绕Z轴的角位移，单位：rad 
                 Publish_Odom();      
                 _Last_Time = _Now;
             }
            rclcpp::spin_some(this->get_node_base_interface());
        }
        catch (const rclcpp::exceptions::RCLError & e )
        {
            RCLCPP_ERROR(this->get_logger(),"unexpectedly failed whith %s",e.what());	
        }
    }
}
turn_on_robot::turn_on_robot():rclcpp::Node ("wheeltec_robot")
{
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    Cmd_Vel_Sub = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 2, std::bind(&turn_on_robot::Cmd_Vel_Callback, this, _1));
     RCLCPP_INFO(this->get_logger(),"wheeltec_robot Data ready"); //Prompt message //提示信息
     odom_publisher= create_publisher<nav_msgs::msg::Odometry>("odom",2);
     try
    { 

        Stm32_Serial.setPort("/dev/ttyACM0"); //Select the serial port number to enable //选择要开�?的串口号
       
        Stm32_Serial.setBaudrate(115200); //Set the baud rate //设置波特�?
        serial::Timeout _time = serial::Timeout::simpleTimeout(2000); //Timeout //超时等待
        Stm32_Serial.setTimeout(_time);
        Stm32_Serial.open(); //Open the serial port //开�?串口
    }
    catch (serial::IOException& e)
    {
        std::cerr<<e.what()<<std::endl;
        RCLCPP_ERROR(this->get_logger(),"wheeltec_robot can not open serial port,Please check the serial port cable! "); //If opening the serial port fails, an error message is printed //如果开�?串口失败，打印错�?信息
    }
    if(Stm32_Serial.isOpen())
    {
        RCLCPP_INFO(this->get_logger(),"wheeltec_robot serial port opened"); //Serial port opened successfully //串口开�?成功提示
    }
    
}
void turn_on_robot::Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux)
{
  Send_Data.tx[0]=FRAME_HEADER; //frame head 0x7B 
  Send_Data.tx[2] = 0; //set aside //预留�?
  printf("%f:%f:%f", twist_aux->linear.x,twist_aux->linear.x,twist_aux->angular.z);
  RCLCPP_INFO(this->get_logger(),"cmd is ready"); 
  //The target velocity of the X-axis of the robot
  //机器人x轴的�?标线速度,请填写代�?
  short temp=0;
  temp=twist_aux->linear.x*1000;
  Send_Data.tx[4]=temp;
  Send_Data.tx[3]=temp>>8;


  //The target velocity of the Y-axis of the robot
  //机器人y轴的�?标线速度，�?�填写代�?
  temp=0;
  temp=twist_aux->linear.y*1000;
  Send_Data.tx[6]=temp;
  Send_Data.tx[5]=temp>>8;
  

  //The target angular velocity of the robot's Z axis
  //机器人z轴的�?标�?�速度请填写代�?

  temp=0;
  temp=twist_aux->angular.z*1000;
  Send_Data.tx[8]=temp;
  Send_Data.tx[7]=temp>>8;

  Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //For the BCC check bits, see the Check_Sum function 
  Send_Data.tx[10]=FRAME_TAIL; //frame tail 0x7D
  try
  {
    Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); 
  }
  catch (serial::IOException& e)   
  {
    RCLCPP_ERROR(this->get_logger(),("Unable to send data through serial port")); 
  }
}

void turn_on_robot::Publish_Odom()
{
    //Convert the Z-axis rotation Angle into a quaternion for expression 
    //把Z轴转角转�?为四元数进�?�表�?
    tf2::Quaternion q;
    q.setRPY(0,0,Robot_Pos.Z);
    geometry_msgs::msg::Quaternion odom_quat=tf2::toMsg(q);
    geometry_msgs::msg::TransformStamped odom_tf;
    nav_msgs::msg::Odometry odom; //Instance the odometer topic data //实例化里程�?�话题数�?
    odom.header.stamp = rclcpp::Node::now(); ; 
    odom.header.frame_id = "odom"; // Odometer TF parent coordinates //里程�?TF父坐�?
    odom.pose.pose.position.x = Robot_Pos.X; //Position //位置
    odom.pose.pose.position.y = Robot_Pos.Y;
    odom.pose.pose.position.z = Robot_Pos.Z;
    odom.pose.pose.orientation = odom_quat; //Posture, Quaternion converted by Z-axis rotation //姿态，通过Z轴转角转换的四元�?

    odom.child_frame_id = "base_footprint"; // Odometer TF subcoordinates //里程�?TF子坐�?
    odom.twist.twist.linear.x =  Robot_Vel.X; //Speed in the X direction //X方向速度
    odom.twist.twist.linear.y =  Robot_Vel.Y; //Speed in the Y direction //Y方向速度
    odom.twist.twist.angular.z = Robot_Vel.Z; //Angular velocity around the Z axis //绕Z轴�?�速度        
    if(Robot_Vel.X== 0&&Robot_Vel.Y== 0&&Robot_Vel.Z== 0)
    //If the velocity is zero, it means that the error of the encoder will be relatively small, and the data of the encoder will be considered more reliable
    //如果velocity�?零，说明编码器的�?�?会比较小，�?�为编码器数�?更可�?
    {
        memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2));
        memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
    
    }
     else
    //If the velocity of the trolley is non-zero, considering the sliding error that may be brought by the encoder in motion, the data of IMU is considered to be more reliable
    //如果小车velocity非零，考虑到运动中编码器可能带来的滑动�?�?，�?�为imu的数�?更可�?
   {
    memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance));
    memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));  
 
    }
    odom_publisher->publish(odom); //Pub odometer topic //发布里程计话�?

    odom_tf.header.stamp = odom.header.stamp;
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_footprint";
    odom_tf.transform.translation.x = Robot_Pos.X;
    odom_tf.transform.translation.y = Robot_Pos.Y;
    odom_tf.transform.translation.z = 0.0; // 2D 机器人通常 z=0
    odom_tf.transform.rotation = odom_quat;
    tf_broadcaster_->sendTransform(odom_tf);
}