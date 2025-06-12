#ifndef bot_serial
#define bot_serial
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <string>
#include <stdio.h>
#include "geometry_msgs/msg/twist.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

//宏定义
#define SEND_DATA_CHECK   1          //Send data check flag bits //发送数据校验标志位
#define READ_DATA_CHECK   0          //Receive data to check flag bits //接收数据校验标志位
#define FRAME_HEADER      0X7B       //Frame head //帧头
#define FRAME_TAIL        0X7D       //Frame tail //帧尾
#define RECEIVE_DATA_SIZE 24         //The length of the data sent by the lower computer //下位机发送过来的数据的长度
#define SEND_DATA_SIZE    11         //The length of data sent by ROS to the lower machine //ROS向下位机发送的数据的长度
#define PI 				  3.1415926f //PI //圆周率
using std::placeholders::_1;
const double odom_pose_covariance[36]   = {1e-3,    0,    0,   0,   0,    0, 
  0, 1e-3,    0,   0,   0,    0,
  0,    0,  1e6,   0,   0,    0,
  0,    0,    0, 1e6,   0,    0,
  0,    0,    0,   0, 1e6,    0,
  0,    0,    0,   0,   0,  1e3 };

const double odom_pose_covariance2[36]  = {1e-9,    0,    0,   0,   0,    0, 
  0, 1e-3, 1e-9,   0,   0,    0,
  0,    0,  1e6,   0,   0,    0,
  0,    0,    0, 1e6,   0,    0,
  0,    0,    0,   0, 1e6,    0,
  0,    0,    0,   0,   0, 1e-9 };

const double odom_twist_covariance[36]  = {1e-3,    0,    0,   0,   0,    0, 
  0, 1e-3,    0,   0,   0,    0,
  0,    0,  1e6,   0,   0,    0,
  0,    0,    0, 1e6,   0,    0,
  0,    0,    0,   0, 1e6,    0,
  0,    0,    0,   0,   0,  1e3 };
  
const double odom_twist_covariance2[36] = {1e-9,    0,    0,   0,   0,    0, 
  0, 1e-3, 1e-9,   0,   0,    0,
  0,    0,  1e6,   0,   0,    0,
  0,    0,    0, 1e6,   0,    0,
  0,    0,    0,   0, 1e6,    0,
  0,    0,    0,   0,   0, 1e-9} ;
typedef struct _RECEIVE_DATA_     
{
    uint8_t rx[RECEIVE_DATA_SIZE];
    uint8_t Flag_Stop;
    unsigned char Frame_Header;
    float X_speed;  
    float Y_speed;  
    float Z_speed;  	
    unsigned char Frame_Tail;
}RECEIVE_DATA;
//ROS向下位机发送数据的结构体
typedef struct _SEND_DATA_  
{
	  uint8_t tx[SEND_DATA_SIZE];
		float X_speed;	       
		float Y_speed;           
		float Z_speed;         
		unsigned char Frame_Tail; 
}SEND_DATA;
typedef struct __Vel_Pos_Data_
{
	float X;
	float Y;
	float Z;
}Vel_Pos_Data;
class turn_on_robot : public rclcpp::Node
{
	public:
		turn_on_robot();  //Constructor //构造函数
		~turn_on_robot(); //Destructor //析构函数
		void Control();   //Loop control code //循环控制代码
		serial::Serial Stm32_Serial; //Declare a serial object //声明串口对象
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_; 
	private:
		rclcpp::Time _Now, _Last_Time;  //Time dependent, used for integration to find displacement (mileage) //时间相关，用于积分求位移(里程)
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr Cmd_Vel_Sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher; 
    //Initialize the topic subscriber //初始化话题订阅者
    float Sampling_Time;         //Sampling time, used for integration to find displacement (mileage) //采样时间，用于积分求位移(里程)
    Vel_Pos_Data Robot_Vel;
    Vel_Pos_Data Robot_Pos;
    RECEIVE_DATA Receive_Data; //The serial port receives the data structure //串口接收数据结构体
    SEND_DATA Send_Data;       //The serial port sends the data structure //串口发送数据结构体 
    bool Get_Sensor_Data_New();
    unsigned char Check_Sum(unsigned char Count_Number,unsigned char mode);
    void Cmd_Vel_Callback(const geometry_msgs::msg::Twist::SharedPtr twist_aux);
    void Publish_Odom();      //Pub the speedometer topic //发布里程计话题 
    short IMU_Trans(uint8_t Data_High,uint8_t Data_Low);
    float Odom_Trans(uint8_t Data_High,uint8_t Data_Low);
 
    };
short turn_on_robot::IMU_Trans(uint8_t Data_High,uint8_t Data_Low)
{
  short transition_16;
  transition_16 = 0;
  transition_16 |=  Data_High<<8;   
  transition_16 |=  Data_Low;
  return transition_16;     
}
float turn_on_robot::Odom_Trans(uint8_t Data_High,uint8_t Data_Low)
{
  float data_return;
  short transition_16;
  transition_16 = 0;
  transition_16 |=  Data_High<<8;  //Get the high 8 bits of data   //获取数据的高8位
  transition_16 |=  Data_Low;      //Get the lowest 8 bits of data //获取数据的低8位
  data_return   =  (transition_16 / 1000)+(transition_16 % 1000)*0.001; // The speed unit is changed from mm/s to m/s //速度单位从mm/s转换为m/s
  return data_return;
}
unsigned char turn_on_robot::Check_Sum(unsigned char Count_Number,unsigned char mode)
{
    unsigned char check_sum=0,k;
    if(mode==0) //Receive data mode 
    {
        for(k=0;k<Count_Number;k++)
        {
            check_sum=check_sum^Receive_Data.rx[k]; //By bit or by bit //按位异或
        }
    }
    if(mode==1) //Send data mode 
    {
        for(k=0;k<Count_Number;k++)
        {
            check_sum=check_sum^Send_Data.tx[k]; //By bit or by bit //按位异或
        }
    }
    return check_sum; //Returns the bitwise XOR result //返回按位异或结果
}
bool turn_on_robot::Get_Sensor_Data_New()
{
 // short transition_16=0; //Intermediate variable //中间变量
  //uint8_t i=0;
  uint8_t check=0, error=1,Receive_Data_Pr[1]; //Temporary variable to save the data of the lower machine //临时变量，保存下位机数据
  static int count; //Static variable for counting //静态变量，用于计数
  Stm32_Serial.read(Receive_Data_Pr,sizeof(Receive_Data_Pr)); //Read the data sent by the lower computer through the serial port //通过串口读取下位机发送过来的数据

  /*//View the received raw data directly and debug it for use//直接查看接收到的原始数据，调试使用
  ROS_INFO("%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x",
  Receive_Data_Pr[0],Receive_Data_Pr[1],Receive_Data_Pr[2],Receive_Data_Pr[3],Receive_Data_Pr[4],Receive_Data_Pr[5],Receive_Data_Pr[6],Receive_Data_Pr[7],
  Receive_Data_Pr[8],Receive_Data_Pr[9],Receive_Data_Pr[10],Receive_Data_Pr[11],Receive_Data_Pr[12],Receive_Data_Pr[13],Receive_Data_Pr[14],Receive_Data_Pr[15],
  Receive_Data_Pr[16],Receive_Data_Pr[17],Receive_Data_Pr[18],Receive_Data_Pr[19],Receive_Data_Pr[20],Receive_Data_Pr[21],Receive_Data_Pr[22],Receive_Data_Pr[23]);
  */  

  Receive_Data.rx[count] = Receive_Data_Pr[0]; //Fill the array with serial data //串口数据填入数组


  Receive_Data.Frame_Header = Receive_Data.rx[0]; //The first part of the data is the frame header 0X7B //数据的第一位是帧头0X7B
  Receive_Data.Frame_Tail = Receive_Data.rx[23];  //The last bit of data is frame tail 0X7D //数据的最后一位是帧尾0X7D

    if(Receive_Data_Pr[0] == FRAME_HEADER || count>0) //Ensure that the first data in the array is FRAME_HEADER //确保数组第一个数据为FRAME_HEADER
        count++;
    else 
        count=0;
    if(count == 24) //
    {
        count=0;  //Prepare for the serial port data to be refill into the array //为串口数据重新填入数组做准备
        if(Receive_Data.Frame_Tail == FRAME_TAIL) //Verify the frame tail of the packet //验证数据包的帧尾
        {
            check=Check_Sum(22,READ_DATA_CHECK);  //BCC check passes or two packets are interlaced //BCC校验通过或者两组数据包交错

          if(check == Receive_Data.rx[22])  
          {
              error=0;  //XOR bit check successful //异或位校验成功
              //printf("successfully!");
          }
          if(error == 0)
          {
             Receive_Data.Flag_Stop=Receive_Data.rx[1]; //set aside //预留位
             Robot_Vel.X = Odom_Trans(Receive_Data.rx[2],Receive_Data.rx[3]); 
             //Get the speed of the moving chassis in the X direction 
             Robot_Vel.Y = Odom_Trans(Receive_Data.rx[4],Receive_Data.rx[5]); 
             //Get the speed of the moving chassis in the Y direction, 
             Robot_Vel.Z = Odom_Trans(Receive_Data.rx[6],Receive_Data.rx[7]);
              //Get the speed of the moving chassis in the Z direction   
            return true;
        }
    }
  }
  return false;
}
turn_on_robot::~turn_on_robot()
{
   //对象turn_on_robot结束前向下位机发送停止运动命令
  Send_Data.tx[0]=FRAME_HEADER;
  Send_Data.tx[1] = 0;  
  Send_Data.tx[2] = 0; 

  //The target velocity of the X-axis of the robot //机器人X轴的目标线速度 
  Send_Data.tx[4] = 0;     
  Send_Data.tx[3] = 0;  

  //The target velocity of the Y-axis of the robot //机器人Y轴的目标线速度 
  Send_Data.tx[6] = 0;
  Send_Data.tx[5] = 0;  

  //The target velocity of the Z-axis of the robot //机器人Z轴的目标角速度 
  Send_Data.tx[8] = 0;  
  Send_Data.tx[7] = 0;    
  Send_Data.tx[9]=Check_Sum(9,SEND_DATA_CHECK); //Check the bits for the Check_Sum function //校验位，规则参见Check_Sum函数
  Send_Data.tx[10]=FRAME_TAIL; 
  try
  {
    Stm32_Serial.write(Send_Data.tx,sizeof (Send_Data.tx)); //Send data to the serial port //向串口发数据  
  }
  catch (serial::IOException& e)   
  {
    RCLCPP_ERROR(this->get_logger(),"Unable to send data through serial port"); //If sending data fails, an error message is printed //如果发送数据失败,打印错误信息
  }
  Stm32_Serial.close(); //Close the serial port //关闭串口  
  RCLCPP_INFO(this->get_logger(),"Shutting down"); //Prompt message //提示信息
}
#endif