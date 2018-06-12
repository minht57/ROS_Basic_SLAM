#include <stdio.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <stdint.h>

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

uint8_t u8_bufWrite[50];
int mainfd = -1;                                          
int n;
static geometry_msgs::Twist vel_;
std::string port_id;

int open_port(const char* port)
{
    int fd;                            

    fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);

    if (fd == -1)
    {                                              
        ROS_ERROR("Unable to open port %s", port);
        return -1;
    }

    ROS_INFO("Serial Port initialized: %s - id: %d", port, fd);
    return (fd);
}

//void write_callback(const std_msgs::String::ConstPtr& msg){
    //ROS_INFO_STREAM("Writing to serial port" << msg->data);
    //ser.write(msg->data);
    //ROS_INFO("Data: [%s]", msg->data.c_str());
//}

void velCallback(const geometry_msgs::Twist::ConstPtr& vel){
    static int cnt = 0;
    // if((vel_.linear.x != vel->linear.x) || (vel_.angular.z != vel->angular.z))
    if ((vel_.linear.x != vel->linear.x) ||
            (vel_.angular.z != vel->angular.z))
    {
        cnt = 0;
        ROS_INFO("Linear: [%.3f] | Angular: [%.3f]", vel->linear.x, vel->angular.z);
        sprintf((char *)u8_bufWrite, "[%.3f,%.3f,]",vel->linear.x, vel->angular.z);
        write (mainfd, (char *) u8_bufWrite, strlen((char *) u8_bufWrite));
    }
    else if((vel->linear.x == 0) && (vel->angular.z == 0))
    {
        if(cnt > 10)
        {
            cnt = 0;
            ROS_INFO("Linear: [%.3f] | Angular: [%.3f]", vel->linear.x, vel->angular.z);
            sprintf((char *)u8_bufWrite, "[%.3f,%.3f]",vel->linear.x, vel->angular.z);
            write (mainfd, (char *) u8_bufWrite, strlen((char *) u8_bufWrite));
        }
        cnt++;
    }
    else if ((vel_.linear.x == vel->linear.x) ||
            (vel_.angular.z == vel->angular.z))
    {
        if(cnt > 3)
        {
            cnt = 0;
            ROS_INFO("Linear: [%.3f] | Angular: [%.3f]", vel->linear.x, vel->angular.z);
            sprintf((char *)u8_bufWrite, "[%.3f,%.3f,]",vel->linear.x, vel->angular.z);
            write (mainfd, (char *) u8_bufWrite, strlen((char *) u8_bufWrite));
        }
        cnt++;
    }
    vel_.linear.x = vel->linear.x;
    vel_.angular.z = vel->angular.z;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "serial_node");
    ros::NodeHandle nh;

    ros::Subscriber write_sub = nh.subscribe("cmd_vel", 100, velCallback);
    ros::Publisher read_pub = nh.advertise<std_msgs::String>("read", 1000);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_odom", 1000);

    geometry_msgs::Twist vel_odom;
    char buf[50];
    char rc_buf;
    int8_t flag_rc = 0;
    int8_t idx_rc = 0;
    char* cToken;
    static int cnt_resend = 0;
    struct termios options;

    nh.param<std::string>("port_id",port_id,"/dev/ttyUSB0");
    mainfd = open_port(port_id.c_str());

    if(mainfd == -1)
    {
        ROS_ERROR("Exit node");
        return -1;
    }

    fcntl(mainfd, F_SETFL, FNDELAY);                  
                                     
    tcgetattr(mainfd, &options);
    cfsetispeed(&options, B115200);                
    cfsetospeed(&options, B115200);

                                   
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB; 
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |=  CS8;                              
    options.c_cflag &= ~CRTSCTS;               /* Disable hardware flow control */  

                                 
    options.c_lflag &= ~(ICANON | ECHO | ISIG);

                                        
    tcsetattr(mainfd, TCSANOW, &options);

    ros::Rate loop_rate(10);

    while(ros::ok()){

        ros::spinOnce();
        n = read(mainfd, &rc_buf, 1);
        if(n > 0)
        {
            if(rc_buf == ']')
            {
                flag_rc = 0;
                cToken = strtok(buf, ",");
                if(cToken != NULL)
                {
                    vel_odom.linear.x = atof(cToken);
                    cToken = strtok(NULL,",");
                    if(cToken != NULL)
                    {
                        vel_odom.angular.z = atof(cToken);
                        ROS_INFO("Data: [%f] [%f]", vel_odom.linear.x, vel_odom.angular.z);
                        cmd_pub.publish(vel_odom);
                    }
                }
            }
            if(flag_rc)
            {
                buf[idx_rc++] = rc_buf;
            }
            if(rc_buf == '[')
            {
                flag_rc = 1;
                idx_rc = 0;
            }
        }
        else
        {
            //ROS_INFO("Wait ...");
        }
        /*if((vel_.linear.x == 0) && (vel_.angular.z == 0))
	    {
	        if(cnt_resend > 20)
	        {
	            cnt_resend = 0;
	            sprintf((char *)u8_bufWrite, "[%f,%f]",vel_.linear.x, vel_.angular.z);
	            ROS_INFO("No data -> Resend: Linear: [%.3f] | Angular: [%.3f]", vel_.linear.x, vel_.angular.z);
	            write (mainfd, (char *) u8_bufWrite, strlen((char *) u8_bufWrite));
	        }
	        cnt_resend++;
	        loop_rate.sleep();
	    }*/
        //loop_rate.sleep();
    }

//  uint8_t data;
//  char *p;
//  data = (int) strtol(argv[1],&p,10 );
  //data=0b10000011;
//  write (mainfd, &data, 1); 
//  printf("Sending: %i\n", data);                                               
    close(mainfd);
    return 0;
}

