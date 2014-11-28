/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Jose Prado on 31/10/2014
*********************************************************************/

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <metal_detector_msgs/Coil.h>
#include <metal_detector_msgs/SetCoilsZero.h>
#include <cereal_port/CerealPort.h>
#include <sensor_msgs/Imu.h>
#include <iostream>
#include <fstream>
#include <string>
#include "math.h"

#define MIDDLE_COIL 	0

struct coil
{
	int channel[3];
	int zero[3];
};

coil coils[3];

tf::TransformBroadcaster * tf_broadcaster_ptr;
ros::Publisher * coil_pub_ptr;
ros::Publisher * gy80_pub_ptr;
//ros::Publisher * coil_pub_ptrR;
std::string frame_id;

/*  Data Format is 38 bytes lenght:
	String idx		Data 
	0	    		1 byte header - char '$'
	1,2  			2 bytes metal detector amount of metal - unsigned short
	3,4   		 	2 bytes magnetometer X - signed short - small endian
	5,6	   			2 bytes magnetometer Z - signed short - small endian //yes Z and Y are inverted for mag only
	7,8				2 bytes magnetometer Y - signed short - small endian
	9,10			2 bytes accelerometer X - signed short - small endian
	11,12			2 bytes accelerometer Y - signed short - small endian
	13,14			2 bytes accelerometer Z - signed short - small endian
	15,16			2 bytes gyros X - signed short - small endian
	17,18			2 bytes gyros Y - signed short - small endian
	19,20			2 bytes gyros Z - signed short - small endian
	21,22,23,24 	4 bytes roll - float
	25,26,27,28 	4 bytes pitch - float
	29,30,31,32 	4 bytes yaw - float //yaw optimizado usando roll e pitch
	33,34,35,36 	4 bytes yawU - float //yaw basico, so funciona bem no plano
	37      		1 byte footer - char '\n'
*/
void streamCallback(std::basic_string<char>* msg)
{
    fprintf(stderr,".");
	if(msg->length() == 11){
			std::string error_str = msg->substr(1,9);
			fprintf(stderr,"\nWarning: %s",error_str.c_str());
		return;
	}
    
    if(msg->length() != 38){
		//	fprintf(stderr,"\nAn unexpected message arrived - ignored");
        return;
    }

   // int coil_n = (int)msg->at(7) - 48;   //descobre qual eh a coil
	int signs = 1;
	//CHANNEL 1
	/*if(msg->at(13) == '+')
		signs = 1;
	if(msg->at(13) == '-')
		signs = -1;
*/
//--------------------------------------------------------------
// De-serialize data
//Metal-detector
	std::string value_str = msg->substr(1,2);
	int channel_value = 0;
	unsigned int temp = (value_str[1]<<24);
	unsigned int temp2 = temp>>24;
	int temp1 = value_str[0]<<8;
	//fprintf(stderr,"\n %x", temp1);
	//fprintf(stderr," %x", temp2);
	channel_value = temp1 + temp2; //juntar bits mais significativos com os menos signifivativos

//Roll
	char temp2_str[4];
	temp2_str[0] = (char)msg->at(24);
	temp2_str[1] = (char)msg->at(23);
	temp2_str[2] = (char)msg->at(22);
	temp2_str[3] = (char)msg->at(21);
	float roll = *(float *)temp2_str;
    //fprintf(stderr,"\nRoll: %f", roll);
	
//Pitch
	//char temp2_str[4];
	temp2_str[0] = (char)msg->at(28);
	temp2_str[1] = (char)msg->at(27);
	temp2_str[2] = (char)msg->at(26);
	temp2_str[3] = (char)msg->at(25);
	float pitch = *(float *)temp2_str;
    //fprintf(stderr,"   Pitch: %f", pitch);
	
//Yaw
	//char temp2_str[4];
	temp2_str[0] = (char)msg->at(32);
	temp2_str[1] = (char)msg->at(31);
	temp2_str[2] = (char)msg->at(30);
	temp2_str[3] = (char)msg->at(29);
	float yaw = *(float *)temp2_str;
    //fprintf(stderr,"   Yaw: %f", yaw);
	
	
	
//--------------------------------------------------------------
	
//Prepare and send messages and TF -----------------------------	
	//---------------------------
	//Metal detector message ----
	coils[0].channel[0] = signs*channel_value;
	
	// Prepara mensagem coil data
	ros::Time current_time = ros::Time::now();
	std::string frame = frame_id;
	frame.append("_middle_coil");
    metal_detector_msgs::Coil coil_msg;//cria mensagem
	coil_msg.header.stamp = current_time;
	coil_msg.header.frame_id = frame;
	coil_msg.channel.resize(3);
	coil_msg.zero.resize(3);
	for(int i=0 ; i<3 ; i++)
	{
		coil_msg.channel[i] = coils[0].channel[i];
		coil_msg.zero[i] = coils[0].zero[i];
	}
    //Publica msg detector de metais
    coil_pub_ptr->publish(coil_msg);
       
	// Publish the transform over tf
	geometry_msgs::TransformStamped coil_transform;
	coil_transform.header.stamp = current_time;
	coil_transform.header.frame_id = frame_id;
	coil_transform.child_frame_id = frame;
	coil_transform.transform.translation.x = 0.0;
	coil_transform.transform.translation.y = 0.0;//variava o y se tivesse varias coils
	coil_transform.transform.translation.z = 0.0;
	coil_transform.transform.rotation = tf::createQuaternionMsgFromYaw(0);
	// Send the transform
	tf_broadcaster_ptr->sendTransform(coil_transform);
	//---------------------------
	
	//-- IMU message -------------------------
	// Prepara mensagem imu
	// usar o mesmo current_time do DM
    std::string frame_imu = "imu";
	frame.append("_gy80");
    sensor_msgs::Imu gy80_msg; //cria mensagem
    gy80_msg.header.stamp = current_time;
    gy80_msg.header.frame_id = frame_imu;

    //Convert RPY to quaternion.
    tf::Quaternion q;
    //double roll, pitch, yaw;
    //tf::quaternionMsgToTF(imu_msg->orientation, q);
    //tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);
    //ROS_DEBUG("RPY = (%lf, %lf, %lf)", roll, pitch, yaw);

    q = tf::createQuaternionFromRPY(roll,pitch,yaw);

    gy80_msg.orientation.x = q.getX();
    gy80_msg.orientation.y = q.getY();
    gy80_msg.orientation.z = q.getZ();
    gy80_msg.orientation.w = q.getW();

    //fprintf(stderr,"\nPublicar");
    //Publica msg gy80
    gy80_pub_ptr->publish(gy80_msg);
	
//--------------------------------------------------------------

}

bool setCoilsZero(metal_detector_msgs::SetCoilsZero::Request &req, metal_detector_msgs::SetCoilsZero::Response &res)
{
	if(req.coil.size() == 0)
	{
		for(int i=0 ; i<3 ; i++)
		{
			for(int j=0 ; j<3 ; j++)
			{
				coils[i].zero[j] = coils[i].channel[j];
			}
		}

		res.status_message = "Set coils zero was successful using current coil values.";
		return true;
	}
	else if(req.coil.size() == 3)
	{
		for(int i=0 ; i<3 ; i++)
		{
			if(req.coil[i].zero.size() != 3)
			{
				res.status_message = "Set coils zero failed, number of coil channels is invalid.";
				return false;
			}
			for(int j=0 ; j<3 ; j++)
			{
				coils[i].zero[j] = req.coil[i].zero[j];
			}
		}

		res.status_message = "Set coils zero was successful using the provided values.";
		return true;
	}
	else
	{
		res.status_message = "Set coils zero failed, number of coils is invalid.";
		return false;
	}
	
	return true;
}

int main(int argc, char ** argv)
{
	ros::init(argc, argv, "gy80_md_driver_node");

	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	tf::TransformBroadcaster tf_broadcaster;
	tf_broadcaster_ptr = &tf_broadcaster;
	
	//Publishers
    ros::Publisher coil_pub = n.advertise<metal_detector_msgs::Coil>("coils", 50);
    ros::Publisher gy80_pub = n.advertise<sensor_msgs::Imu>("gy80/data", 50);
    //ros::Publisher coil_pubR = n.advertise<metal_detector_msgs::Coil>("coilR", 50);


	coil_pub_ptr = &coil_pub;
    gy80_pub_ptr = &gy80_pub;
    //coil_pub_ptrR = &coil_pubR;
	ros::ServiceServer service = n.advertiseService("set_coils_zero", setCoilsZero);

	for(int i=0 ; i<3 ; i++)
	{
		for(int j=0 ; j<3 ; j++)
		{
			coils[i].channel[j] = 0;
			coils[i].zero[j] = 0;
		}
	}

	XmlRpc::XmlRpcValue middle_coil_zero;
    	if( n.getParam("middle_coil_zero", middle_coil_zero) )
    	{   
        	ROS_ASSERT(middle_coil_zero.getType() == XmlRpc::XmlRpcValue::TypeArray);
        
		for(int i=0 ; i<middle_coil_zero.size() ; ++i) 
		{
		    ROS_ASSERT(middle_coil_zero[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
		    coils[0].zero[i] = static_cast<int>(middle_coil_zero[i]);
		}
    	}

	XmlRpc::XmlRpcValue left_coil_zero;
    	if( n.getParam("left_coil_zero", left_coil_zero) )
    	{   
        	ROS_ASSERT(left_coil_zero.getType() == XmlRpc::XmlRpcValue::TypeArray);
        
		for(int i=0 ; i<left_coil_zero.size() ; ++i) 
		{
		    ROS_ASSERT(left_coil_zero[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
		    coils[1].zero[i] = static_cast<int>(left_coil_zero[i]);
		}
    	}

	XmlRpc::XmlRpcValue right_coil_zero;
    	if( n.getParam("right_coil_zero", right_coil_zero) )
    	{   
        	ROS_ASSERT(right_coil_zero.getType() == XmlRpc::XmlRpcValue::TypeArray);
        
		for(int i=0 ; i<right_coil_zero.size() ; ++i) 
		{
		    ROS_ASSERT(right_coil_zero[i].getType() == XmlRpc::XmlRpcValue::TypeInt);
		    coils[2].zero[i] = static_cast<int>(right_coil_zero[i]);
		}
    	}

	pn.param<std::string>("frame_id", frame_id, "metal_detector");

	std::string port;
	
    pn.param<std::string>("port", port, "/dev/ttyACM0");

    //pn.param<std::string>("port", port, "/dev/metal_detector");
    	int baudrate;
	pn.param("baudrate", baudrate, 115200);

	cereal::CerealPort md_port;
	md_port.open(port.c_str(), baudrate);
	if(!md_port.portOpen())
	{
        ROS_FATAL("Schiebel MD and Gy80 -- Failed to open serial port %s at %d baud!", port.c_str(), baudrate);
		ROS_BREAK();
	}
    ROS_INFO("Schiebel MD and Gy80 -- Successfully connected to the Schiebel MD!");
	
	md_port.startReadBetweenStream(streamCallback, '$', '\n');

	ros::spin();

	md_port.close();
	return(0);
}

// EOF

