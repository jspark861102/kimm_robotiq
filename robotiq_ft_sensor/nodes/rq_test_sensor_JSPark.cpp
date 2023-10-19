/* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Robotiq, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Robotiq, Inc. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Copyright (c) 2014, Robotiq, Inc
*/

/**
 * \file rq_test_sensor.cpp
 * \date July 14, 2014
 *  \author Jonathan Savoie <jonathan.savoie@robotiq.com>
 *  \maintainer Jean-Philippe Roberge <ros@robotiq.com>
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "robotiq_ft_sensor/ft_sensor.h"
#include "robotiq_ft_sensor/sensor_accessor.h"

#include <sstream>

#include <sys/ioctl.h>
#include <termios.h>
#include "std_msgs/Bool.h"
#include "std_msgs/Int64.h"

int isreset = 0;
int msg = 0;

bool _kbhit()
{
    termios term;
    tcgetattr(0, &term);

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(0, TCSANOW, &term2);

    int byteswaiting;
    ioctl(0, FIONREAD, &byteswaiting);

    tcsetattr(0, TCSANOW, &term);

    return byteswaiting > 0;
};

void ResetProc(){  
	if (_kbhit()){

		char key = getchar();
		key = tolower(key);    

		switch (key){
		case 'r': //reset          
			msg = 1;
			break;      
		case '\n':
			msg = 0;
			break;
		case '\r':
			msg = 0;
			break;
		default:
			msg = 0;
			break;	
		}
    }  
}

/*void receiveCallback(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}*/

void reCallback(const robotiq_ft_sensor::ft_sensor& msg)
{
	ROS_INFO("I heard: FX[%f] FY[%f] FZ[%f] MX[%f] MY[%f] MZ[%f]", msg.Fx,msg.Fy,msg.Fz,msg.Mx,msg.My,msg.Mz);
}


void resetCallback(std_msgs::Bool reset_msg)
{
	if (reset_msg.data) msg = 1;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

	ros::init(argc, argv, "rq_test_sensor_JSPark");

	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<robotiq_ft_sensor::sensor_accessor>("robotiq_ft_sensor_acc");
	ros::Subscriber sub1 = n.subscribe("robotiq_ft_sensor",100,reCallback);
	robotiq_ft_sensor::sensor_accessor srv;

	ros::Subscriber sub_reset = n.subscribe("robotiq_ft_sensor_reset",100,resetCallback);

	ros::Rate r(10);
	while (ros::ok())
	{		
		ResetProc();
		if (msg) {
			/// Deprecated Interface
			// srv.request.command = "SET ZRO";

			/// New Interface with numerical commands
			srv.request.command_id = srv.request.COMMAND_SET_ZERO;
			if(client.call(srv)){
				ROS_INFO("ret: %s", srv.response.res.c_str());
			}
			ROS_INFO("FT sensor calibrated.");	
			msg = 0;		
		}

		ros::spinOnce();
		r.sleep();
	}
	return 0;
}


