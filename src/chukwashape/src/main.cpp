#include <ros/ros.h>

#include <std_msgs/String.h>

#include <sstream>

#include <string>

#include <geometry_msgs/Twist.h>

#include <kobuki_msgs/Led.h>

#include <kobuki_msgs/Sound.h>

#define shapes_count 2
#define max_steps 11 

using namespace std;


class Chukwashape
{
private:
	ros::NodeHandle nh;
	ros::Subscriber sub_from_face;
	ros::Publisher shape_publisher;
	ros::Publisher led1_pub;
	ros::Publisher led2_pub;
	ros::Publisher party_pub;
	ros::Publisher chukwa_move_pub;
	ros::Publisher chukwa_sound_pub;
	ros::Timer chukwa_shape_timer;
	ros::Timer party_timer;

	std_msgs::String all_shapes;
	geometry_msgs::Twist chukwa_twist;
	kobuki_msgs::Sound chukwa_sound;
	kobuki_msgs::Led led1;
	kobuki_msgs::Led led2;

	bool startParty;
	int step, counter_for_timer;


	int pattern[max_steps][2],
		square[max_steps][2]={{100,0},{0,47},{100,0},{0,47},{100,0},{0,47},{100,0},{0,47}},
		diamond[max_steps][2]={{150,0},{0,43},{24,0},{0,20},{52,0},{0,20},{24,0},{0,43},{65,0},{0,43}};


	struct ShapeStruct
	{
		string name;
	}
	shapes[shapes_count];

	void callback_face(const std_msgs::String command_send)
	{
		string command_name; 
		string command_rest; 
		
		stringstream ss (command_send.data);
			ss >> command_name;
			ss >> command_rest;

		
		ROS_INFO("Recieved: \n name: %s \n data: %s", command_name.c_str(), command_rest.c_str());

		
		if(command_name == "SHAPE")
		{
			moveChukwa(command_rest);
		}
		else if(command_name ==  "LED")
		{
			controlLEDs(command_rest);
		}
		else if(command_name ==  "SOUND")
		{
			playSounds(command_rest);
		}
	}

	void moveChukwa(const string& shape)
	{
		if (shape == "1")
		{
			copy_array(square, pattern);
			chukwa_shape_timer.start();
		}
		else if (shape == "2")
		{
			copy_array(diamond, pattern);
			chukwa_shape_timer.start();
		}
		step = 0;
		counter_for_timer++;
	}

	void copy_array(int copyfrom[max_steps][2], int copyto[max_steps][2])
	{
		for (int i = 0; i < max_steps; ++i)
		{
			for (int j = 0; j < 2; ++j)
			{
				copyto[i][j]=copyfrom[i][j];
			}
		}
	}

	void publishing(const ros::TimerEvent&)
	{
		if (pattern[step][0] != 0 && pattern[step][1] == 0)
		{
			chukwa_twist.linear.x = 0.125;
			chukwa_twist.angular.z = 0;
		}
		
		else if (pattern[step][0] == 0 && pattern[step][1] != 0)
		{
			chukwa_twist.linear.x = 0;
			chukwa_twist.angular.z = 0.785;
		}
		else if (pattern[step][0] == 0 && pattern[step][1] == 0)
		
		{
			chukwa_twist.linear.x = 0;
			chukwa_twist.angular.z = 0;
		}


		if (pattern[step][0] == 0 && pattern[step][1] == 0)
		{
			next_step();
		}
		else if (pattern[step][0] != 0 && pattern[step][1] == 0 && counter_for_timer > pattern[step][0])
		{
			next_step();
		}
		else if (pattern[step][0] == 0 && pattern[step][1] != 0 && counter_for_timer > pattern[step][1])
		{
			next_step();
		}
		chukwa_move_pub.publish(chukwa_twist);
		counter_for_timer++;
	}

	void next_step()
	{
		step++;
		counter_for_timer = 0;

		chukwa_sound.value = 6;
		chukwa_sound_pub.publish(chukwa_sound);

		if (pattern[step][0] == 0 && pattern[step][1] == 0)
		{
			chukwa_twist.linear.x = 0;
			chukwa_twist.angular.z = 0;
			step = 0;
			chukwa_shape_timer.stop();
		}
		ros::Duration(1).sleep();
	}

	void controlLEDs(const string& led)
	{
		if(led == "1")
		{
			led1.value ? led1.value = 0 : led1.value = 1;
			led1_pub.publish(led1); 
		}
		else if(led == "2")
		{
			led2.value? led2.value = 0 : led2.value = 1;
			led2_pub.publish(led2); 
		}
		else if(led == "PARTY")
		{
			startParty = !startParty; 
		}
	}

	void callParty(const ros::TimerEvent&)
	{
		if(startParty)
		{
			led1.value = rand() % 4; 
			led1_pub.publish(led1);
			led2.value = rand() % 4; 
			led2_pub.publish(led2);
		}
	}

	void playSounds(const string& shape)
	{
		chukwa_sound.value = atoi(shape.c_str()) - 1; 

		chukwa_sound_pub.publish(chukwa_sound);
	}

	void publishShapes()
	{
		stringstream ss;

		for (int i = 0; i < shapes_count; ++i)
		{
			ss << shapes[i].name << " ";
		}

		all_shapes.data = ss.str();

		while(! shape_publisher.getNumSubscribers() > 0)
		{
			ros::spinOnce(); 
		}

		shape_publisher.publish(all_shapes);
	}
public:
	Chukwashape()
	{
		sub_from_face = nh.subscribe<std_msgs::String>("Chukwashape_trigger", 10, &Chukwashape::callback_face, this);

		shape_publisher = nh.advertise<std_msgs::String>("Chukwa_shapes", 1);

		chukwa_move_pub = nh.advertise<geometry_msgs::Twist>("mobile_base/commands/velocity", 1);

		chukwa_sound_pub = nh.advertise<kobuki_msgs::Sound>("mobile_base/commands/sound", 1);

		led1_pub = nh.advertise<kobuki_msgs::Led>("mobile_base/commands/led1", 1);
		led2_pub = nh.advertise<kobuki_msgs::Led>("mobile_base/commands/led2", 1);

		chukwa_shape_timer = nh.createTimer(ros::Duration(0.05),  &Chukwashape::publishing, this);
		
		party_timer = nh.createTimer( ros::Duration(0.1), &Chukwashape::callParty, this);

		shapes[0].name = "SQUARE";
		shapes[1].name = "DIAMOND";

		counter_for_timer = 0;
		
		startParty = 0;

		chukwa_shape_timer.stop();

		
		publishShapes();
	};
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "Chukwashape");

	
	Chukwashape Chukwa_go;

	ros::spin();
	return 0;
}
