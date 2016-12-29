#include <ros/ros.h>

#include <std_msgs/String.h>

#include <sstream>

#include <vector>


using namespace std;

class Chukwaface
{
private:
	
	ros::NodeHandle nh;
	ros::Subscriber sub_from_shape;
	ros::Publisher command_pub;

	vector<string> shapes;
	std_msgs::String command;

	char choice;

	void callback_shape(const std_msgs::String shapes_str)
	{
		string str(shapes_str.data.c_str());
		string buf;
		stringstream ss(str);

		while(ss >> buf) 
		{
			shapes.push_back(buf); 
		}
	}

	void showShapes()
	{
		ros::spinOnce(); 
		
		system("clear");
		cout << "*****************************\n";
		if(shapes.empty())
			{
				cout << "No shapes available!\n";
			}
		else 
			{
				int count = 0;
				for (vector<string>::iterator i = shapes.begin(); i != shapes.end(); ++i)
				{
					cout << ++count << " - " << *i << "\n"; 
				}
			}
		cout << "0 - Back\n"
			 << "*****************************\n";

		do
		{
			cin >> choice;

			switch(choice)
				{
					case('1'):
						command.data = "SHAPE 1";
						break;
					case('2'):
						command.data = "SHAPE 2";
						break;
					case('0'):
						start();
						break;
					default:
						ROS_WARN("'%c' is not a valid input!", choice);
				}

				
			command_pub.publish(command);

		}while(ros::ok());
	}

	void showChangeLeds()
	{
		system("clear");
		cout	<< "*************************\n"
				<< "1 - Toggle led1\n"
				<< "2 - Toggle led2\n"
				<< "3 - Toggle DISCO-MODE\n"
				<< "0 - Back\n"
				<< "*************************\n";

		do
		{
			cin >> choice;

			switch(choice)
			{
				case('1'):
					command.data = "LED 1"; 
					break;
				case('2'):
					command.data = "LED 2";
					break;
				case('3'):
					command.data = "LED PARTY";
					break;
				case('0'):
					start();
					break;
				default:
					ROS_WARN("'%c' is not a valid input!", choice);
			}

			command_pub.publish(command);

		} while(ros::ok());
			
	}

	void showSounds()
	{
		system("clear");
		cout	<< "*************************\n"
				<< "Choose sound to play:\n"
				<< "1 - turn on\n"
				<< "2 - turn off\n"
				<< "3 - recharge start\n"
				<< "4 - press button\n"
				<< "5 - error sound\n"
				<< "6 - start cleaning\n"
				<< "7 - cleaning end\n"
				<< "0 - Back\n"
				<< "*************************\n";

		do
		{
			cin >> choice;

			switch(choice)
			{
				case('1'):
					command.data = "SOUND 1"; 
					break;
				case('2'):
					command.data = "SOUND 2";
					break;
				case('3'):
					command.data = "SOUND 3";
					break;
				case('4'):
					command.data = "SOUND 4"; 
					break;
				case('5'):
					command.data = "SOUND 5";
					break;
				case('6'):
					command.data = "SOUND 6";
					break;
				case('7'):
					command.data = "SOUND 7"; 
					break;
				case('0'):
					start();
					break;
				default:
					command.data = "SOUND"; 
					ROS_WARN("'%c' is not a valid input!", choice);
			}

			command_pub.publish(command);

		} while(ros::ok());
			
	}

	void showAbout()
	{
		system("clear");
		cout	<< "*************************\n"
				<< "This program is made by: B217\n"
				<< "Group members:\n"
				<< "Aleksandra Zasadni\n"
				<< "Andrej Orsula\n"
				<< "Asger Printz Madsen\n"
				<< "Christoffer Sand Andersen\n"
				<< "Jesper Frederik Hansen\n"
				<< "Lukas Wyon\n"
				<< "Soren Myhre Voss\n\n"
				<< "0 - Back\n"
				<< "*************************\n";

		do
		{
			cin >> choice;

			if (choice == '0')
			{
				start();
			}
			else
			{
				ROS_WARN("'%c' is not a valid input!", choice);
			}
		}while(ros::ok());
	}

	void showHelp()
	{
		system("clear");
		cout	<< "*************************\n"
				<< "No help available.\n"
				<< "0 - Back\n"
				<< "*************************\n";

		do
		{
			cin >> choice;

			if (choice == '0')
			{
				start();
			}
			else
			{
				ROS_WARN("'%c' is not a valid input!", choice);
			}
		}while(ros::ok());
	}

	void start()
	{
		system("clear");
		cout	<< "*************************\n"
				<< "1  - Show Shapes\n"
				<< "2  - Change LEDs\n"
				<< "3  - Play Sounds\n"
				<< "8  - About\n"
				<< "9  - Help\n"
				<< "^C - EXIT\n"
				<< "*************************\n";
		do
		{
			cin >> choice;

			switch(choice)
			{
				case('1'):
					showShapes();
					break;
				case('2'):
					showChangeLeds();
					break;
				case('3'):
					showSounds();
					break;	
				case('8'):
					showAbout();
					break;
				case('9'):
					showHelp();
					break;
				default:
					ROS_WARN("'%c' is not a valid input!", choice);
			}
		}while(ros::ok());
	}

public:
	Chukwaface()
	{
		sub_from_shape = nh.subscribe<std_msgs::String>("Chukwa_shapes", 10, &Chukwaface::callback_shape, this);

		command_pub = nh.advertise<std_msgs::String>("Chukwashape_trigger", 10);

		start();
	};
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "Chukwaface");

	Chukwaface Chukwa_face;

	ros::spin();
	return 0;
}
