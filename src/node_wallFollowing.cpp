#include "patrol_jackal/wallFollowing.h"
#include <math.h>
#define PI 3.141592

int case3counter = 200;

NodeWallFollowing::NodeWallFollowing(ros::Publisher pub, ros::ServiceClient srv_client, object_marker::Objects objs, double wallDist, double maxSp, int dir, double pr, double di, double an):ac("move_base",true)
{
	wallDistance = wallDist;
	maxSpeed = maxSp;
	direction = dir;
	P = pr;
	D = di;
	angleCoef = an;
	e = 0;
	e_left = 0;
	e_right = 0;
	angleMin = 0; //angle, at which was measured the shortest distance
	pubMessage = pub;
	object_marker_client = srv_client;
	follow_mode = 0;
	objects = objs;
	case3aj = false;
}

NodeWallFollowing::~NodeWallFollowing()
{
}

//Publisher
void NodeWallFollowing::publishMessage()
{
	//preparing message
	geometry_msgs::Twist msg;
	int size;
	int minIndex;
	int maxIndex;

	

	if (follow_mode == 3 || follow_mode == 4)
	{
		if (((direction == 1 && distLeft > 1.5 * wallDistance) || (direction == -1 && distRight > 1.5 * wallDistance)) && case3aj == false) //distFront > wallDistance &&
			follow_mode = 4;
		else
			follow_mode = 3;
	}
	else
	{
		ROS_INFO("%f,%f,%f", distFront, distLeft, distRight);

		if (distFront > wallDistance && distLeft > wallDistance && distRight > wallDistance && follow_mode != 2)
			follow_mode = 1;
		if ((distFront < wallDistance || distLeft < wallDistance || distRight < wallDistance) && follow_mode != 2)
		{
			if (distFront < wallDistance)
			{
				if (distFront_left < distFront_right)
				{
					direction = 1;
				}
			}
			else if (distLeft < distRight)
			{
				direction = 1;
			}
			follow_mode = 2;
		}
		if ((distLeft < 1.5 * wallDistance && fabs(angleMin - PI * direction / 2) < 0.1) || (distRight < 1.5 * wallDistance && fabs(angleMin - PI * direction / 2) < 0.1))
		{
			follow_mode = 3;
		}
	}

	ROS_INFO("%d", follow_mode);

	switch (follow_mode)
	{
	case 1:
		msg.linear.x = maxSpeed * 0.5;
		break;
		// follow move_base goal
	case 2:
		// msg.angular.z = direction*(P*e + D*diffE) + angleCoef * (angleMin - PI*direction/2);    //PD controller
		ROS_INFO("%f", angleMin);
		msg.angular.z = angleCoef * (angleMin - PI * direction / 2);
		if (msg.angular.z < -0.5)
			msg.angular.z = -0.5;
		if (msg.angular.z > 0.5)
			msg.angular.z = 0.5;
		// msg.linear.x = maxSpeed - 4*maxSpeed*(wallDistance-distFront)/wallDistance;
		msg.linear.x = 0;
		break;
	case 3:

		case3counter++;
		if (direction == 1)
		{
			diffE = (distLeft - wallDistance) - e_left;
			e_left = distLeft - wallDistance;
			msg.linear.x = 0.3 * maxSpeed;
			if (distRight < 0.3)
			{
				msg.angular.z = 0.3 * direction * (P * e_left + D * diffE) - (distRight - 0.3); //PD controller
			}
			else
			{
				msg.angular.z = 0.3 * direction * (P * e_left + D * diffE); //PD controller
			}
			ROS_INFO("left");
		}
		else if (direction == -1)
		{
			diffE = (distRight - wallDistance) - e_right;
			e_right = distRight - wallDistance;
			msg.linear.x = 0.3 * maxSpeed;
			if (distLeft < 0.3)
			{
				msg.angular.z = 0.3 * direction * (P * e_right + D * diffE) + (distLeft - 0.3); //PD controller
			}
			else
			{
				msg.angular.z = 0.3 * direction * (P * e_right + D * diffE); //PD controller
			}
			ROS_INFO("right");
		}

		std::cout << "case3counter" << case3counter << std::endl;

		if (case3counter > 500 && ((distRight>0.2 && distLeft>0.15 && direction == 1) || (distRight>0.15 && distLeft>0.2 && direction == -1))) 
		{
			
			ROS_INFO("ready to detect");
			if (case3aj == false)
				ninetyDegreeTurn(0.5 * direction, 0);

			// adjust orientation to face the object
			int last_e_laserIndex = e_laserIndex;
			if (abs(e_laserIndex) > 10)
			{
				ROS_INFO("ttt");
				case3aj = true;
				int diff_e_laser = e_laserIndex - last_e_laserIndex;
				last_e_laserIndex = e_laserIndex;
				msg.angular.z = 0.001 * (P * last_e_laserIndex + D * diff_e_laser); //PD controller
				// pubMessage.publish(msg);
				// ros::spinOnce();
			}else{
				case3aj = false;

			// move backward 0.1 m
			while (!ac.waitForServer(ros::Duration(5.0)))
			{
				ROS_INFO("Waiting for the move_base action server to come up");
			}
			goal.target_pose.header.frame_id = "base_link";
			goal.target_pose.header.stamp = ros::Time::now();
			goal.target_pose.pose.position.x = -0.2;
			goal.target_pose.pose.orientation.w = 1.0; // need to update
			ac.sendGoal(goal);
			ac.waitForResult();
			ros::Duration(3).sleep();
			if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				ROS_INFO("Start Detecting");
				srv.request.old_objects = objects;
				if (object_marker_client.call(srv))
				{
					if (srv.response.success)
					{
						ROS_INFO("Call Service Successfully");
						int new_count = srv.response.updated_objects.objects.size() - srv.request.old_objects.objects.size();
						int total_count = srv.response.updated_objects.objects.size();
						ROS_INFO("New detected %d objects, now have %d", new_count, total_count);
						objects = srv.response.updated_objects;
					}
					else 
						ROS_INFO("Called Service, No New Object Found");
					
				}
				else
				{
					ROS_ERROR("Failed to call service Object Mark");
				}
			}
			else
				ROS_INFO("The base failed to move for some reason");

			ninetyDegreeTurn(-0.5 * direction, 0);
			//publish
			// srv.request.old_objects = objects;

			// std_msgs::Bool msg_detected_object;
			// msg_detected_object.data = true;
			// pubDetectionReady.publish(msg_detected_object);
			// ros::Duration(10.0).sleep();
			// if (isDetectionObject == false)
			// {
			// 	msg_detected_object.data = false;
			// 	pubDetectionReady.publish(msg_detected_object);
			// 	ninetyDegreeTurn(-0.5 * direction, 0);
			// }
			// else
			// {
			// 	msg.linear.x = 0;
			// 	msg.angular.z = 0;
			// }
			case3counter = 0;
			ROS_INFO("set counter to 0");
			}
		}
		
		isturn = false;
		ROS_INFO("break 3");
		break;
	case 4:
		size = laser_msg.ranges.size();
		minIndex = round((double)size * (double)(direction + 1) / 4.0 + 3.0 * (double)(1 + direction) * size / 16.0);
		maxIndex = round((double)size * (double)(direction + 3) / 4.0 - 3.0 * (double)(1 - direction) * size / 16.0);
		//ROS_INFO("%d, %d,%d", size, minIndex, maxIndex);
		for (int i = minIndex; i < maxIndex; i++)
		{
			if (laser_msg.ranges[i] < laser_msg.ranges[minIndex] && laser_msg.ranges[i] > 0.0)
			{
				minIndex = i;
			}
		}
		//ROS_INFO("minimum_id: %d, minimum_distance: %f", minIndex, laser_msg.ranges[minIndex]);
		if (laser_msg.ranges[minIndex] < 1.5 * wallDistance) // there's still wall
			msg.linear.x = 0.5 * maxSpeed;					 // more control might be needed
		else
		{
			double start_time = ros::Time::now().toSec();
			double current_time = start_time;
			while (current_time - start_time < 3.3 && isturn == false)
			{
				msg.angular.z = 0.5 * direction;
				msg.linear.x = fabs(msg.angular.z * wallDistance);
				pubMessage.publish(msg);
				ros::Duration(0.01).sleep();
				current_time = ros::Time::now().toSec();
				//ROS_INFO("%f,%f", start_time, current_time);
			}
			isturn = true;
		}
		break;
	default:
		break;
	}

	//publishing message
	//if(current_cmd.linear.x!=msg.linear.x || current_cmd.angular.z != msg.angular.z){
	pubMessage.publish(msg);
	current_cmd = msg;
	ROS_INFO("finish call back");
	// }
}

void NodeWallFollowing::ninetyDegreeTurn(double yaw_speed, double x_speed)
{
	double start_time = ros::Time::now().toSec();
	double current_time = start_time;
	geometry_msgs::Twist msg;
	while (current_time - start_time < 3 && isturn == false)
	{
		msg.angular.z = yaw_speed;
		msg.linear.x = x_speed;
		pubMessage.publish(msg);
		ros::Duration(0.01).sleep();
		current_time = ros::Time::now().toSec();
		//ROS_INFO("%f,%f", start_time, current_time);
	}
}

//Subscriber
void NodeWallFollowing::messageCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	ROS_INFO("new laser mesge");
	if (isDetectionObject == false)
	{
		laser_msg = *msg;
		int size = laser_msg.ranges.size();

		//Variables whith index of highest and lowest value in array.
		int minIndex = size * (direction + 1) / 4;
		int maxIndex = size * (direction + 3) / 4;
		int mid_laser_index = (int)size / 2;

		//This cycle goes through array and finds minimum
		for (int i = minIndex; i < maxIndex; i++)
		{
			if (laser_msg.ranges[i] < 0.002)
			{
				laser_msg.ranges[i] = 100;
			}
			if (laser_msg.ranges[i] < laser_msg.ranges[minIndex] && laser_msg.ranges[i] > 0.0)
			{
				minIndex = i;
			}
		}

		//Calculation of angles from indexes and storing data to class variables.
		angleMin = (minIndex - size / 2) * laser_msg.angle_increment;
		double distMin;
		distMin = laser_msg.ranges[minIndex];
		distFront = laser_msg.ranges[size / 2];
		for (int i = 1; i < 10; i++){
			distFront_left = distFront_left + laser_msg.ranges[size/2 + i];
			distFront_right = 	distFront_right + laser_msg.ranges[size/2 - i];
		}
		distFront_left = distFront_left / 9;
		distFront_right = distFront_right / 9;	
		distLeft = laser_msg.ranges[size - 1];
		distRight = laser_msg.ranges[0];
		// if (distFront < 0.02)
		// 	distFront = 100;
		// if (distLeft < 0.02)
		// 	distLeft = 100;
		// if (distRight < 0.02)
		// 	distRight = 100;
		diffE = (distMin - wallDistance) - e;
		e = distMin - wallDistance;
		e_laserIndex = minIndex - mid_laser_index;

		//Invoking method for publishing message
		ROS_INFO("pub message");
		publishMessage();
		ROS_INFO("finish pub");
	}
}

void NodeWallFollowing::objectDetectedCallback(const std_msgs::Bool::ConstPtr &msg)
{
	isDetectionObject = msg->data;
}
