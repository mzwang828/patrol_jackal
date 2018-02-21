#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sstream>
#include <fstream>
#include <string>
#include <vector>

#include "patrol_jackal/wallFollowing.h"
#include <object_marker/Object.h>
#include <object_marker/Objects.h>
#include <object_marker/MarkObject.h>
#include <geometry_msgs/TransformStamped.h>

#define SUBSCRIBER_BUFFER_SIZE 1   // Size of buffer for subscriber.
#define PUBLISHER_BUFFER_SIZE 1000 // Size of buffer for publisher.
#define WALL_DISTANCE 0.7
#define MAX_SPEED 0.3
#define P 5          // Proportional constant for controller
#define D 25         // Derivative constant for controller
#define ANGLE_COEF 1 // Proportional constant for angle controller
#define DIRECTION -1 // 1 for wall on the left side of the robot (-1 for the right side).
#define PUBLISHER_TOPIC "/cmd_vel"
#define SUBSCRIBER_TOPIC "/laser/scan"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "patrol");
    ros::NodeHandle n("~");

    // read in existed objects from saved file
    ifstream infile("/home/mzwang/Desktop/objects.txt", ios::in | ios::out | ios::binary);
    vector<vector<float>> objects;
    vector<float> one_object;
    string line;
    int row = 0;
    while (getline(infile, line))
    {
        stringstream ss;
        ss << line;
        float a;
        while (ss >> a) //write as ss>>a as while condition to avoid repeated last element
        {
            one_object.push_back(a);
        }
        objects.push_back(one_object);
        one_object.clear();
        row++;
    }

    object_marker::Objects existed_objects;
    for (int i = 0; i < row; i++)
    {
        object_marker::Object existed_object;
        existed_object.transform.translation.x = objects[i][0];
        existed_object.transform.translation.y = objects[i][1];
        existed_object.transform.translation.z = objects[i][2];
        existed_object.transform.rotation.x = objects[i][3];
        existed_object.transform.rotation.y = objects[i][4];
        existed_object.transform.rotation.z = objects[i][5];
        existed_object.transform.rotation.w = objects[i][6];
        existed_objects.objects.push_back(existed_object);
    }

    ros::Publisher pubMessage = n.advertise<geometry_msgs::Twist>(PUBLISHER_TOPIC, PUBLISHER_BUFFER_SIZE);
    ros::ServiceClient client = n.serviceClient<object_marker::MarkObject>("object_mark");
    
    NodeWallFollowing *nodeWallFollowing = new NodeWallFollowing(pubMessage, client, existed_objects, WALL_DISTANCE, MAX_SPEED, DIRECTION, P, D, 1)
    ros::Subscriber sub = n.subscribe(SUBSCRIBER_TOPIC, SUBSCRIBER_BUFFER_SIZE, &NodeWallFollowing::messageCallback, nodeWallFollowing);
    
    ros::spin();
    
    return 0;
}