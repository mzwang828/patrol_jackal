#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "patrol_jackal/wallFollowing.h"
#include <object_marker/Object.h>
#include <object_marker/Objects.h>
#include <object_marker/MarkObject.h>
#include <object_marker/RecheckObject.h>
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

using namespace std;

visualization_msgs::Marker marker_publish(object_marker::Object object_to_be_pub, int id, int action)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = action;
    marker.pose.position.x = object_to_be_pub.transform.translation.x;
    marker.pose.position.y = object_to_be_pub.transform.translation.y;
    marker.pose.position.z = object_to_be_pub.transform.translation.z;
    marker.pose.orientation = object_to_be_pub.transform.rotation;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 2;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration();
    return marker;
}

/* void write_objects(object_marker::Objects all_detect_objects_)
{
    ofstream out;
    out.open("/home/river/Desktop/objects.txt", ios_base::trunc | ios_base::out);
    for (i = 0; i < objects.size(); i++)
    {
        float object_pose_orientation[7] = {all_detect_objects_.objects[i].transform.translation.x, all_detect_objects_.objects[i].transform.translation.y, all_detect_objects_.objects[i].transform.translation.z,
                                            all_detect_objects_.objects[i].transform.rotation.x, all_detect_objects_.objects[i].transform.rotation.y, all_detect_objects_.objects[i].transform.rotation.z, all_detect_objects_.objects[i].transform.rotation.w};
        for (int j = 0; j < 7; j++)
        {
            out << object_pose_orientation[j] << " ";
        }
        out << "\n";
    }
    out.close();
} */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "patrol");
    ros::NodeHandle n("~");
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("/object_marker", 10);
    ros::Publisher pubMessage = n.advertise<geometry_msgs::Twist>(PUBLISHER_TOPIC, PUBLISHER_BUFFER_SIZE);
    ros::ServiceClient client = n.serviceClient<object_marker::MarkObject>("/object_marker/object_mark");
    
    object_marker::Objects existed_objects;
    object_marker::Objects existed_objects_update;

    // read in existed objects from saved file
    ifstream infile("/home/mzwang/Desktop/objects.txt", ios::in | ios::out | ios::binary);
    if (infile.good())
    {
        vector<vector <float> > objects;
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
            visualization_msgs::Marker marker_ = marker_publish(existed_objects.objects[i], i, 0);
            marker_pub.publish(marker_);
        }

        ros::ServiceClient recheck_client = n.serviceClient<object_marker::RecheckObject>("/object_marker/object_recheck");
        object_marker::RecheckObject srv;
        MoveBaseClient ac("move_base", true);
        move_base_msgs::MoveBaseGoal goal;

        // check for old objects
        for (int i = 0; i < existed_objects.objects.size(); i++)
        {
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = existed_objects.objects[i].transform.translation.x;
            goal.target_pose.pose.position.y = existed_objects.objects[i].transform.translation.y;
            goal.target_pose.pose.position.z = existed_objects.objects[i].transform.translation.z;
            goal.target_pose.pose.orientation = existed_objects.objects[i].transform.rotation;
            ac.sendGoal(goal);
            ac.waitForResult();
            if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                if (recheck_client.call(srv))
                {
                    if (srv.response.existence)
                    {
                        existed_objects_update.objects.push_back(existed_objects.objects[i]);
                        visualization_msgs::Marker marker_ = marker_publish(existed_objects.objects[i], i, 2);
                        marker_pub.publish(marker_);
                    }
                }
            }
            else
                ROS_INFO("The base failed to move for some reason");
        }
    }
    // TODO: situation for exit!!!!!!!!!!!!!!!!!!!!!!

    NodeWallFollowing *nodeWallFollowing = new NodeWallFollowing(pubMessage, client, existed_objects_update, WALL_DISTANCE, MAX_SPEED, DIRECTION, P, D, 1);

    ros::Subscriber sub = n.subscribe(SUBSCRIBER_TOPIC, SUBSCRIBER_BUFFER_SIZE, &NodeWallFollowing::messageCallback, nodeWallFollowing);

    ros::spin();

    return 0;
}