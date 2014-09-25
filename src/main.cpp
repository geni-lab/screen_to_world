#include <stdlib.h>
#include <iostream>

#include <DepthSense.hxx>

#include "ros/ros.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/UInt16MultiArray.h"
#include "std_msgs/Float32MultiArray.h"
#include "geometry_msgs/Point.h"
//#include "targets.h"
#include "geometry_msgs/PoseStamped.h"
#include <scale_face_point/User.h>
#include <scale_face_point/UserList.h>

#include "tf/transform_broadcaster.h"
#include <sstream>
#include <iostream>

using namespace std;
using namespace DepthSense;
using namespace scale_face_point;

ros::Publisher *coordPublishFace;
ros::Subscriber *coordSubscribeFace;
ros::Publisher *coordPublishSaliency;
ros::Subscriber *coordSubscribeSaliency;
ros::Publisher *coordPublishPersonIDs;

Context context;
float cx;
float cy;
float fx;
float fy;

float d = 1.f;//TODO: Change the fixed depth to something sensible

UserList user_list;
tf::TransformBroadcaster *br;

void setupDeviceAndGlobals(void)
{
    // create a connection to the DepthSense server at localhost
    context = Context::create();

    // obtain the list of devices attached to the host
    vector<Device> devices = context.getDevices();
    StereoCameraParameters stereo;

    if (devices.size() != 0)
    {
        for(int i = 0; i < devices.size(); i++)
        {
            stereo=devices[i].getStereoCameraParameters();

        }
    }

    // XXX since we are getting face coordinate from the color camera
    // might want the intrinsics parameters of the color camera instead

    fx = stereo.depthIntrinsics.fx;
    fy = stereo.depthIntrinsics.fy;

    cx = stereo.depthIntrinsics.cx;
    cy = stereo.depthIntrinsics.cy;
}

UserList coordCallbackMain(const UserListConstPtr& msg)
{
    int size = msg->users.size();

    UserList outMsg;

    for(int i = 0; i < size; ++i)
    {
        geometry_msgs::Point input = msg->users[i].head.pose.position;
        User output = msg->users[i];

        float u = input.x;
        float v = input.y;

        // XXX the best depth would be the corresponding depth value from the depth map

        ROS_INFO("I heard: [%f, %f]", u, v);

        output.head.pose.position.x = d * (u - cx)/fx;
        output.head.pose.position.y = d * (v - cy)/fy;
        output.head.pose.position.z = d;

        outMsg.users.push_back(output);
    }

    return outMsg;
}

void coordCallbackFace(const UserListConstPtr& msg)
{
  std::cerr << "coordCallbackFace begin..." << std::endl;

  UserList outMsg = coordCallbackMain(msg);

  coordPublishFace->publish(outMsg);
  
  int size = outMsg.users.size();
  
  std_msgs::UInt16MultiArray user_list_msg;
  
  for(int i = 0; i < size; ++i)
  {
    tf::Transform transform;
    geometry_msgs::Point mPoint = outMsg.users[i].head.pose.position;
    
    transform.setOrigin( tf::Vector3(mPoint.x, mPoint.y, mPoint.z) );
    
    geometry_msgs::Quaternion mQuaternion = outMsg.users[i].head.pose.orientation;
    tf::Quaternion q(mQuaternion.x, mQuaternion.y, mQuaternion.z, mQuaternion.w);
    
    transform.setRotation(q);
    
    std::stringstream child_id, frame_id;
    child_id << "Person" << i << "_head";
    frame_id << outMsg.users[i].user_id;
    
    br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id.str(), child_id.str()));
    
    user_list_msg.data.push_back(outMsg.users[i].user_id);
  }
  
  coordPublishPersonIDs->publish(user_list_msg);

  std::cerr << "coordCallbackFace done..." << std::endl;  
}

void coordCallbackSaliency(const UserListConstPtr& msg)
{
  UserList outMsg = coordCallbackMain(msg);

  coordPublishSaliency->publish(outMsg);
}

//Here a more complete example (working with DS325):

int main (int argc, char** argv)
{
    std::cerr << "Before Init..." << std::endl;

    ros::init(argc, argv, "scale_face_point");

    std::cerr << "After Init..." << std::endl;

    setupDeviceAndGlobals();


    ros::NodeHandle n;

    coordPublishFace = new ros::Publisher();
    coordSubscribeFace = new ros::Subscriber();
    coordPublishSaliency = new ros::Publisher();
    coordSubscribeSaliency = new ros::Subscriber();
    coordPublishPersonIDs = new ros::Publisher();

    br = new tf::TransformBroadcaster();

    *coordSubscribeFace = n.subscribe("facedetect", 16, coordCallbackFace);
    *coordSubscribeSaliency = n.subscribe("/nmpt_saliency_point", 16, coordCallbackSaliency);
    *coordPublishFace = n.advertise<UserList>("facedetect_world", 16);
    *coordPublishSaliency = n.advertise<UserList>("/nmpt_saliency_point_world", 16);
    *coordPublishPersonIDs = n.advertise<std_msgs::UInt16MultiArray>("simple_face_tracker/user_list", 16);

    ros::spin();

    delete coordPublishFace;
    delete coordSubscribeFace;
    delete coordPublishSaliency;
    delete coordSubscribeSaliency;
    delete coordPublishPersonIDs;

    delete br;

    return 0;
}
