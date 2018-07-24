#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>


using namespace visualization_msgs;
using namespace geometry_msgs;

// Global variables
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;
ros::Publisher goal_pub;
PoseStamped goal;

double getYawFromQuat(geometry_msgs::Quaternion quat)
{
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  return yaw;
}


void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  goal.pose = feedback->pose;
  server->applyChanges();
}

Marker makeBox( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::ARROW;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.2;
  marker.scale.z = msg.scale * 0.2;
  marker.color.r = 0.5;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( makeBox(msg) );
  msg.controls.push_back( control );

  return msg.controls.back();
}

void makeQuadrocopterMarker( const tf::Vector3& position)
{
  InteractiveMarker int_marker;
  int_marker.header.frame_id = "map";
  tf::pointTFToMsg(position, int_marker.pose.position);
  int_marker.scale = 1;

  int_marker.name = "goal_pose";
  int_marker.description = "GOAL";

  makeBoxControl(int_marker);

  InteractiveMarkerControl control;

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = InteractiveMarkerControl::MOVE_ROTATE;
  int_marker.controls.push_back(control);
  control.interaction_mode = InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  server->insert(int_marker);
  server->setCallback(int_marker.name, &processFeedback);
}

void goalUpdateCb(PoseStamped goal_)
{
  goal.pose = goal_.pose;
  server->setPose("goal_pose", goal_.pose);
  server->applyChanges();
}


int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "marker_server");
  ros::NodeHandle n;
  goal_pub = n.advertise<PoseStamped>("/goal", 1000);
  ros::Subscriber goal_sub = n.subscribe("/goal", 10, &goalUpdateCb);

  // Init marker server
  server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );
  ros::Duration(0.1).sleep();
  makeQuadrocopterMarker(tf::Vector3(0, 0, 0));
  server->applyChanges();

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
      goal.header.frame_id = "map";
      goal.header.stamp = ros::Time::now();
      goal_pub.publish(goal);

      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
