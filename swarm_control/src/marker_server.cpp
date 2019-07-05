#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <drone_msgs/Goal.h>

using namespace visualization_msgs;
using namespace drone_msgs;

// Global variables
boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server;

Goal goal;

double getYawFromQuat(geometry_msgs::Quaternion quat)
{
  tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  return yaw;
}


void updateGoal(geometry_msgs::Pose pose)
{
  goal.ctr_type = Goal::POSE;
  goal.pose.point.x = pose.position.x;
  goal.pose.point.y = pose.position.y;
  goal.pose.point.z = pose.position.z;
  goal.pose.course = getYawFromQuat(pose.orientation);
}

void processFeedback( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
{
  updateGoal(feedback->pose);
  server->applyChanges();
}

Marker arrowMarker( InteractiveMarker &msg )
{
  Marker marker;

  marker.type = Marker::ARROW;
  marker.scale.x = msg.scale * 0.75;
  marker.scale.y = msg.scale * 0.1;
  marker.scale.z = msg.scale * 0.1;
  marker.color.r = 0.5;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 0.8;
  return marker;
}

Marker textMarker( Goal &msg )
{
  Marker marker;

  std::string text_msg = "           x:" +std::to_string(msg.pose.point.x)+ "\n";
              text_msg += "           y:" +std::to_string(msg.pose.point.y)+ "\n";
              text_msg += "           z:" +std::to_string(msg.pose.point.z)+ "\n";
              text_msg += "           yaw:" +std::to_string(goal.pose.course * 57.2958)+ "\n";

  float _scale = 0.08;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "goal_text";
  marker.type = Marker::TEXT_VIEW_FACING;
  marker.text =  text_msg;
  marker.scale.x = _scale;
  marker.scale.y = _scale;
  marker.scale.z = _scale;
  marker.pose.position.x = msg.pose.point.x;
  marker.pose.position.y = msg.pose.point.y;
  marker.pose.position.z = msg.pose.point.z;

  marker.pose.position.z += 0.15;

  //marker.pose.orientation.w = 1.;

  marker.color.r = 0.8;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;
  return marker;
}

InteractiveMarkerControl& makeBoxControl( InteractiveMarker &msg )
{
  InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back( arrowMarker(msg) );
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

void goalUpdateCb(Goal goal_)
{
  goal = goal_;
  geometry_msgs::Pose pose;
  pose.position.x = goal_.pose.point.x;
  pose.position.y = goal_.pose.point.y;
  pose.position.z = goal_.pose.point.z;
  tf::Quaternion q = tf::createQuaternionFromYaw(goal_.pose.course);
  tf::quaternionTFToMsg(q, pose.orientation);
  server->setPose("goal_pose", pose);
  server->applyChanges();
}


int main(int argc, char** argv)
{
  // Init ROS node
  ros::init(argc, argv, "marker_server");
  ros::NodeHandle n;
  ros::Publisher goal_pub = n.advertise<Goal>("/goal_pose", 10);
  ros::Publisher marker_text_pub = n.advertise<Marker>("/basix_controls/marker_text", 10);

  ros::Subscriber goal_sub = n.subscribe("/goal_pose", 10, &goalUpdateCb);

  // Init marker server
  server.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );
  ros::Duration(0.1).sleep();
  makeQuadrocopterMarker(tf::Vector3(0, 0, 0));
  server->applyChanges();

  ros::Rate loop_rate(30);

  while (ros::ok())
  {
      marker_text_pub.publish(textMarker(goal));
      ros::spinOnce();
      loop_rate.sleep();
  }

  return 0;
}
