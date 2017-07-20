#include "ros/ros.h"
#include "std_msgs/String.h"
#include "curve_compression/geometryFunctions.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include "curve_compression/compressCurve.h"

#include <sstream>


/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  //Initialize ROS
  ros::init(argc, argv, "curveCompressionClient");
  ros::NodeHandle n;

  ros::Rate loop_rate(10);

  //Get some example data
  int n_points = 1000;
  double tf = 2*M_PI;
  double t;

  nav_msgs::Path Curve;
  geometry_msgs::PoseStamped Pos;
  for (int i = 0; i < n_points; i++){
    t = tf*double(i)/double(n_points-1);
    Pos.pose.position.x = sin(t);
    Pos.pose.position.y = cos(t);
    Pos.pose.position.z = 0;
    Pos.header.stamp = ros::Time::now();

    Curve.poses.push_back(Pos);
  }

  //Get a client to call service
  ros::ServiceClient client0 = n.serviceClient<curve_compression::compressCurve>("/curveCompression2D");
  curve_compression::compressCurve srv;
  srv.request.denseCurve = Curve;
  srv.request.minPoints.data = 5;
  srv.request.compressionFactor.data = 3;
  client0.call(srv);

  //Print output points
  int outPoints = srv.response.compressedCurve.poses.size();
  geometry_msgs::Point Point;
  for(int i = 0; i < outPoints; i++){
    Point = srv.response.compressedCurve.poses[i].pose.position;
    ROS_INFO("Point %d: x: %f\ty: %f\tz: %f\t", i+1, Point.x, Point.y, Point.z);
  }




  while (ros::ok())
  {
  
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}