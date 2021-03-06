#include "ros/ros.h"
#include "std_msgs/String.h"
#include "curve_compression/geometryFunctions.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include "curve_compression/compressCurve.h"
#include "curve_compression/compressCurveUntil.h"

#include <sstream>

bool curveCompressionSvr3D(curve_compression::compressCurve::Request &req,
                           curve_compression::compressCurve::Response &res){
  
  //Get initial time to calculate solving time
  ros::Time t0 = ros::Time::now();

  //Declare initial variables
  int n_points = req.denseCurve.poses.size(); //Number of waypoints
  std::vector<double> x, y, z, t;
  int minPoints = req.minPoints.data;
  int compressionFactor = req.compressionFactor.data;
  int compression;
  ROS_INFO("Compression service! Initial points: %d  Minimum Points: %d  Compression: %d", n_points, minPoints,compressionFactor);

  //Check input data
  if(minPoints < 2){
    ROS_INFO("Minimum Points being set to 2!");
    minPoints = 2;
  }
  if((compressionFactor >= 1) && (compressionFactor <= 7)){
    compression = pow(2,compressionFactor);
  }
  else{
    ROS_INFO("Compression factor being set to 3!");
    compressionFactor = 3;
    compression = pow(2,compressionFactor);
  }


  //Populate xy matrix
  x.resize(n_points);
  y.resize(n_points);
  z.resize(n_points);
  t.resize(n_points);
  for (int i = 0; i < n_points; i++){
    x[i] = req.denseCurve.poses[i].pose.position.x;
    y[i] = req.denseCurve.poses[i].pose.position.y;
    z[i] = req.denseCurve.poses[i].pose.position.z;
    t[i] = req.denseCurve.poses[i].header.stamp.toSec();
  }

  //First delete colinear points
  double epsilon = 0.0025, dist;
  int deleteIndex;
  Eigen::Vector3d p1, p2, p;
  Line_3d line;
  while(n_points > minPoints){
    double deleteIndex = -1;
    for(int i = 1; i < n_points-1; i++){
      p1 << x[i-1], y[i-1], z[i-1];
      p2 << x[i+1], y[i+1], z[i+1];
      p  << x[i],   y[i], z[i];
      line = lineThroughPoints(p1,p2);
      dist = distancePoint2Line(p,line);
      if(dist < epsilon){
        deleteIndex = i;
        break;
      }
    }
    if(deleteIndex > 0){
      x.erase(x.begin() + deleteIndex);
      y.erase(y.begin() + deleteIndex);
      z.erase(z.begin() + deleteIndex);
      t.erase(t.begin() + deleteIndex);
      n_points = n_points - 1;
    }
    else{
      break;
    }
  }
  ROS_INFO("Number of non-colinear points: %d", n_points);

  //Now compress the remaining points
  bool flag = true;
  double minDist, initMinDist;
  // indexes = 1:n_points;
  while(n_points > minPoints){
    minDist = std::numeric_limits<float>::infinity();
    for(int i = 1; i < n_points-1; i++){
      p1 << x[i-1], y[i-1], z[i-1];
      p2 << x[i+1], y[i+1], z[i+1];
      p  << x[i],   y[i], z[i];
      line = lineThroughPoints(p1,p2);
      dist = distancePoint2Line(p,line);
      // ROS_INFO("dist: %f\t minDist: %f",dist,minDist);
      if(dist < minDist){
        minDist = dist;
        deleteIndex = i;
      }
    }

    if(flag){
      initMinDist = std::max(minDist,0.01);
      flag = false;
    }

    if(minDist > compression*initMinDist){
      break;
    }
    else{
      x.erase(x.begin() + deleteIndex);
      y.erase(y.begin() + deleteIndex);
      z.erase(z.begin() + deleteIndex);
      t.erase(t.begin() + deleteIndex);
      n_points = n_points - 1;
    }
  }

  //Return compressed values
  nav_msgs::Path CompressedCurve;
  geometry_msgs::PoseStamped Pos;
  CompressedCurve.poses.resize(n_points);
  for (int i = 0; i < n_points; i++){
    Pos.pose.position.x = x[i];
    Pos.pose.position.y = y[i];
    Pos.pose.position.z = z[i];
    Pos.header.stamp = ros::Time(t[i]);

    CompressedCurve.poses[i] = Pos;
  }

  res.compressedCurve = CompressedCurve;


  //Calculate solution time
  ros::Duration SolverTime = ros::Time::now() - t0;
  ROS_INFO("Compressed points: %d\tCalculation time: %f", n_points, SolverTime.toSec());

  return true;

}

bool curveCompressionUntilSvr3D(curve_compression::compressCurveUntil::Request &req,
                                curve_compression::compressCurveUntil::Response &res){
  
  //Get initial time to calculate solving time
  ros::Time t0 = ros::Time::now();

  //Declare initial variables
  int n_points = req.denseCurve.poses.size(); //Number of waypoints
  std::vector<double> x, y, z, t;
  int n_compressed_points = req.n_compressed_points.data;
  int compression;
  ROS_INFO("Compression service! Initial points: %d  Minimum Points: %d", n_points, n_compressed_points);

  //Check input data
  if(n_compressed_points < 2){
    ROS_INFO("Minimum Points being set to 2!");
    n_compressed_points = 2;
  }

  //Populate xy matrix
  x.resize(n_points);
  y.resize(n_points);
  z.resize(n_points);
  t.resize(n_points);
  for (int i = 0; i < n_points; i++){
    x[i] = req.denseCurve.poses[i].pose.position.x;
    y[i] = req.denseCurve.poses[i].pose.position.y;
    z[i] = req.denseCurve.poses[i].pose.position.z;
    t[i] = req.denseCurve.poses[i].header.stamp.toSec();
  }

  //First delete colinear points
  double epsilon = 0.0025, dist;
  int deleteIndex;
  Eigen::Vector3d p1, p2, p;
  Line_3d line;
  while(n_points > n_compressed_points){
    double deleteIndex = -1;
    for(int i = 1; i < n_points-1; i++){
      p1 << x[i-1], y[i-1], z[i-1];
      p2 << x[i+1], y[i+1], z[i+1];
      p  << x[i],   y[i], z[i];
      line = lineThroughPoints(p1,p2);
      dist = distancePoint2Line(p,line);
      if(dist < epsilon){
        deleteIndex = i;
        break;
      }
    }
    if(deleteIndex > 0){
      x.erase(x.begin() + deleteIndex);
      y.erase(y.begin() + deleteIndex);
      z.erase(z.begin() + deleteIndex);
      t.erase(t.begin() + deleteIndex);
      n_points = n_points - 1;
    }
    else{
      break;
    }
  }
  ROS_INFO("Number of non-colinear points: %d", n_points);

  //Now compress the remaining points
  bool flag = true;
  double minDist, initMinDist;
  // indexes = 1:n_points;
  while(n_points > n_compressed_points){
    minDist = std::numeric_limits<float>::infinity();
    for(int i = 1; i < n_points-1; i++){
      p1 << x[i-1], y[i-1], z[i-1];
      p2 << x[i+1], y[i+1], z[i+1];
      p  << x[i],   y[i], z[i];
      line = lineThroughPoints(p1,p2);
      dist = distancePoint2Line(p,line);
      // ROS_INFO("dist: %f\t minDist: %f",dist,minDist);
      if(dist < minDist){
        minDist = dist;
        deleteIndex = i;
      }
    }

    if(flag){
      initMinDist = std::max(minDist,0.01);
      flag = false;
    }

    x.erase(x.begin() + deleteIndex);
    y.erase(y.begin() + deleteIndex);
    z.erase(z.begin() + deleteIndex);
    t.erase(t.begin() + deleteIndex);
    n_points = n_points - 1;
  }

  //Return compressed values
  nav_msgs::Path CompressedCurve;
  geometry_msgs::PoseStamped Pos;
  CompressedCurve.poses.resize(n_points);
  for (int i = 0; i < n_points; i++){
    Pos.pose.position.x = x[i];
    Pos.pose.position.y = y[i];
    Pos.pose.position.z = z[i];
    Pos.header.stamp = ros::Time(t[i]);

    CompressedCurve.poses[i] = Pos;
  }

  res.compressedCurve = CompressedCurve;


  //Calculate solution time
  ros::Duration SolverTime = ros::Time::now() - t0;
  ROS_INFO("Compressed points: %d\tCalculation time: %f", n_points, SolverTime.toSec());

  return true;

}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  //Initialize ROS
  ros::init(argc, argv, "curveCompressionServices");
  ros::NodeHandle n;

  //Run services
  ROS_INFO("Starting compression services...");
  ros::ServiceServer compression_Srv = n.advertiseService("/curveCompression", curveCompressionSvr3D);
  ros::ServiceServer compressionUntil_Srv = n.advertiseService("/curveCompressionUntil", curveCompressionUntilSvr3D);
  ROS_INFO("Compression services started!");

  ros::Rate loop_rate(10);
  ros::spinOnce();



  while (ros::ok())
  {
  
    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}