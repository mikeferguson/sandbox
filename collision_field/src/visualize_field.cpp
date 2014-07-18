/*
 * Copyright 2014 Michael E. Ferguson
 */

#include <ros/ros.h>
#include <geometric_shapes/shapes.h>
#include <moveit/distance_field/propagation_distance_field.h>

int main( int argc, char* argv[] )
{
  // specs
  double w = 0.9;
  double d = 0.45;
  std::vector<double> h;
  h.push_back(0);
  h.push_back(0.5);
  h.push_back(1.0);

  ros::init( argc, argv, "visualize_field");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");

  //nh.param("min_range", min_range_, 0.5);
  //nh.param("max_range", max_range_, 5.5);

  ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("field", 1, true /* latched */);

  ros::Time t = ros::Time::now();
  distance_field::PropagationDistanceField field(2.0, 2.0, 2.0,   /* x, y, z */
                                                 0.02,           /* resolution */
                                                 -0.5, -1.0, 0.0, /* origin */
                                                 2.0);            /* max distance (all cells initialized to this) */
  std::cout << "Creation time: " << (ros::Time::now() - t).toSec() << std::endl;


  std::cout << "Cells in x direction: " << field.getXNumCells() << std::endl;

  t = ros::Time::now();
  for (size_t i = 0; i < h.size(); ++i)
  {
    // This would be filled in by the robotstate
    shapes::Shape * box = new shapes::Box(d, w, 0.05);
    geometry_msgs::Pose pose;
    pose.position.x = 0.75;
    pose.position.y = 0.0;
    pose.position.z = h[i] + 0.05/2;  // raise half the thickness of the shelf
    pose.orientation.w = 1;
    field.addShapeToField(box, pose);
  }
  std::cout << "Update time: " << (ros::Time::now() - t).toSec() << std::endl;

  visualization_msgs::Marker marker;
  field.getIsoSurfaceMarkers(0.0, 0.0, "base_link", ros::Time::now(), marker);
  marker.color.a = 1.0;  // solid, since rviz seems busted

  pub.publish(marker);
  ros::spin();
}
