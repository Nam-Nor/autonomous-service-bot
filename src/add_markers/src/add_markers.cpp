#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <math.h>

double poseAMCLx, poseAMCLy, poseAMCLw;
// Callback method for the Robot's AMCL pose subscriber
void poseAMCLCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msgAMCL)
{
    poseAMCLx = msgAMCL->pose.pose.position.x;
    poseAMCLy = msgAMCL->pose.pose.position.y;
    poseAMCLw = msgAMCL->pose.pose.orientation.w; 
}

// Helper method to find euclidean distance
double distance(double x1, double y1, double x2, double y2)
{
  return sqrt(pow(x1-x2,2)+pow(y1-y2,2));
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  // Subscriber to the AMCL pose
  ros::Subscriber sub_amcl = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, poseAMCLCallback);
  
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  double goal_x, goal_y, goal_w;
  enum States{
    pickup,
    transit,
    delivery
  };
  States current_st = pickup; // Denote the current state for behavior evaluation

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    // Finite state machine to handle marker behavior
    switch (current_st)
    {
    case pickup:
      // Add marker to pickup location
      marker.action = visualization_msgs::Marker::ADD;
      ROS_INFO_ONCE("Navigating to pickup...");
      goal_x = 3.0;
      goal_y = 0.5;
      goal_w = 1.0;
      // Check to see if robot is within range of pickup location
      if(distance(goal_x, goal_y, poseAMCLx, poseAMCLy) < 0.2 )
      	current_st = transit;
      break;
    case transit:
      // Delete marker from map
      marker.action = visualization_msgs::Marker::DELETE;
      ROS_INFO_ONCE("Pickup complete. Waiting for delivery...");
      goal_x = -2.0;
      goal_y = -1.0;
      goal_w = 1.0;
      // Check to see if robot is within range of delivery location
      if(distance(goal_x, goal_y, poseAMCLx, poseAMCLy) < 0.2 )
      	current_st = delivery;
      break;
    case delivery:
      // Add marker to delivery location
      marker.action = visualization_msgs::Marker::ADD;
      ROS_INFO_ONCE("Package delivered!");
      break;
    }
    
    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = goal_x;
    marker.pose.position.y = goal_y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = goal_w;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // Set the color to blue
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    
    // Publish the marker
    marker_pub.publish(marker);
   
    r.sleep();
    
    // Handle ROS communication events
  	ros::spinOnce();
  }
}