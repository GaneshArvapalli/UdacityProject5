#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

std::vector<double> robot_pos{0, 0};

void robot_pos_callback(const nav_msgs::Odometry::ConstPtr& msg) {
	// ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
	// ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 
    //         msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	robot_pos[0] = msg->pose.pose.position.x;
	robot_pos[1] = msg->pose.pose.position.y;
	// ROS_INFO("Currently at Position-> x: [%f], y: [%f].", robot_pos[0], robot_pos[1]);
}

int main(int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  ros::Subscriber odom_sub = n.subscribe("/odom", 10, robot_pos_callback);
  
  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  
  while (ros::ok())
  {    
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = -1.0;
    marker.pose.position.y = 1.0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
    ROS_INFO("Object Published. Waiting for robot. Robot is at Position-> x: [%f], y: [%f]. Marker is at Position-> x: [%f], y: [%f].", robot_pos[0], robot_pos[1], marker.pose.position.x, marker.pose.position.y);

    // Wait until the robot reaches the pickup
    std::cout << "Waiting to reach pickup" << std::endl;
    while((robot_pos[0] - marker.pose.position.x  > 0.001) || (robot_pos[1] - marker.pose.position.y  > 0.001)) {
      ros::Duration(1).sleep();
      marker_pub.publish(marker);
      ROS_INFO("Sleeping for 1 second. Robot is at Position-> x: [%f], y: [%f]. Marker is at Position-> x: [%f], y: [%f].", robot_pos[0], robot_pos[1], marker.pose.position.x, marker.pose.position.y);
      ros::spinOnce();
    }
    
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::DELETE;
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    // Wait 5 sec to simulate pickup
    ROS_INFO("Picking up object. Robot is at Position-> x: [%f], y: [%f]. Marker is at Position-> x: [%f], y: [%f].", robot_pos[0], robot_pos[1], marker.pose.position.x, marker.pose.position.y);
    sleep(5);
    marker_pub.publish(marker);
    ROS_INFO("Object picked up. Robot is at Position-> x: [%f], y: [%f]. Marker is at Position-> x: [%f], y: [%f].", robot_pos[0], robot_pos[1], marker.pose.position.x, marker.pose.position.y);
    
    marker.header.stamp = ros::Time::now();
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();
    
    // Wait until the robot reaches the dropoff
    // ROS_INFO("Waiting Boolean has value: " + (robot_pos[0] != marker.pose.position.x || robot_pos[1] != marker.pose.position.y) ? "true" : "false");
    while((robot_pos[0] - marker.pose.position.x  > 0.001) || (robot_pos[1] - marker.pose.position.y  > 0.001)) {
      ros::Duration(1).sleep();
      // marker_pub.publish(marker);
      ROS_INFO("Sleeping for 1 second on the way to delivery. Robot is at Position-> x: [%f], y: [%f]. Marker is at Position-> x: [%f], y: [%f].", robot_pos[0], robot_pos[1], marker.pose.position.x, marker.pose.position.y);
      ros::spinOnce();
    }

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
    ROS_INFO("Dropping off object. Robot is at Position-> x: [%f], y: [%f]. Marker is at Position-> x: [%f], y: [%f].", robot_pos[0], robot_pos[1], marker.pose.position.x, marker.pose.position.y);
    
    // Let's just leave the marker there for 10 seconds so we can see it before deletion
    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    sleep(10);
    marker.action = visualization_msgs::Marker::DELETE;
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    break;
    

    r.sleep();
  }
  ros::spin();
}