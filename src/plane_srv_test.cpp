#include <iostream>
#include <fstream>
#include <unistd.h>
#include <termios.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

#include <point_cloud_jackal/Plane.h>
#include <point_cloud_jackal/PlanarSegmentation.h>

char getch()
{
    /*#include <unistd.h>   //_getch*/
    /*#include <termios.h>  //_getch*/
    char buf = 0;
    struct termios old = {0};
    fflush(stdout);
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
        perror("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror("tcsetattr ~ICANON");
    //printf("%c\n", buf);
    return buf;
}

class ObjectMarker
{
  public:
    ObjectMarker(ros::NodeHandle n) : nh_(n)
    {
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("object_marking", 10);
        client_ = nh_.serviceClient<point_cloud_jackal::PlanarSegmentation>("planer_segment");
        srv_.request.center.x = 1; 
        srv_.request.center.y = 1; 
        srv_.request.center.z = 1; 
    }

    void marker_publish(point_cloud_jackal::Plane plane)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "/base_link";
        marker.header.stamp = ros::Time();
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = plane.center.x;
        marker.pose.position.y = plane.center.y;
        marker.pose.position.z = plane.center.z;
        marker.pose.orientation = this->calculate_quaternion(plane);
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.lifetime = ros::Duration();
        marker_pub_.publish(marker);
    }


    // calculate the quaternion rotation between two vector, up_vector and axis_vector
    geometry_msgs::Quaternion calculate_quaternion(point_cloud_jackal::Plane plane)
    {
        tf::Vector3 axis_vector(plane.normal[0], plane.normal[1], plane.normal[2]);
        tf::Vector3 up_vector(0.0, 0.0, 1.0);
        tf::Vector3 right_vector = axis_vector.cross(up_vector);
        right_vector.normalized();
        tf::Quaternion q(right_vector, -1.0*acos(axis_vector.dot(up_vector)));
        q.normalize();
        geometry_msgs::Quaternion cylinder_orientation;
        tf::quaternionTFToMsg(q, cylinder_orientation);
        return cylinder_orientation;
    }

    void doit()
    {
        while (ros::ok())
        {
            char key_press;
            key_press = getch();
            if (key_press == 'd')
            {
                ROS_INFO("Segment...");
                if (client_.call(srv_))
                {
                    ROS_INFO("Segmentation Success");
                    plane_s_ = srv_.response.plane_object;
                    this->marker_publish(plane_s_);
                }
                else
                {
                    ROS_INFO("Segmentation Fail");
                }
            }
            ros::spinOnce();
        }
    }

  private:
    ros::NodeHandle nh_;
    ros::Publisher marker_pub_;
    ros::ServiceClient client_;
    point_cloud_jackal::PlanarSegmentation srv_;
    point_cloud_jackal::Plane plane_s_;
};

main(int argc, char **argv)
{
    ros::init(argc, argv, "plane_seg_test_client");
    ros::NodeHandle n;

    ObjectMarker om(n);
    om.doit();

    return 0;
}