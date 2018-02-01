#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/Matrix3x3.h>
#include <geometry_msgs/TransformStamped.h>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/CheckForObjectsAction.h>

#include <vector>
#include <std_msgs/Int32.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <point_cloud_jackal/Planes.h>
#include <point_cloud_jackal/Plane.h>

#include <object_marker/Object.h>


using namespace std;

class object_marker
{
public:
  object_marker(ros::NodeHandle n) : nh_(n)
  {
    info_sub_ = nh_.subscribe("camera/depth_registered/camera_info", 10, &object_marker::info_callback, this);
    image_sub_ = nh_.subscribe("camera/depth_registered/image_raw", 10, &object_marker::depthcb, this);
    yolo_sub_ = nh_.subscribe<darknet_ros_msgs::BoundingBoxes>("darknet_ros/bounding_boxes", 1, &object_marker::yolo_callback, this);
    save_sub_ = nh_.subscribe<std_msgs::Int32>("save_status", 1, &object_marker::save_callback, this);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("object_marker", 10);
    planar_segment_client_ = nh_.serviceClient<point_cloud_jackal::PlanarSegmentation>("planer_segment");
  }

  void info_callback(const sensor_msgs::CameraInfoConstPtr &msg)
  {
    model1_.fromCameraInfo(msg);
  }

  void yolo_callback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg)
  {
    pixel_x_.clear(); //column
    pixel_y_.clear(); //row
    temp_count_ = 0;
    int size = msg->boundingBoxes.size();
    for (int i = 0; i < size; i++)
    {
      if (msg->boundingBoxes[i].Class.compare("tag") == 0)
      {
        pixel_x_.push_back((msg->boundingBoxes[i].xmin + msg->boundingBoxes[i].xmax) / 2);
        pixel_y_.push_back((msg->boundingBoxes[i].ymin + msg->boundingBoxes[i].ymax) / 2);
        temp_count_ = temp_count_ + 1;
        isYoloUpdated_ = true;
      }
    }
    //ROS_INFO ("\t(%d, %d)\n", pixel_x[0], pixel_y[0]);  //test use
  }

  void depthcb(const sensor_msgs::ImageConstPtr &msg)
  {
    if (isYoloUpdated_)
    {
      coordinate_camera_.resize(temp_count_);
      cv_bridge::CvImageConstPtr depth_img_cv;
      depth_img_cv = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
      for (int i = 0; i < temp_count_; i++)
      {
        cv::Point2d pixel_point(pixel_x_[i], pixel_y_[i]);
        float depth = depth_img_cv->image.at<short int>(pixel_point);
        cv::Point3d xyz = model1_.projectPixelTo3dRay(pixel_point);
        cv::Point3d coordinate = xyz * depth;
        if (depth > 0.01)
        {
          coordinate_camera_[i] = coordinate / 1000;
        }
      }
    }
    //ROS_INFO("\t(%f, %f, %f)\n", co_x[0], co_y[0], co_z[0]); //test use
  }

  void marker_publish(object_marker::Objects objects_to_be_pub)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = counts_;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = objects_to_be_pub.back().transform.getOrigin().x();
    marker.pose.position.y = objects_to_be_pub.back().transform.getOrigin().y();
    marker.pose.position.z = objects_to_be_pub.back().transform.getOrigin().z();
    marker.pose.orientation.x = objects_to_be_pub.back().transform.getRotation().x();
    marker.pose.orientation.y = objects_to_be_pub.back().transform.getRotation().y()
    marker.pose.orientation.z = objects_to_be_pub.back().transform.getRotation().z()
    marker.pose.orientation.w = objects_to_be_pub.back().transform.getRotation().w()
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

  tf::Transform tf_calculation(float x, float y, float z)
  {
    //calculate transform from /map to /object
    tf::TransformListener listener;
    float normal[3];
    tf::StampedTransform stampedtransform;
    //ros::Rate rate(5.0);
    //rate.sleep();
    listener.waitForTransform("/map", "/camera_rgb_optical_frame", ros::Time::now(), ros::Duration(3.0));
    listener.lookupTransform("/map", "/camera_rgb_optical_frame", ros::Time(0), stampedtransform);
    tf::Transform transform;
    if (planar_segment_client_(planar_segment_srv_))
    {
      ROS_INFO("Get surface normal");
      normal[1] = planar_segment_srv_.response.plane_object.normal[1];
      normal[2] = planar_segment_srv_.response.plane_object.normal[2];
      normal[3] = planar_segment_srv_.response.plane_object.normal[3];
    }
    // convert calculated coordinate information to tf transform, pay attention to x-y-z axis,
    // will be used to add cylinder marker in rviz
    tf::Matrix3x3 rotation_matrix(0,0,1,
                                  sqrt(1-normal[1]),sqrt(1-normal[2]),0
                                  normal[1], normal[2], 0]);
    transform.setOrigin(tf::Vector3(x, y, z));
    transform.setRotation(rotation_matrix.getRotation);
    tf::Transform ntransform;
    ntransform = stampedtransform * transform;

    //float object_x = transform.getOrigin().x();
    //float object_y = transform.getOrigin().y(); //position of object in 2d map //to be saved
    //float result = {object_x, object_y};
    //return result;
    return ntransform;
  }

  void save_callback(const std_msgs::Int32ConstPtr &msg)
  {
    geometry_msgs::Point p;
    int lock_count = temp_count_;
    for (int i = 0; i < lock_count; i++)
    {
      fx = coordinate_camera_[i].x;
      fy = coordinate_camera_[i].y;
      fz = coordinate_camera_[i].z;
      tf::Transform transform = this->tf_calculation(coordinate_camera_[i].x, coordinate_camera_[i].y, coordinate_camera_[i].z);
      object_marker::Object single_object;
      single_object.transform = transform;
      detect_objects_.push_back(single_object);
      counts_ = detect_objects_.size();
      this->marker_publish(detect_objects_);
      // an array[ID, x, y] to be save
      float object_pose_orientation[7] = {detect_objects_.back().transform.getOrigin().x(), detect_objects_.back().transform.getOrigin().y(), detect_objects_.back().transform.getOrigin().z(),
                                          detect_objects_.back().transform.getRotation().x(), detect_objects_.back().transform.getRotation().y(), detect_objects_.back().transform.getRotation().z(), detect_objects_.back().transform.getRotation().w()};
      ofstream out;
      out.open("/home/mzwang/Desktop/objects.txt", ios_base::app | ios_base::out);
      for (int i = 0; i < 7; i++)
      {
        out << object_pose_orientation[i] << " ";
      }
      out << "\n";
      out.close();

      
    }

    cout << "Detected Object Saved\n";
    cout << "New saved " << lock_count << " objects, total saved " << counts_ << " objects.\n";
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber info_sub_, image_sub_, yolo_sub_, save_sub_;
  ros::Publisher marker_pub_;
  ros::ServiceClient planar_segment_client_;
  point_cloud_jackal::PlanarSegmentation planar_segment_srv_;
  image_geometry::PinholeCameraModel model1_;
  object_marker::Objects detect_objects_;

  vector<int> pixel_x_, pixel_y_;
  vector<cv::Point3d> coordinate_camera_;
  int temp_count_, counts_;
  bool isYoloUpdated_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_marker");
  ros::NodeHandle n('~');

  ros::spin();
  return 0;
}