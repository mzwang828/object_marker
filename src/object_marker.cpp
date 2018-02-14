#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Matrix3x3.h>
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

#include <point_cloud_jackal/Plane.h>
#include <point_cloud_jackal/PlanarSegmentation.h>

#include <object_marker/Object.h>
#include <object_marker/Objects.h>

using namespace std;

class ObjectMarker
{
public:
  ObjectMarker(ros::NodeHandle n) : nh_(n)
  {
    info_sub_ = nh_.subscribe("camera/depth_registered/camera_info", 10, &ObjectMarker::info_callback, this);
    image_sub_ = nh_.subscribe("camera/depth_registered/image_raw", 10, &ObjectMarker::depthcb, this);
    yolo_sub_ = nh_.subscribe<darknet_ros_msgs::BoundingBoxes>("darknet_ros/bounding_boxes", 1, &ObjectMarker::yolo_callback, this);
    save_sub_ = nh_.subscribe<std_msgs::Int32>("save_status", 1, &ObjectMarker::save_callback, this);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("object_marking", 10);
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
    marker.pose.position.x = objects_to_be_pub.objects.back().transform.translation.x;
    marker.pose.position.y = objects_to_be_pub.objects.back().transform.translation.y;
    marker.pose.position.z = objects_to_be_pub.objects.back().transform.translation.z;
    marker.pose.orientation = objects_to_be_pub.objects.back().transform.rotation;
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
    tf::Quaternion q(right_vector, -1.0 * acos(axis_vector.dot(up_vector)));
    q.normalize();
    geometry_msgs::Quaternion cylinder_orientation;
    tf::quaternionTFToMsg(q, cylinder_orientation);
    return cylinder_orientation;
  }

  object_marker::Object object_calculation(float x, float y, float z)
  {
    //calculate transform from /map to /object
    tf::TransformListener listener;
    tf::StampedTransform stampedtransform;

    listener.waitForTransform("/map", "/camera_rgb_optical_frame", ros::Time::now(), ros::Duration(3.0));
    listener.lookupTransform("/map", "/camera_rgb_optical_frame", ros::Time(0), stampedtransform);
    tf::Transform transform;
    // convert calculated coordinate information to tf transform
    transform.setOrigin(tf::Vector3(x, y, z));
    tf::Transform ntransform;
    ntransform = stampedtransform * transform;
    // calculate quaternion of cylinder axis
    geometry_msgs::Quaternion cylinder_orientation;
    geometry_msgs::Point object_location;
    object_location.x = ntransform.getOrigin().x();
    object_location.y = ntransform.getOrigin().y();
    object_location.z = ntransform.getOrigin().z();
    planar_segment_srv_.request.center = object_location;
    if (planar_segment_client_.call(planar_segment_srv_))
    {
      ROS_INFO("Get surface normal");
      plane_s_ = planar_segment_srv_.response.plane_object;
      cylinder_orientation = this->calculate_quaternion(plane_s_);
    }

    // create the object marker
    object_marker::Object single_object;
    single_object.transform.translation.x = ntransform.getOrigin().x();
    single_object.transform.translation.y = ntransform.getOrigin().y();
    single_object.transform.translation.z = ntransform.getOrigin().z();
    single_object.transform.rotation = cylinder_orientation;

    //float object_x = transform.getOrigin().x();
    //float object_y = transform.getOrigin().y(); //position of object in 2d map //to be saved
    //float result = {object_x, object_y};
    //return result;

    return single_object;
  }

  void save_callback(const std_msgs::Int32ConstPtr &msg)
  {
    geometry_msgs::Point p;
    int lock_count = temp_count_;
    for (int i = 0; i < lock_count; i++)
    {
      // fx = coordinate_camera_[i].x;
      // fy = coordinate_camera_[i].y;
      // fz = coordinate_camera_[i].z;
      object_marker::Object single_object = this->object_calculation(coordinate_camera_[i].x, coordinate_camera_[i].y, coordinate_camera_[i].z);
      // object_marker::Object single_object;
      // single_object.transform.translation.x = transform.getOrigin().x();
      // single_object.transform.translation.y = transform.getOrigin().y();
      // single_object.transform.translation.z = transform.getOrigin().z();
      // single_object.transform.rotation.x = transform.getRotation().x();
      // single_object.transform.rotation.y = transform.getRotation().y();
      // single_object.transform.rotation.z = transform.getRotation().z();
      // single_object.transform.rotation.w = transform.getRotation().w();
      detect_objects_.objects.push_back(single_object);
      counts_ = detect_objects_.objects.size();
      this->marker_publish(detect_objects_);
      // an array[ID, x, y] to be save
      float object_pose_orientation[7] = {detect_objects_.objects.back().transform.translation.x, detect_objects_.objects.back().transform.translation.y, detect_objects_.objects.back().transform.translation.z,
                                          detect_objects_.objects.back().transform.rotation.x, detect_objects_.objects.back().transform.rotation.y, detect_objects_.objects.back().transform.rotation.z, detect_objects_.objects.back().transform.rotation.w};
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
  point_cloud_jackal::Plane plane_s_;
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
  ros::NodeHandle n("~");
  ObjectMarker om(n);

  ros::spin();
  return 0;
}