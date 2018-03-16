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
#include <std_msgs/Int8.h>
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
#include <object_marker/MarkObject.h>
#include <object_marker/RecheckObject.h>

using namespace std;

class ObjectMarker
{
public:
  ObjectMarker(ros::NodeHandle n) : nh_(n), yolo_msgs_(new darknet_ros_msgs::BoundingBoxes), depth_msgs_(new sensor_msgs::Image), depth_img_cv_(new cv_bridge::CvImage)
  {
    info_sub_ = nh_.subscribe("/kinect2/qhd/camera_info", 10, &ObjectMarker::info_callback, this);
    image_sub_ = nh_.subscribe("/kinect2/qhd/image_depth_rect", 10, &ObjectMarker::depthcb, this);
    yolo_sub_ = nh_.subscribe<darknet_ros_msgs::BoundingBoxes>("/darknet_ros/bounding_boxes", 1, &ObjectMarker::yolo_callback, this);
    yolo_number_sub_ = nh_.subscribe<std_msgs::Int8>("/darknet_ros/found_object", 1, &ObjectMarker::yolo_number_callback, this);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("object_marking", 10);
    planar_segment_client_ = nh_.serviceClient<point_cloud_jackal::PlanarSegmentation>("/planer_segment");
    object_marker_src_ = nh_.advertiseService("object_mark", &ObjectMarker::markObjectCB, this);
    object_recheck_ = nh_.advertiseService("object_recheck", &ObjectMarker::RecheckObjectCB, this);

    // set parameter
    distanceThreshold_ = 0.1;
  }

  void info_callback(const sensor_msgs::CameraInfoConstPtr &msg)
  {
    model1_.fromCameraInfo(msg);
  }

  void yolo_callback(const darknet_ros_msgs::BoundingBoxesConstPtr &msg)
  {
    yolo_msgs_ = msg;
  }

  void yolo_number_callback(const std_msgs::Int8ConstPtr &msg)
  {
    if (msg->data == 0)
    isYoloCalled_ = false;
    else
    isYoloCalled_ = true;
  }

  void depthcb(const sensor_msgs::ImageConstPtr &msg)
  {
    depth_img_cv_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    //depth_msgs_ = msg;
  }

  bool rgbProc()
  {
    pixel_x_.clear(); //column
    pixel_y_.clear(); //row
    temp_count_ = 0;
    int size = yolo_msgs_->boundingBoxes.size();
    ROS_INFO("%d", size);
    for (int i = 0; i < size; i++)
    {
      if (yolo_msgs_->boundingBoxes[i].Class.compare("cat") == 0)
      {
        pixel_x_.push_back((yolo_msgs_->boundingBoxes[i].xmin + yolo_msgs_->boundingBoxes[i].xmax) / 2);
        pixel_y_.push_back((yolo_msgs_->boundingBoxes[i].ymin + yolo_msgs_->boundingBoxes[i].ymax) / 2);
        temp_count_ = temp_count_ + 1;
        isYoloUpdated_ = true;
      }
    }
    //ROS_INFO ("\t(%d, %d)\n", pixel_x[0], pixel_y[0]);  //test use
    if (isYoloCalled_ == true && temp_count_ != 0)
    { 
      return true;
    }
    else
    {
    return false;
    }
  }

  bool depthProc()
  {
    if (isYoloUpdated_)
    {
      coordinate_camera_.resize(temp_count_);
      //cv_bridge::CvImageConstPtr depth_img_cv;
      //depth_img_cv = cv_bridge::toCvCopy(depth_msgs_, sensor_msgs::image_encodings::TYPE_16UC1);
      for (int i = 0; i < temp_count_; i++)
      {
        cv::Point2d pixel_point(pixel_x_[i], pixel_y_[i]);
        float depth = depth_img_cv_->image.at<short int>(pixel_point);
        cv::Point3d xyz = model1_.projectPixelTo3dRay(pixel_point);
        cv::Point3d coordinate = xyz * depth;
        if (depth > 0.01)
        {
          coordinate_camera_[i] = coordinate / 1000;
        }
      }
    }

    return true;
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
    // create a tf transform to translate the face of cylinder to detected position
    tf::Transform marker_transform;
    marker_transform.setOrigin(tf::Vector3(objects_to_be_pub.objects.back().transform.translation.x,
                                           objects_to_be_pub.objects.back().transform.translation.y,
                                           objects_to_be_pub.objects.back().transform.translation.z));
    marker_transform.setRotation(tf::Quaternion(objects_to_be_pub.objects.back().transform.rotation.x,
                                                objects_to_be_pub.objects.back().transform.rotation.y,
                                                objects_to_be_pub.objects.back().transform.rotation.z,
                                                objects_to_be_pub.objects.back().transform.rotation.w));
    tf::Vector3 cylinder_face(0, 0, 0.6);
    cylinder_face = marker_transform * cylinder_face;
    marker.pose.position.x = cylinder_face.getX();
    marker.pose.position.y = cylinder_face.getY();
    marker.pose.position.z = cylinder_face.getZ();
    marker.pose.orientation = objects_to_be_pub.objects.back().transform.rotation;
    marker.scale.x = 0.32;
    marker.scale.y = 0.32;
    marker.scale.z = 1.2;
    marker.color.a = 0.8; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration();
    marker_pub_.publish(marker);
    ROS_INFO("position; %f,%f,%f", marker.pose.position.x, marker.pose.position.y, marker.pose.position.z);
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

    listener.waitForTransform("/map", "/kinect2_rgb_optical_frame", ros::Time::now(), ros::Duration(3.0));
    listener.lookupTransform("/map", "/kinect2_rgb_optical_frame", ros::Time(0), stampedtransform);
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
    object_location.z = ntransform.getOrigin().z(); // xyz is in /map frame
    planar_segment_srv_.request.center = object_location;
    if (planar_segment_client_.call(planar_segment_srv_))
    {
      if (planar_segment_srv_.response.success)
      {
        ROS_INFO("Get surface normal");
        plane_s_ = planar_segment_srv_.response.plane_object;
        cylinder_orientation = this->calculate_quaternion(plane_s_);
      }
      else
      {
        ROS_INFO("Failed get surface normal");
        cylinder_orientation.x = 0;
        cylinder_orientation.y = 0;
        cylinder_orientation.z = 0;
        cylinder_orientation.w = 0;
      }
    }

    /* TO DO:
    Add surface normal detection fail detection and correction
    */

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

  double calDistance(object_marker::Object a, object_marker::Object b)
  {
    double x1 = a.transform.translation.x;
    double y1 = a.transform.translation.y;
    double z1 = a.transform.translation.z;
    double x2 = b.transform.translation.x;
    double y2 = b.transform.translation.y;
    double z2 = b.transform.translation.z;
    double distance = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));
    return distance;
  }

  bool markObjectCB(object_marker::MarkObject::Request &req,
                    object_marker::MarkObject::Response &res)
  {
    all_detect_objects_ = req.old_objects;
    int old_size = all_detect_objects_.objects.size();
    if (!this->rgbProc())
    {
      ROS_INFO("failed finding object from image");
      res.success = false;
      return true;
    }
    if (!this->depthProc())
    {
      ROS_INFO("failed calculate depth");
      res.success = false;
      return true;
    }
    int lock_count = temp_count_;
    int new_count = 0;
    for (int i = 0; i < lock_count; i++)
    {
      object_marker::Object single_object = this->object_calculation(coordinate_camera_[i].x, coordinate_camera_[i].y, coordinate_camera_[i].z);
      if ((single_object.transform.rotation.x == 0) && (single_object.transform.rotation.w == 0))
        continue;
      // check if duplicate
      bool isDuplicated = false;
      for (int j = 0; j < all_detect_objects_.objects.size(); j++)
      {
        if (this->calDistance(single_object, all_detect_objects_.objects[j]) < distanceThreshold_)
        {
          isDuplicated = true;
          break;
        }
      }
      if (!isDuplicated)
      {
        all_detect_objects_.objects.push_back(single_object);
        counts_ = all_detect_objects_.objects.size();

        this->marker_publish(all_detect_objects_);
        // an array[ID, x, y] to be save
        /*         float object_pose_orientation[7] = {all_detect_objects_.objects.back().transform.translation.x, all_detect_objects_.objects.back().transform.translation.y, all_detect_objects_.objects.back().transform.translation.z,
                                            all_detect_objects_.objects.back().transform.rotation.x, all_detect_objects_.objects.back().transform.rotation.y, all_detect_objects_.objects.back().transform.rotation.z, all_detect_objects_.objects.back().transform.rotation.w};
        ofstream out;
        out.open("/home/river/Desktop/objects.txt", ios_base::app | ios_base::out);
        for (int i = 0; i < 7; i++)
        {
          out << object_pose_orientation[i] << " ";
        }
        out << "\n";
        out.close(); */
        this->write_objects(all_detect_objects_);
      }
    }
    if (all_detect_objects_.objects.size() > old_size)
    {
      res.success = true;
      res.updated_objects = all_detect_objects_;
      return true;
    }
    else
    {
      ROS_INFO("No new object detected");
      res.success = false;
      return true;
    }
    // cout << "Detected Object Saved\n";
    // cout << "New saved " << lock_count << " objects, total saved " << counts_ << " objects.\n";
  }

  bool RecheckObjectCB(object_marker::RecheckObject::Request &req,
                       object_marker::RecheckObject::Response &res)
  {
    if (!this->rgbProc())
    {
      ROS_INFO("Object no longer exist");
      res.existence = false;
      return true;
    }
    else
    {
      ROS_INFO("Object is still there");
      res.existence = true;
      return true;
    }
  }

  void write_objects(object_marker::Objects objects)
  {
    ofstream out;
    out.open("/home/river/Desktop/objects.txt", ios_base::trunc | ios_base::out);
    for (int i = 0; i < objects.objects.size(); i++)
    {
      float object_pose_orientation[7] = {objects.objects[i].transform.translation.x, objects.objects[i].transform.translation.y, objects.objects[i].transform.translation.z,
                                          objects.objects[i].transform.rotation.x, objects.objects[i].transform.rotation.y, objects.objects[i].transform.rotation.z, objects.objects[i].transform.rotation.w};
      for (int j = 0; j < 7; j++)
      {
        out << object_pose_orientation[j] << " ";
      }
      out << "\n";
    }
    out.close();
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber info_sub_, image_sub_, yolo_sub_, yolo_number_sub_;
  ros::Publisher marker_pub_;
  ros::ServiceClient planar_segment_client_;
  ros::ServiceServer object_marker_src_, object_recheck_;
  point_cloud_jackal::PlanarSegmentation planar_segment_srv_;
  point_cloud_jackal::Plane plane_s_;
  image_geometry::PinholeCameraModel model1_;
  object_marker::Objects all_detect_objects_;

  darknet_ros_msgs::BoundingBoxesConstPtr yolo_msgs_;
  sensor_msgs::ImageConstPtr depth_msgs_;
  cv_bridge::CvImagePtr depth_img_cv_;

  vector<int> pixel_x_, pixel_y_;
  vector<cv::Point3d> coordinate_camera_;
  int temp_count_, counts_;
  double distanceThreshold_;
  bool isYoloUpdated_, isYoloCalled_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "object_marker");
  ros::NodeHandle n("~");
  ObjectMarker om(n);

  ROS_INFO("service initialized");

  ros::spin();
  return 0;
}