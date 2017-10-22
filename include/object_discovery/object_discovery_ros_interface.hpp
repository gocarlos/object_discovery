/*
 * ObjectDiscovery.hpp
 *
 *  Created on: 07.12.2016
 *      Author: Carlos Gomes
 *	 Institute: ETH Zurich, RSL
 *
 */

#pragma once

#include <iostream>
#include <string>
#include <vector>

// ROS
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <shape_msgs/Mesh.h>
#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

// PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

// Messages
#include "object_discovery/Obj.h"
#include "object_discovery/ObjectsMsgs.h"

#include "object_discovery/object.hpp"
#include "object_discovery/segmenter.hpp"

namespace object_discovery {

/*!
 * The cloud interpretation main class. Coordinates the ROS interfaces, the
 * timing, and the data handling between the other classes.
 */
class ObjectDiscoveryRosInterface {
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  ObjectDiscoveryRosInterface(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  ~ObjectDiscoveryRosInterface();
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */

  bool readParameters();

  /*!
   * Callback function for publishing status of the whole operation.
   * @param timer_event the timer event.
   */
  void timerCallback(const ros::TimerEvent& timer_event);

  /*!
   * ROS service callback function to trigger cloud segmentation process.
   * @param request the ROS service request.
   * @param response the ROS service response.
   * @return true if successful.
   */
  bool startSegmentation(std_srvs::Empty::Request& request,
                         std_srvs::Empty::Response& response);

  /*!
   * ROS service callback function to start/stop cloud publishing/visualisation
   * process.
   * @param request the ROS service request, true or false.
   * @param response the ROS service response.
   * @return true if successful.
   */
  bool publishObjects(std_srvs::SetBool::Request& request,
                      std_srvs::SetBool::Response& response);

  /*!
   * Callback for the incoming point clouds, which should be segmented.
   */
  void pointCloudCallback(const sensor_msgs::PointCloud2& rosMsg);

  /*!
   * Callback for the incoming point clouds, which should be segmented.
   */
  void meshCallback(const shape_msgs::Mesh& rosMsg);

  /*!
   * Converts a ROS Mesh into a PCL PolygonMesh.
   */
  void convertRosMeshToPCLPolygonMesh(const shape_msgs::Mesh& ros_mesh_in,
                                      pcl::PolygonMesh::Ptr polygon_mesh_out);

  /*!
   * Publish ROS clouds, markers etc, to visualize the whole segmentation in
   * RVIZ.
   */
  void objectVisualization();

 private:
  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  // Time to publish the cloud frequently.
  ros::Timer main_timer_;

  //! ROS publishers for segmented objects.
  ros::Publisher segmented_objects_pub_;

  //! ROS publishers for segmented objects.
  ros::Publisher segmented_clouds_pub_;

  //! ROS subscribers for scene cloud.
  ros::Subscriber scene_point_cloud_sub_;

  //! ROS topic: topic where the whole cloud is subscribed.
  const std::string scene_point_cloud_topic_param_ =
      "object_discovery/sub_topics/scene_point_cloud_topic";
  std::string scene_point_cloud_topic_;

  //! ROS Subscribers for scene mesh.
  ros::Subscriber scene_mesh_sub_;

  //! ROS topic: topic where the whole mesh is subscribed.
  const std::string scene_mesh_topic_param_ =
      "object_discovery/sub_topics/scene_mesh_topic";
  std::string scene_mesh_topic_;

  //! ROS Service triggers cloud segmentation.
  ros::ServiceServer segment_point_cloud_service_;
  const std::string segment_point_cloud_service_name_param_ =
      "object_discovery/services/segment_scene_service_name";
  std::string segment_point_cloud_service_name_;

  //! ROS service triggers a ROS publisher to publish the segmented objects.
  ros::ServiceServer publish_point_cloud_service_;
  const std::string publish_point_cloud_service_name_param_ =
      "object_discovery/services/publish_scene_service_name";
  std::string publish_point_cloud_service_name_;

  //! ROS topic: topic where the segmented objects are published/subscribed.
  const std::string segment_object_topic_param_ =
      "object_discovery/pub_topics/segmented_objects_topic";
  std::string segmented_objects_topic_;
  const std::string segment_cloud_topic_param_ =
      "object_discovery/pub_topics/segmented_cloud_topic";
  std::string segmented_cloud_topic_;
  const std::string visualization_topic_param_ =
      "object_discovery/pub_topics/segment_objects_visualisation_topic";
  std::string visualization_marker_array_;
  const std::string mesh_visualization_topic_param_ =
      "object_discovery/pub_topics/scene_mesh_visualization_topic";
  std::string mesh_visualization_marker_;

  //! Point cloud of the scene.
  pcl::PointCloud<Point_t>::Ptr scene_point_cloud_ptr_;

  //! Incoming ROS mesh representing the scene.
  shape_msgs::Mesh scene_ros_mesh_;

  //! PCL PolygonMesh one wants to segment.
  pcl::PolygonMesh::Ptr mesh_to_segment_ptr;

  //! Vector containing all segmented point clouds.
  std::vector<Cloud_t::Ptr> segmented_clouds_;
  std::vector<Mesh_t::Ptr> segmented_meshes_;
  std::vector<shape_msgs::Mesh::Ptr> segmented_ros_meshes_;

  //! Vector containing all segmented point clouds with normals.
  std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> segmented_normal_clouds_;

  //! Whether a point cloud was received over a ROS topic.
  bool got_point_cloud_;

  //! Whether a mesh was received over a ROS topic.
  bool got_mesh_;

  //! Flag to publish the ROS Mesh as a triangle list for RVIZ.
  bool visualize_ros_mesh_;

  //! Classes which provide methods used here.
  Utils utils_;
  GeomUtils geom_utils_;
  Measures measures_;
  Segmenter segmenter_;

  //! Class which represents a segmented object.
  std::vector<Object> objects_;

  //! ROS message representing objects.
  Obj object_msgs_;
  ObjectsMsgs objects_msgs_;

  //! ROS Frame id where the objects and markers are placed in. This is a ROS
  //! parameter.
  std::string obj_frame_id_name_;

  //! Visualization.
  ros::Publisher markers_objects_pub_;
  ros::Publisher markers_mesh_pub_;
  std::string obj_marker_namespace_{"segmented_objects"};
  std::string obj_marker_text_namespace_{"segmented_objects_text"};
  visualization_msgs::Marker mesh_marker_;
  std::string mesh_marker_namespace_{"scene_mesh"};
  visualization_msgs::MarkerArray obj_markers_;
  visualization_msgs::MarkerArray obj_text_markers_;
};

}  // namespace object_discovery
