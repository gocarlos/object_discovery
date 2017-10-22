/*
 * object_discovery_ros_interface.cpp
 *
 *  Created on: 07.12.16
 *  Author: Carlos Gomes
 *  Institute: ETH Zurich, RSL
 *
 */

#include "object_discovery/object_discovery_ros_interface.hpp"

#include <glog/logging.h>

#include <pcl/Vertices.h>
#include <pcl/io/obj_io.h>

#include <iostream>
#include <string>
#include <vector>

namespace object_discovery {

ObjectDiscoveryRosInterface::ObjectDiscoveryRosInterface(
    ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle), got_point_cloud_(false), got_mesh_(false) {
  if (!readParameters()) {
    ROS_ERROR(
        "ObjectDiscoveryRosInterface: "
        "could not read parameters from parameter server!");
    ros::shutdown();
  }

  // ROS Services.
  segment_point_cloud_service_ = nodeHandle_.advertiseService(
      segment_point_cloud_service_name_,
      &ObjectDiscoveryRosInterface::startSegmentation, this);

  publish_point_cloud_service_ = nodeHandle_.advertiseService(
      publish_point_cloud_service_name_,
      &ObjectDiscoveryRosInterface::publishObjects, this);

  // ROS Subscribers.
  // Complete point cloud of the scene.
  scene_point_cloud_sub_ = nodeHandle_.subscribe(
      scene_point_cloud_topic_, 10,
      &ObjectDiscoveryRosInterface::pointCloudCallback, this);
  scene_mesh_sub_ = nodeHandle_.subscribe(
      scene_mesh_topic_, 10, &ObjectDiscoveryRosInterface::meshCallback, this);

  // ROS Advertisers.
  segmented_objects_pub_ = nodeHandle_.advertise<object_discovery::ObjectsMsgs>(
      segmented_objects_topic_, 100);
  segmented_clouds_pub_ = nodeHandle_.advertise<sensor_msgs::PointCloud2>(
      segmented_cloud_topic_, 100);
  markers_objects_pub_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>(
      visualization_marker_array_, 20);
  markers_mesh_pub_ = nodeHandle_.advertise<visualization_msgs::Marker>(
      mesh_visualization_marker_, 20);

  main_timer_ = nodeHandle_.createTimer(
      ros::Duration(0.2), &ObjectDiscoveryRosInterface::timerCallback, this,
      false, false);

  ROS_INFO("object detection ready, waiting for service call");

  scene_point_cloud_ptr_.reset(new Cloud_t);
}

void ObjectDiscoveryRosInterface::timerCallback(
    const ros::TimerEvent& timer_event) {
  //! ROS point cloud which is published for visualization.
  sensor_msgs::PointCloud2 point_cloud_to_publish;

  // Publish point cloud of one object, just for visualization.
  if (segmented_clouds_pub_.getNumSubscribers() > 0u) {
    for (size_t i = 0u; i < objects_.size(); ++i) {
      pcl::toROSMsg(objects_[i].getCloud(), point_cloud_to_publish);
      point_cloud_to_publish.height = 1;
      point_cloud_to_publish.header.frame_id = obj_frame_id_name_;
      point_cloud_to_publish.header.stamp = ros::Time::now();
      segmented_clouds_pub_.publish(point_cloud_to_publish);
    }
  }

  // Publish marker for all objects, only for visualization.
  if (markers_objects_pub_.getNumSubscribers() > 0u) {
    markers_objects_pub_.publish(obj_text_markers_);
    markers_objects_pub_.publish(obj_markers_);
  }

  // Publish all objects.
  if (segmented_objects_pub_.getNumSubscribers() > 0u) {
    segmented_objects_pub_.publish(objects_msgs_);
  }
}

bool ObjectDiscoveryRosInterface::startSegmentation(
    std_srvs::Empty::Request& request, std_srvs::Empty::Response& response) {
  ROS_INFO("Segmentation Service has been started.");
  mesh_to_segment_ptr.reset(new Mesh_t);

  bool okay_to_segment = false;
  if (got_mesh_ && got_point_cloud_) {
    ROS_INFO("Going to use the incoming ROS messages.");

    convertRosMeshToPCLPolygonMesh(scene_ros_mesh_, mesh_to_segment_ptr);

    okay_to_segment = true;
  } else {
    ROS_INFO("Going to read from STL file.");

    int number_of_points = pcl::io::loadPolygonFileSTL(
        ros::package::getPath("object_discovery") + "/scenes/scene_mesh.stl",
        *mesh_to_segment_ptr);

    if (number_of_points > 1) {
      okay_to_segment = true;
    } else {
      okay_to_segment = false;
      ROS_ERROR("Could not read from STL File. ");
    }
  }

  if (okay_to_segment) {
    if (mesh_to_segment_ptr->polygons.size() > 0u) {
      ROS_DEBUG("Segment the mesh.");

      //! Number of segmented objects which do not have an empty point cloud.
      size_t not_empty_objects = 0u;

      // Objects are segmented with this method.
      segmenter_.segmentMesh(mesh_to_segment_ptr);
      ROS_INFO_STREAM("Done with segmentation! ");
      Eigen::MatrixXf objectness_measures_;

      ROS_DEBUG("Count how many segments are not empty.");
      for (size_t i = 0u; i < segmenter_.kept_objects_.size(); ++i) {
        if (segmenter_.kept_objects_[i] != 0) {
          if ((segmenter_.all_segments_before_projection_[i])->empty()) {
            ROS_ERROR("Segmented cloud is empty!");
          } else {
            ++not_empty_objects;
          }
        }
      }

      ROS_DEBUG("Fill the objectness measures.");
      objectness_measures_.resize(not_empty_objects,
                                  segmenter_.measures_normalized_.cols());
      size_t objectness_iterator = 0u;
      for (size_t i = 0u; i < segmenter_.kept_objects_.size(); ++i) {
        if (segmenter_.kept_objects_[i] != 0) {
          for (size_t t = 0u; t < segmenter_.measures_normalized_.cols(); ++t) {
            objectness_measures_(objectness_iterator, t) =
                segmenter_.measures_normalized_(i, t);
          }
          ++objectness_iterator;
        }
      }

      ROS_DEBUG("Copy the cloud segments.");
      for (size_t i = 0u; i < segmenter_.kept_objects_.size(); ++i) {
        if (segmenter_.kept_objects_[i] != 0) {
          if ((segmenter_.all_segments_before_projection_[i])->empty()) {
            ROS_ERROR("Segmented cloud is empty!");
          } else if ((segmenter_.meshes_[i]->polygons.size()) < 1) {
            ROS_ERROR("Segmented mesh is empty!");
          } else {
            segmented_meshes_.push_back(segmenter_.meshes_[i]);
            segmented_clouds_.push_back(
                segmenter_.all_segments_before_projection_[i]);
          }
        }
      }

      ROS_INFO_STREAM("Objects found on the scene: " << not_empty_objects);

      segmented_normal_clouds_.resize(segmented_clouds_.size());
      segmented_ros_meshes_.resize(segmented_meshes_.size());

      ROS_DEBUG("Get normals of the point cloud of each object.");
      for (size_t j = 0u; j < not_empty_objects; ++j) {
        segmented_normal_clouds_[j].reset(
            new pcl::PointCloud<pcl::PointNormal>);
        geom_utils_.calculateNormals(segmented_clouds_[j],
                                     segmented_normal_clouds_[j]);

        segmented_ros_meshes_[j].reset(new shape_msgs::Mesh);
        geom_utils_.convertPolygonMeshToRosMesh((segmented_meshes_[j]),
                                                segmented_ros_meshes_[j]);
      }
      Eigen::Vector3f cloud_position;
      Eigen::Quaternionf cloud_orientation;
      Eigen::Vector3f cloud_scale;
      Eigen::Vector3f mass_center_out;
      pcl::PointCloud<Point_t>::Ptr temp_cloud(new pcl::PointCloud<Point_t>);
      Object obj;
      std::vector<float> objects_objectness;
      objects_objectness.resize(segmenter_.measures_normalized_.cols());
      objects_.clear();
      // Construct the objects.
      for (size_t j = 0u; j < not_empty_objects; ++j) {
        for (size_t t = 0u; t < segmenter_.measures_normalized_.cols(); ++t) {
          objects_objectness[t] = objectness_measures_(j, t);
        }
        obj.setObjectness(objects_objectness);
        obj.setCloud(segmented_normal_clouds_[j]);
        obj.setMesh(segmented_ros_meshes_[j]);
        pcl::copyPointCloud(*segmented_normal_clouds_[j], *temp_cloud);
        geom_utils_.calculatePCA(temp_cloud, cloud_position, cloud_orientation,
                                 cloud_scale, mass_center_out);
        obj.setPosition(cloud_position);
        obj.setScale(cloud_scale);
        obj.setOrientation(cloud_orientation);
        obj.setObjectId(j);
        ROS_INFO_STREAM("Cloud from object "
                        << j << " has "
                        << segmented_normal_clouds_[j]->points.size()
                        << " points.");
        objects_.push_back(obj);
      }

      ROS_DEBUG("Put the supporting surface as the last object into the list.");
      if (segmenter_.keep_only_objects_above_main_plane_) {
        pcl::PointCloud<pcl::PointNormal>::Ptr point_normal;
        point_normal.reset(new pcl::PointCloud<pcl::PointNormal>);
        geom_utils_.calculateNormals(segmenter_.plane_point_cloud_,
                                     point_normal);
        // The supporting surface does not have objectness value, set to zero.
        std::vector<float> zeros(segmenter_.measures_normalized_.cols(), 0.0f);
        obj.setObjectness(zeros);

        obj.setCloud(point_normal);
        pcl::copyPointCloud(*point_normal, *temp_cloud);
        geom_utils_.calculatePCA(temp_cloud, cloud_position, cloud_orientation,
                                 cloud_scale, mass_center_out);
        obj.setPosition(cloud_position);
        obj.setScale(cloud_scale);
        obj.setOrientation(cloud_orientation);
        obj.setObjectId(objects_.size());
        obj.setSupportingSurface(true);
        objects_.push_back(obj);
      }

      sensor_msgs::PointCloud2 ros_msg_objects_cloud;
      objects_msgs_.header.frame_id = obj_frame_id_name_;
      objects_msgs_.header.stamp = ros::Time::now();
      ROS_DEBUG("Fill the ROS Objects message.");
      for (size_t j = 0u; j < objects_.size(); ++j) {
        pcl::toROSMsg(objects_[j].getCloud(), ros_msg_objects_cloud);

        object_msgs_.cloud = ros_msg_objects_cloud;
        object_msgs_.mesh = objects_[j].getMesh();
        object_msgs_.pose.position.x = objects_[j].getPosition()[0];
        object_msgs_.pose.position.y = objects_[j].getPosition()[1];
        object_msgs_.pose.position.z = objects_[j].getPosition()[2];

        object_msgs_.pose.orientation.x = objects_[j].getOrientation().x();
        object_msgs_.pose.orientation.y = objects_[j].getOrientation().y();
        object_msgs_.pose.orientation.z = objects_[j].getOrientation().z();
        object_msgs_.pose.orientation.w = objects_[j].getOrientation().w();

        object_msgs_.scale[0] = objects_[j].getScale()[0];
        object_msgs_.scale[1] = objects_[j].getScale()[1];
        object_msgs_.scale[2] = objects_[j].getScale()[2];

        object_msgs_.object_id = objects_[j].getObjectId();
        object_msgs_.is_supporting_surface = objects_[j].isSupportingSurface();

        object_msgs_.objectness = objects_[j].getObjectness();

        objects_msgs_.objects.push_back(object_msgs_);
      }
      mesh_to_segment_ptr.reset();
    } else {
      ROS_ERROR("Mesh is empty!");
    }
  }

  ROS_DEBUG_STREAM("Done with startSegmentation! ");
  return true;
}

bool ObjectDiscoveryRosInterface::publishObjects(
    std_srvs::SetBool::Request& request,
    std_srvs::SetBool::Response& response) {
  if (request.data) {
    ROS_INFO("Going to publish the objects and the cloud");
    if (objects_.size() < 1) {
      ROS_ERROR("There are no objects!");
      response.success = false;
      response.message = "There are no objects!";
    } else {
      objectVisualization();
      main_timer_.start();
      response.success = true;
      response.message = "Started!";
    }
  }
  if (!request.data) {
    ROS_INFO("Going to stop publishing the objects and the cloud");
    main_timer_.stop();
    response.success = true;
    response.message = "Stopped!";
  }

  return true;
}

ObjectDiscoveryRosInterface::~ObjectDiscoveryRosInterface() {}

void ObjectDiscoveryRosInterface::objectVisualization() {
  ROS_DEBUG("objectVisualization start");
  obj_markers_.markers.clear();

  visualization_msgs::Marker obj_marker;
  obj_marker.header.frame_id = obj_frame_id_name_;
  obj_marker.header.stamp = ros::Time::now();
  obj_marker.ns = obj_marker_namespace_;
  for (size_t i = 0u; i < objects_.size(); ++i) {
    obj_marker.id = objects_[i].getObjectId();

    obj_marker.lifetime = ros::Duration();
    obj_marker.type = 1;
    obj_marker.action = visualization_msgs::Marker::ADD;

    obj_marker.pose.position.x = objects_[i].getPosition()[0];
    obj_marker.pose.position.y = objects_[i].getPosition()[1];
    obj_marker.pose.position.z = objects_[i].getPosition()[2];

    obj_marker.pose.orientation.x = objects_[i].getOrientation().x();
    obj_marker.pose.orientation.y = objects_[i].getOrientation().y();
    obj_marker.pose.orientation.z = objects_[i].getOrientation().z();
    obj_marker.pose.orientation.w = objects_[i].getOrientation().w();

    obj_marker.scale.x = objects_[i].getScale()[0];
    obj_marker.scale.y = objects_[i].getScale()[1];
    obj_marker.scale.z = objects_[i].getScale()[2];

    obj_marker.color.r = (static_cast<double>(rand()) / (RAND_MAX));
    obj_marker.color.g = (static_cast<double>(rand()) / (RAND_MAX));
    obj_marker.color.b = (static_cast<double>(rand()) / (RAND_MAX));
    constexpr float kAlphaChannel = 0.4f;
    obj_marker.color.a = kAlphaChannel;

    obj_markers_.markers.push_back(obj_marker);
  }
  markers_objects_pub_.publish(obj_markers_);

  visualization_msgs::Marker obj_txt_marker;
  obj_txt_marker.header.frame_id = obj_frame_id_name_;
  obj_txt_marker.header.stamp = ros::Time::now();
  obj_txt_marker.ns = obj_marker_text_namespace_;

  for (size_t i = 0u; i < objects_.size(); ++i) {
    constexpr double kScaleZ = 0.02;
    constexpr float kRedChannel = 1.0f;
    constexpr float kGreenChannel = 1.0f;
    constexpr float kBlueChannel = 1.0f;
    constexpr float kAlphaChannel = 0.8f;
    obj_txt_marker.id = objects_[i].getObjectId();

    obj_txt_marker.lifetime = ros::Duration();
    obj_txt_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    obj_txt_marker.action = visualization_msgs::Marker::ADD;

    obj_txt_marker.pose.position.x = objects_[i].getPosition()[0];
    obj_txt_marker.pose.position.y = objects_[i].getPosition()[1];
    obj_txt_marker.pose.position.z = objects_[i].getPosition()[2];

    obj_txt_marker.pose.orientation.x = objects_[i].getOrientation().x();
    obj_txt_marker.pose.orientation.y = objects_[i].getOrientation().y();
    obj_txt_marker.pose.orientation.z = objects_[i].getOrientation().z();
    obj_txt_marker.pose.orientation.w = objects_[i].getOrientation().w();

    obj_txt_marker.scale.z = kScaleZ;

    obj_txt_marker.color.r = kRedChannel;
    obj_txt_marker.color.g = kGreenChannel;
    obj_txt_marker.color.b = kBlueChannel;
    obj_txt_marker.color.a = kAlphaChannel;

    obj_txt_marker.text = "obj" + std::to_string(objects_[i].getObjectId());
    obj_text_markers_.markers.push_back(obj_txt_marker);
  }
  markers_objects_pub_.publish(obj_text_markers_);

  if (visualize_ros_mesh_ && scene_ros_mesh_.triangles.size() >= 1u &&
      got_mesh_) {
    constexpr double kScale = 1.0;
    constexpr float kRedChannel = 1.0f;
    constexpr float kGreenChannel = 0.0f;
    constexpr float kBlueChannel = 0.0f;
    constexpr float kAlphaChannel = 0.8f;

    mesh_marker_.points.clear();
    std::cout << "ROS Mesh Visualisation, vertices: "
              << scene_ros_mesh_.triangles.size()
              << " points: " << scene_ros_mesh_.vertices.size() << "\n";

    geometry_msgs::Point point;

    mesh_marker_.header.frame_id = obj_frame_id_name_;
    mesh_marker_.header.stamp = ros::Time::now();

    mesh_marker_.ns = mesh_marker_namespace_;
    mesh_marker_.id = 0u;

    mesh_marker_.lifetime = ros::Duration();
    mesh_marker_.type = visualization_msgs::Marker::TRIANGLE_LIST;
    mesh_marker_.action = visualization_msgs::Marker::ADD;

    // Quaternions are zero initialized.
    mesh_marker_.pose.orientation.w = 1.0;

    mesh_marker_.scale.x = kScale;
    mesh_marker_.scale.y = kScale;
    mesh_marker_.scale.z = kScale;

    mesh_marker_.color.r = kRedChannel;
    mesh_marker_.color.g = kGreenChannel;
    mesh_marker_.color.b = kBlueChannel;
    mesh_marker_.color.a = kAlphaChannel;

    for (size_t i = 0u; i < scene_ros_mesh_.vertices.size(); ++i) {
      point.x = scene_ros_mesh_.vertices[i].x;
      point.y = scene_ros_mesh_.vertices[i].y;
      point.z = scene_ros_mesh_.vertices[i].z;
      mesh_marker_.points.push_back(point);
    }

    if (mesh_marker_.points.size() % 3 == 2) {
      mesh_marker_.points.pop_back();
      mesh_marker_.points.pop_back();
    }
    if (mesh_marker_.points.size() % 3 == 1) {
      mesh_marker_.points.pop_back();
    }
    markers_mesh_pub_.publish(mesh_marker_);
  }
  ROS_DEBUG("objectVisualization end");
}

void ObjectDiscoveryRosInterface::pointCloudCallback(
    const sensor_msgs::PointCloud2& rosMsg) {
  if (!got_point_cloud_) {
    ROS_INFO("pointCloudCallback: Got ROS PointCloud.");

    pcl::fromROSMsg(rosMsg, *scene_point_cloud_ptr_);

    got_point_cloud_ = true;
  }
}

void ObjectDiscoveryRosInterface::meshCallback(
    const shape_msgs::Mesh& mesh_msg) {
  if (!got_mesh_) {
    ROS_INFO("meshCallback: Got ROS Mesh.");
    scene_ros_mesh_ = mesh_msg;
    got_mesh_ = true;
  }
}

void ObjectDiscoveryRosInterface::convertRosMeshToPCLPolygonMesh(
    const shape_msgs::Mesh& ros_mesh_in,
    pcl::PolygonMesh::Ptr polygon_mesh_out) {
  ROS_DEBUG("Convert ROS Mesh into PCL PolygonMesh.");

  std::size_t ros_mesh_nr_of_points = scene_ros_mesh_.vertices.size();
  std::size_t ros_mesh_nr_of_triangles = scene_ros_mesh_.triangles.size();
  std::size_t pcl_cloud_nr_of_points = scene_point_cloud_ptr_->points.size();

  // Fill the cloud of the PCL PolygonMesh
  pcl::toPCLPointCloud2(*scene_point_cloud_ptr_, polygon_mesh_out->cloud);

  // Convert the ROS triangles into PCL polygons.
  polygon_mesh_out->polygons.resize(ros_mesh_nr_of_triangles);

  for (size_t i = 0u; i < ros_mesh_nr_of_triangles; ++i) {
    polygon_mesh_out->polygons[i].vertices.resize(3);
    for (size_t j = 0u; j < 3u; ++j) {
      polygon_mesh_out->polygons[i].vertices[j] =
          scene_ros_mesh_.triangles[i].vertex_indices[j];
    }  // For each vertices in triangle.
  }    // For each triangles.

  ROS_DEBUG_STREAM("PCL PolygonMesh: \n"
                   << "Cloud Header: " << polygon_mesh_out->cloud.header
                   << "Polygons Size: " << polygon_mesh_out->polygons.size());
}

bool ObjectDiscoveryRosInterface::readParameters() {
  bool read_param_success = true;

  // ROS publishing topics.
  read_param_success += nodeHandle_.getParam(segment_object_topic_param_,
                                             segmented_objects_topic_);
  read_param_success +=
      nodeHandle_.getParam(segment_cloud_topic_param_, segmented_cloud_topic_);
  read_param_success += nodeHandle_.getParam(visualization_topic_param_,
                                             visualization_marker_array_);
  read_param_success += nodeHandle_.getParam(mesh_visualization_topic_param_,
                                             mesh_visualization_marker_);

  // ROS subscribing topics.
  read_param_success += nodeHandle_.getParam(scene_point_cloud_topic_param_,
                                             scene_point_cloud_topic_);
  read_param_success +=
      nodeHandle_.getParam(scene_mesh_topic_param_, scene_mesh_topic_);
  // ROS services.
  read_param_success +=
      nodeHandle_.getParam(segment_point_cloud_service_name_param_,
                           segment_point_cloud_service_name_);
  read_param_success +=
      nodeHandle_.getParam(publish_point_cloud_service_name_param_,
                           publish_point_cloud_service_name_);

  // Visualization.
  read_param_success += nodeHandle_.getParam(
      "object_discovery/visualisation/obj_frame_id_name", obj_frame_id_name_);

  read_param_success +=
      nodeHandle_.getParam("object_discovery/display", segmenter_.display_);
  read_param_success +=
      nodeHandle_.getParam("object_discovery/keep_upper_part_of_mesh",
                           segmenter_.keep_upper_part_of_mesh_);

  read_param_success +=
      nodeHandle_.getParam("object_discovery/vertices_cleaning_tolerance",
                           segmenter_.vertices_cleaning_tolerance_);
  read_param_success +=
      nodeHandle_.getParam("object_discovery/main_plane_offset_distance",
                           segmenter_.main_plane_offset_distance_);
  read_param_success += nodeHandle_.getParam(
      "object_discovery/dist_CoM_threshold", segmenter_.dist_CoM_threshold_);

  // Segmentation parameters
  read_param_success += nodeHandle_.getParam(
      "segmentation_parameters/plane_segmentation_distance_threshold",
      segmenter_.plane_segmentation_distance_threshold_);
  read_param_success += nodeHandle_.getParam("segmentation_parameters/KTHRS",
                                             segmenter_.params.KTHRS);
  read_param_success += nodeHandle_.getParam(
      "segmentation_parameters/MINSEGSIZE", segmenter_.params.MINSEGSIZE);
  read_param_success += nodeHandle_.getParam("segmentation_parameters/MINPTS",
                                             segmenter_.params.MINPTS);
  read_param_success += nodeHandle_.getParam("segmentation_parameters/MAXPTS",
                                             segmenter_.params.MAXPTS);
  read_param_success += nodeHandle_.getParam("segmentation_parameters/THINTHR",
                                             segmenter_.params.THINTHR);
  read_param_success += nodeHandle_.getParam("segmentation_parameters/FLATTHR",
                                             segmenter_.params.FLATTHR);
  read_param_success += nodeHandle_.getParam(
      "segmentation_parameters/MINLENGTH", segmenter_.params.MINLENGTH);
  read_param_success += nodeHandle_.getParam(
      "segmentation_parameters/MAXLENGTH", segmenter_.params.MAXLENGTH);
  read_param_success += nodeHandle_.getParam(
      "segmentation_parameters/SYMMETRY_MEASURE_CLOUD_NORMALS_TRADEOFF",
      segmenter_.params.SYMMETRY_MEASURE_CLOUD_NORMALS_TRADEOFF);
  read_param_success += nodeHandle_.getParam(
      "segmentation_parameters/LOCAL_CONVX_MEASURE_NNRADIUS",
      segmenter_.params.LOCAL_CONVX_MEASURE_NNRADIUS);
  read_param_success += nodeHandle_.getParam(
      "segmentation_parameters/SMOOTHNESS_MEASURE_NNRADIUS",
      segmenter_.params.SMOOTHNESS_MEASURE_NNRADIUS);
  read_param_success +=
      nodeHandle_.getParam("segmentation_parameters/SMOOTHNESS_MEASURE_NUMBINS",
                           segmenter_.params.SMOOTHNESS_MEASURE_NUMBINS);
  read_param_success += nodeHandle_.getParam("segmentation_parameters/IOUTHR",
                                             segmenter_.params.IOUTHR);

  return read_param_success;
}

}  // namespace object_discovery
