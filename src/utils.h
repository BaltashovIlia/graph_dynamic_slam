#pragma once

#include <ros/ros.h>
#include <ros/console.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <std_msgs/Header.h>

#include "map_structure.h"
#include "limits"

namespace graph_dynamic_slam
{

  // Вспомогательные функции
  tf2::Transform odomToTfTransform(const nav_msgs::Odometry::ConstPtr& msg);
  tf2::Transform eigenMatrixToTfTransform(const Eigen::Matrix4f& mat);
  Eigen::Matrix4f tfTransformToEigenMatrix(const tf2::Transform& transform);
  Eigen::Isometry3d tfTransformToEigenIsometry(const tf2::Transform& transform);

  // Получение преобразования между двумя фреймами
  tf2::Transform getTransformFromTf(const tf2_ros::Buffer& buffer, const std::string& parrentFrameId, const std::string& chieldFrameId, const ros::Time& time);

  // Красиво опубликовать tf2::Transform, необходдимо для отлаки
  void printTfTransform(const tf2::Transform& transform);

  // Алгоритм дейкстры поиска ближайшего пути в графе
  std::pair<double, int> findShortestPathLength(std::shared_ptr<g2o::SparseOptimizer> graph, g2o::VertexSE3* nodeStart, g2o::VertexSE3* nodeEnd);
}
