#pragma once

#include <ros/ros.h>
#include <ros/console.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <std_msgs/Header.h>


#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>

#include <pclomp/ndt_omp.h>
#include <pclomp/gicp_omp.h>


#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <future>
#include <thread>
#include <array>
#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>
#include <set>
#include <mutex>

#include <visualization_msgs/MarkerArray.h>


#include "map_structure.h"
#include "utils.h"

namespace g2o
{
  class VertexSE3;
  class EdgeSE3;
}

namespace graph_dynamic_slam
{


class PointcloudMatching
{
public:
  PointcloudMatching();
  ~PointcloudMatching();

private:
  // Основная функция синхронной функции обработки облака точек и одометрии
  void callback(const sensor_msgs::PointCloud2::ConstPtr& cloudMsg, const nav_msgs::Odometry::ConstPtr& odomMsg);

  // Паралельная функция обрабоки подкарт
  std::tuple<tf2::Transform, double, double> submapRegistration(const std::shared_ptr<SubmapStructure> lastSubmap, const std::shared_ptr<SubmapStructure> currentSubmap, const tf2::Transform initSubmapTransform);

  // Публикация odom сообщения и tf (внутри происходит преобразование к publishFrame)
  void pubOdomAndTf(const tf2::Transform& transform, const ros::Time& time);

  // Фильтрация динамических объектов в облаках точек
  void dynamicCloudFilter(pcl::PointCloud<PointType>::Ptr castingCloud, pcl::PointCloud<PointType>::Ptr mapCloud, const PointType castingPose, std::queue<PointType>& pointToFilter);

  // Фильтрация подкарт
  double submapFilter(const std::shared_ptr<SubmapStructure> loopSubmapOne, const std::shared_ptr<SubmapStructure> loopSubmapSeccond, const tf2::Transform& foundTransform);

  // Удаление точек с поддкарты
  double deletePointsFromSubmap(pcl::PointCloud<PointType>::Ptr cloudInput, pcl::PointCloud<PointType>::Ptr cloudOutput, std::queue<PointType>& pointToFilter);

  // Публикация графа и карты
  void pubMapAndGrapth();

  // Поиск замыкания цикла и оптимизация
  void tryToOptimization();

  // Сама оптимизация
  void optimize();

  // Синхронизация входных сообщений одометрии и облака точек
  message_filters::Subscriber<sensor_msgs::PointCloud2> subPointCloud;
  message_filters::Subscriber<nav_msgs::Odometry> subOdom;
  message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>> *sync;


  // Паблишеры облака точек, найденной одометрии, графа подкарт и отфильтрованного облака точек
  ros::Publisher pubMapPointCloud;
  ros::Publisher pubOdom;
  ros::Publisher pubGraphRviz;
  ros::Publisher pubPointCloudFilter;


  // Все фреймы используемые в программе
  std::string sensorFrameId;    // В нём получаем показания от сенсора - velodyne - определяется автоматически
  std::string odomChildFraimId; // Фрейм который тречит одометрия - base_link или base_footprint - определяется автоматически
  std::string mapFrameId;       // Фрейм карты - map - задаётся пользователем
  std::string publishFraimId;   // Фрейм в который паблишу - base_link или odom - задаётся пользователем
  // !Считаю что фрейм родотеля в odom статичен!
  // Преобразование между sensorFrameId и odomChildFraimId обязано быть
  // Преобразование между publishFraimId и odomChildFraimId обязано быть

  // Для публикации одометрии
  tf2_ros::TransformBroadcaster tfBroadcaster;
  bool enableTfPub;

  // Для получение преобразованиями координат
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;


  // Максимальное количество сканов в подкарте
  uint32_t maxNumOfCloudInSubmap;

  // Размер вокселя к которому фильтровать облака точек пере регистрацией
  float voxelFilterLeafSize;

  // Алгоритм регистрации двух покарт. NDT выбран так как он устойчивее к неструктурированным областям
  // А реализация из pclomp просто быстрее из-за sse и openmp
  pclomp::NormalDistributionsTransform<PointType, PointType> registrationMethod;
  pclomp::NormalDistributionsTransform<PointType, PointType> loopRegistrationMethod;
  int NDTMaximumIterations;
  double NDTTransformationEpsilon;
  float NDTResolution;
  double registrationMaxFitnessScore;


  // Для работы с подкартой в парралельном потоке
  std::future<std::tuple<tf2::Transform, double, double>> submapThread;


  // Минимальное перемещение и поворот между двумя облаками точек, чтобы быть добавленными в подкарту
  tf2::Transform lastAddPointCloudTransform;
  double minSqrTransformTrans;
  double minTransformRotation;


  // Фильтрация динамических объектов
  float dynamicPointDl;
  float dynamicPointDr;
  double dynamicPointRadToDelete;
  bool enableDynamicFilter;

  // Карта которую строит алгоритм SLAM
  MapStructure map;
  std::set<std::pair<int, int>> testedMatching;

  // Замыкание цикла
  bool enableLoopClosing;
  int loopMaxIterationsOptimize;
  double loopMaxFitnessScore;
  int loopMaxNodeToOptimize;

  size_t loopMinDeltaId;
  double loopMinAccumulateDist;
  double loopMaxDirectDist;
  double loopKAccDist;
  double loopDistDynmic = 0.0;
  double loopDistDynmicMax;


  // Для работы замыкания цикла в парралельном потоке
  std::future<void> loopThread;
};

}

