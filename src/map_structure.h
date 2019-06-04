#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Transform.h>

// TODO почистить
#include <g2o/stuff/macros.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d/edge_se3_pointxyz.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>
#include <g2o/types/slam3d/edge_se3.h>

#include <mutex>
#include <memory.h>
#include <queue>

#include "utils.h"
namespace g2o
{
  class VertexSE3;
  class EdgeSE3;
}

namespace graph_dynamic_slam
{
using PointType = pcl::PointXYZI;


struct CloudStructure
{
  pcl::PointCloud<PointType>::Ptr cloudTransformToSubmap; // Исхоное облако точек во фрейме сенсора
  tf2::Transform sensorToOdom;           // Преобразование между сенсором и роботом в момент прихода данных с лидара
  tf2::Transform odomTfTransform;        // Данные одометрии в момент прихода данных сенсора
  tf2::Transform submapToSensor;
};


struct SubmapStructure
{
  std::vector<std::shared_ptr<CloudStructure>> cloudVector; // Набор всех сканов в данной подкарте
  pcl::PointCloud<PointType>::Ptr cloudRaw; // Облако точек равное сумме облаков точек со всех сканов, приведённых к позиции первого скана
  tf2::Transform submapStartOdomTf;         // Текущие преобразование до начала покарты // TODO - можно удалить так как cloudVector[0].odomTfTransform имеет туже информацию
  tf2::Transform inMapPose; // Позиция покарты на карте - скорее всего можно будет потом удалить
  std::queue<PointType> pointToFilter;   // Найденные динамически точки

  g2o::VertexSE3* node; // Указатель на узел ребра, которому соответствует данная подкарта
};


class MapStructure
{
public:
  MapStructure();
  ~MapStructure();


  std::vector<std::shared_ptr<SubmapStructure>> submapVector; // Набор всех подкарт в карте

  tf2::Transform getLastSubmapStartTf(); // TODO кажется его можно будет удалить
  tf2::Transform getLastLastSubmapStartTf(); // TODO кажется его можно будет удалить


  // Оптимизатор
  std::shared_ptr<g2o::SparseOptimizer> graph;

  // Тупые указатели использую так как в g2o так заведено
  // Добавление узлов и рёбер графа
  g2o::VertexSE3* add_node(const tf2::Transform& pose);
  g2o::EdgeSE3* add_edge(g2o::VertexSE3* v1, g2o::VertexSE3* v2, const tf2::Transform& relative_pose, const double fitnessScore, const double dynamicPointsPersent);
  std::mutex optMutex;
};

}
