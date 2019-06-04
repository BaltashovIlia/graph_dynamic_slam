#include "utils.h"


namespace graph_dynamic_slam
{
  // Вспомогателььные функции
  // Преобразования типов
  tf2::Transform odomToTfTransform(const nav_msgs::Odometry::ConstPtr& msg)
  {
    tf2::Transform odomTransform;
    odomTransform.setOrigin(tf2::Vector3(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
    odomTransform.setRotation(tf2::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
    return odomTransform;
  }

  Eigen::Matrix4f tfTransformToEigenMatrix(const tf2::Transform& transform)
  {
    // Проще способа пока ненашёл
    Eigen::Quaternionf quat;
    quat.x() = static_cast<float>(transform.getRotation().x());
    quat.y() = static_cast<float>(transform.getRotation().y());
    quat.z() = static_cast<float>(transform.getRotation().z());
    quat.w() = static_cast<float>(transform.getRotation().w());

    Eigen::Matrix3f mat3 (quat);

    Eigen::Matrix4f tmpMat;
    tmpMat.setIdentity();

    tmpMat.block(0,0,3,3) = mat3;
    tmpMat(0, 3) = static_cast<float>(transform.getOrigin().x());
    tmpMat(1, 3) = static_cast<float>(transform.getOrigin().y());
    tmpMat(2, 3) = static_cast<float>(transform.getOrigin().z());

    return tmpMat;
  }

  tf2::Transform eigenMatrixToTfTransform(const Eigen::Matrix4f& mat)
  {
    // Более простого способа получить квантарнион из матрицы преобразования пока не нашёл
    Eigen::Matrix3f mat3 = mat.block(0,0,3,3);
    Eigen::Quaternionf quat(mat3);

    tf2::Transform transform;
    transform.setOrigin(tf2::Vector3(static_cast<double>(mat(0, 3)), static_cast<double>(mat(1, 3)), static_cast<double>(mat(2, 3))));
    transform.setRotation(tf2::Quaternion(static_cast<double>(quat.x()), static_cast<double>(quat.y()), static_cast<double>(quat.z()), static_cast<double>(quat.w())));

    return transform;
  }

  Eigen::Isometry3d tfTransformToEigenIsometry(const tf2::Transform& transform)
  {
    Eigen::Quaterniond quat;
    quat.x() = transform.getRotation().x();
    quat.y() = transform.getRotation().y();
    quat.z() = transform.getRotation().z();
    quat.w() = transform.getRotation().w();

    Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
    isometry.linear() = quat.toRotationMatrix();
    isometry.translation() = Eigen::Vector3d(transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());

    return isometry;
  }


  // Получение преобразования между двумя фреймами
  tf2::Transform getTransformFromTf(const tf2_ros::Buffer &buffer, const std::string& parrentFrameId, const std::string& chieldFrameId, const ros::Time& time)
  {
    geometry_msgs::TransformStamped transformMsg;
    try
    {
      transformMsg = buffer.lookupTransform(parrentFrameId, chieldFrameId, time);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN_STREAM(ex.what());
      tf2::Transform emptyTransform;
      emptyTransform.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
      emptyTransform.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
      return emptyTransform;
    }

    tf2::Transform transform;
    tf2::fromMsg(transformMsg.transform, transform);
    return transform;
  }


  void printTfTransform(const tf2::Transform& transform)
  {
    std::cout << "ori - x = " << transform.getOrigin().x() << " y = " << transform.getOrigin().y() <<  " z = " << transform.getOrigin().z() << std::endl;
    std::cout << "ros - x = " << transform.getRotation().x() << " y = " << transform.getRotation().y() << " z = "  << transform.getRotation().z() << " w = "  << transform.getRotation().w() << std::endl;
    return;
  }



  // Алгоритм дейкстры поиска ближайшего пути в графе
  std::pair<double, int> findShortestPathLength(std::shared_ptr<g2o::SparseOptimizer> graph, g2o::VertexSE3* nodeStart, g2o::VertexSE3* nodeEnd)
  {
    // Создали массив длинн до вершин
    std::map<int, double> dist;
    std::map<int, int> distVertex;
    for(auto vert_itr = graph->vertices().begin(); vert_itr != graph->vertices().end(); vert_itr++)
    {
      g2o::VertexSE3* vert_se3 = dynamic_cast<g2o::VertexSE3*>(vert_itr->second);
      dist[vert_se3->id()] = std::numeric_limits<double>::max();
      distVertex[vert_se3->id()] = 0;
    }

    // Созаём пустой список
    std::set<int> vertList;

    // Добавили в него первый элимент
    vertList.insert(nodeStart->id());
    dist[nodeStart->id()] = 0;


    // Сам алгоритм
    while (vertList.size()>0)
    {
      // Находим узел с наименьшим расстоянием
      double minDist = std::numeric_limits<double>::max();
      int minId = *vertList.begin();
      for (int id : vertList)
      {
        if (dist[id]<minDist)
        {
          minDist = dist[id];
          minId = id;
        }
      }

      // Извлекаем его из списка
      vertList.erase(minId);

      // Если это искомый узел, то закончили
      if (minId == nodeEnd->id())
      {
        return {minDist, distVertex[minId]};
      }

      // Для каждого соседнего к текущему узлу n
      for(auto edge_itr = graph->vertex(minId)->edges().begin(); edge_itr != graph->vertex(minId)->edges().end(); edge_itr++)
      {
        g2o::HyperGraph::Edge* edge = *edge_itr;
        g2o::EdgeSE3* edge_se3 = dynamic_cast<g2o::EdgeSE3*>(edge);
        if(edge)
        {
          g2o::VertexSE3* conEdgev1 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[0]);
          g2o::VertexSE3* conEdgev2 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[1]);

          double distOfEdge = sqrt( edge_se3->measurement().translation().x() * edge_se3->measurement().translation().x()
                                  + edge_se3->measurement().translation().y() * edge_se3->measurement().translation().y()
                                  + edge_se3->measurement().translation().z() * edge_se3->measurement().translation().z());

          if (conEdgev1->id()!=minId)
          {
            if(dist[conEdgev1->id()] > dist[minId] + distOfEdge)
            {
              dist[conEdgev1->id()] = dist[minId] + distOfEdge;
              distVertex[conEdgev1->id()] = distVertex[minId] + 1;
              vertList.insert(conEdgev1->id());
            }
          }
          else if (conEdgev2->id()!=minId)
          {
            if(dist[conEdgev2->id()] > dist[minId] + distOfEdge)
            {
              dist[conEdgev2->id()] = dist[minId] + distOfEdge;
              distVertex[conEdgev2->id()] = distVertex[minId] + 1;
              vertList.insert(conEdgev2->id());
            }
          }
          else
          {
            std::cout << "WTF!" << std::endl;
          }
        }
      }
    }
    std::cout << "WTF2!" << std::endl;
    return {0.0, 0};

  }
}
