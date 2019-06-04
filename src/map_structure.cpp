#include "map_structure.h"

// Инициализация алгоритмов оптимизации графа
G2O_USE_OPTIMIZATION_LIBRARY(pcg)
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)
G2O_USE_OPTIMIZATION_LIBRARY(csparse)
G2O_USE_TYPE_GROUP(slam3d);

namespace graph_dynamic_slam
{

MapStructure::MapStructure()
{
  graph.reset(new g2o::SparseOptimizer());

  // Инициализирую алгоритм оптимизации графа
  g2o::OptimizationAlgorithmFactory* solver_factory = g2o::OptimizationAlgorithmFactory::instance();
  g2o::OptimizationAlgorithmProperty solver_property;
  g2o::OptimizationAlgorithm* solver = solver_factory->construct("lm_var", solver_property); // TODO Имя оптимизатора в лаунч
  graph->setAlgorithm(solver);

  if (!graph->solver()) {
    std::cerr << std::endl;
    std::cerr << "error : failed to allocate solver!!" << std::endl;
    solver_factory->listSolvers(std::cerr);
    std::cerr << "-------------" << std::endl;
    std::cin.ignore(1);
    return;
  }
  std::cout << "done" << std::endl;

}


MapStructure::~MapStructure()
{
  graph.reset();
}


g2o::VertexSE3* MapStructure::add_node(const tf2::Transform& pose)
{
  optMutex.lock();
  g2o::VertexSE3* vertex(new g2o::VertexSE3());
  vertex->setId(static_cast<int>(graph->vertices().size()));
  Eigen::Isometry3d nodePose;
  vertex->setEstimate(tfTransformToEigenIsometry(pose));
  graph->addVertex(vertex);
  optMutex.unlock();

  //vertex->write(std::cout);
  //std::cout << std::endl;

  return vertex;
}


 g2o::EdgeSE3* MapStructure::add_edge(g2o::VertexSE3* v1, g2o::VertexSE3* v2, const tf2::Transform& relative_pose, const double fitnessScore, const double dynamicPointsPersent)
{
  optMutex.lock();
  g2o::EdgeSE3* edge = new g2o::EdgeSE3();

  edge->vertices()[0] = v1;
  edge->vertices()[1] = v2;

  // TODO параметры в launch
  // Чем больше fitnessScore (среднекваратическое расстояние межу облаками точек) тем меньше я доверяю данной связи
  // Чем больше количество точек опреелённых как динамические, тем меньше я доверяю данной связи
  double posErr =  5000.0 / (1.0 + 1.0 * fitnessScore + 0.10 * dynamicPointsPersent);
  double oriErr = 25000.0 / (1.0 + 1.0 * fitnessScore + 0.10 * dynamicPointsPersent);

  // Я больше довряю последовательным рёбрам, даже если они завершились с худшим замыканием цикла
  if (abs(v1->id()-v2->id())==1)
  {
    posErr += 7500.0;
    oriErr += 50000.0;
  }

  // Именно так задаю рёбра, так как простое задание матрицы не работало (не знаю почему, в рёбра бред писался).
  std::string stringvalues = std::to_string(relative_pose.getOrigin().x()) + " " + std::to_string(relative_pose.getOrigin().y()) + " "+ std::to_string(relative_pose.getOrigin().z()) + " "
      + std::to_string(relative_pose.getRotation().x()) + " "+ std::to_string(relative_pose.getRotation().y()) + " "+ std::to_string(relative_pose.getRotation().z()) + " "+ std::to_string(relative_pose.getRotation().w())
      + " " + std::to_string(posErr) + " 0 0 0 0 0 " + std::to_string(posErr) + " 0 0 0 0 " + std::to_string(posErr)
      + " 0 0 0 " + std::to_string(oriErr) + "  0 0 " + std::to_string(oriErr) + " 0 " + std::to_string(oriErr);

  std::istringstream iss (stringvalues);
  edge->read(iss);

  //edge->write(std::cout);
  //std::cout << std::endl;

  graph->addEdge(edge);
  optMutex.unlock();

  return edge;
}

tf2::Transform MapStructure::getLastSubmapStartTf()
{
  if(submapVector.size()>1)
  {
    return submapVector[submapVector.size()-2]->submapStartOdomTf;
  }
  else
  {
    return submapVector.back()->submapStartOdomTf;
  }
}
tf2::Transform MapStructure::getLastLastSubmapStartTf()
{
  if (submapVector.size()>2)
  {
    return submapVector[submapVector.size()-3]->submapStartOdomTf;
  }
  else if(submapVector.size()>1)
  {
    return submapVector[submapVector.size()-2]->submapStartOdomTf;
  }
  else
  {
    return submapVector.back()->submapStartOdomTf;
  }
}

}
