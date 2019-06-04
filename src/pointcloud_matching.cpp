#include "pointcloud_matching.h"

namespace graph_dynamic_slam
{
using namespace std::chrono_literals;

PointcloudMatching::PointcloudMatching():
  tfListener(tfBuffer)
{
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  std::cout << std::boolalpha;



  // Считывание параметров из ROS
  // Имя фрейма карты
  pn.param<std::string>("map_frame_id", mapFrameId, "map");
  ROS_INFO_STREAM("map_frame_id = " << mapFrameId);

  // Имя публикуемого фрейма
  pn.param<std::string>("publish_fraim_id", publishFraimId, "base_link"); // Определили фрейм в котором публиковать одометрию
  ROS_INFO_STREAM("publish_fraim_id = " << publishFraimId);

  // Максималььное количество сканов в подкарте
  int tmpInt;
  pn.param<int>("max_num_of_cloud_in_submap", tmpInt, 10);
  maxNumOfCloudInSubmap = static_cast<uint32_t>(tmpInt);
  ROS_INFO_STREAM("max_num_of_cloud_in_submap = " << maxNumOfCloudInSubmap);

  // Размер воксельного фильтра перед регистрацией подкарт
  pn.param<float>("voxel_filter_leaf_size", voxelFilterLeafSize, 0.02f);
  ROS_INFO_STREAM("voxel_filter_leaf_size = " << voxelFilterLeafSize);

  // Публиковать ли tf
  pn.param<bool>("enable_tf_pub", enableTfPub, false);
  ROS_INFO_STREAM("enable_tf_pub = " << enableTfPub);

  // Минимальное перемещение между двумя сканами
  double tmpDouble;
  pn.param<double>("min_transform_trans", tmpDouble, 0.1);
  ROS_INFO_STREAM("min_transform_trans = " << tmpDouble);
  minSqrTransformTrans = tmpDouble*tmpDouble;
\
  // Минимальный поворот между двумя сканами
  pn.param<double>("min_transform_rotation", minTransformRotation, 0.17);
  ROS_INFO_STREAM("min_transform_rotation = " << minTransformRotation);

  // Смещение объекта, чтобы считаться инамическим
  pn.param<float>("dynamic_point_dl", dynamicPointDl, 0.03f); // 0.1 много 0.005 мало
  ROS_INFO_STREAM("dynamic_point_dl = " << dynamicPointDl);

  // Минимальное расстояние до объекта за динамическим объектом
  pn.param<float>("dynamic_point_dr", dynamicPointDr, 1.0f); // 1.0
  ROS_INFO_STREAM("dynamic_point_dr = " << dynamicPointDr);


  // Параметры NDT
  pn.param<int>("ndt_maximum_iterations", NDTMaximumIterations, 64);
  ROS_INFO_STREAM("ndt_maximum_iterations = " << NDTMaximumIterations);

  pn.param<double>("ndt_transformation_epsilon", NDTTransformationEpsilon, 0.01);
  ROS_INFO_STREAM("ndt_transformation_epsilon = " << NDTTransformationEpsilon);

  pn.param<float>("ndt_resolution", NDTResolution, 0.5);
  ROS_INFO_STREAM("ndt_resolution = " << NDTResolution);

  // Если fitness_score больше max_fitness_score, то регистрация завершена неусмпешно
  pn.param<double>("registration_max_fitness_score", registrationMaxFitnessScore, 0.5);
  ROS_INFO_STREAM("registration_max_fitness_score = " << registrationMaxFitnessScore);

  pn.param<bool>("enable_dynamic_filter", enableDynamicFilter, false);
  ROS_INFO_STREAM("enable_dynamic_filter = " << enableDynamicFilter);

  pn.param<bool>("enable_loop_closing", enableLoopClosing, false);
  ROS_INFO_STREAM("enable_loop_closing = " << enableLoopClosing);

  pn.param<double>("dynamic_point_rad_to_delete", dynamicPointRadToDelete, 0.4);
  ROS_INFO_STREAM("dynamic_point_rad_to_delete = " << dynamicPointRadToDelete);


  pn.param<int>("loop_min_delta_id", tmpInt, 5);
  loopMinDeltaId = static_cast<size_t>(tmpInt);
  ROS_INFO_STREAM("loop_min_delta_id = " << loopMinDeltaId);

  pn.param<double>("loop_min_accumulate_dist", loopMinAccumulateDist, 4.0);
  ROS_INFO_STREAM("loop_min_accumulate_dist = " << loopMinAccumulateDist);

  pn.param<double>("loop_max_direct_dist", loopMaxDirectDist, 6.0);
  ROS_INFO_STREAM("loop_max_direct_dist = " << loopMaxDirectDist);

  pn.param<int>("loop_max_iterations_optimize", loopMaxIterationsOptimize, 10);
  ROS_INFO_STREAM("loop_max_iterations_optimize = " << loopMaxIterationsOptimize);

  pn.param<double>("loop_max_fitness_score", loopMaxFitnessScore, 0.5);
  ROS_INFO_STREAM("loop_max_fitness_score = " << loopMaxFitnessScore);

  pn.param<int>("loop_max_node_to_optimize", loopMaxNodeToOptimize, 4);
  ROS_INFO_STREAM("loop_max_node_to_optimize = " << loopMaxNodeToOptimize);

  pn.param<double>("loop_k_acc_dist", loopKAccDist, 0.05);
  ROS_INFO_STREAM("loop_k_acc_dist = " << loopKAccDist);

  pn.param<double>("loop_dist_dynmic_max", loopDistDynmicMax, 1.0);
  ROS_INFO_STREAM("loop_dist_dynmic_max = " << loopDistDynmicMax);

  // Объявили паблишеров
  pubMapPointCloud = n.advertise<sensor_msgs::PointCloud2>("/point_map", 1);
  pubOdom = n.advertise<nav_msgs::Odometry>("/odom", 100);
  pubPointCloudFilter = n.advertise<sensor_msgs::PointCloud2>("/filtered_point", 1);
  pubGraphRviz = n.advertise<visualization_msgs::MarkerArray>("/map_graph", 1);

  // Инициализация синхронезированного callback
  subPointCloud.subscribe(n, "/velodyne_points", 1);
  subOdom.subscribe(n, "/odometry/filtered", 1);

  sync = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>>(
             message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry>(10), subPointCloud, subOdom);

  sync->registerCallback(boost::bind(&PointcloudMatching::callback, this, _1, _2));


  // Инициализация параметров NDT-OMP
  registrationMethod.setMaximumIterations(NDTMaximumIterations);
  registrationMethod.setTransformationEpsilon(NDTTransformationEpsilon);
  registrationMethod.setResolution(NDTResolution);
  registrationMethod.setNeighborhoodSearchMethod(pclomp::DIRECT7);

  loopRegistrationMethod.setMaximumIterations(NDTMaximumIterations);
  loopRegistrationMethod.setTransformationEpsilon(NDTTransformationEpsilon);
  loopRegistrationMethod.setResolution(NDTResolution);
  loopRegistrationMethod.setNeighborhoodSearchMethod(pclomp::DIRECT7);
}


PointcloudMatching::~PointcloudMatching()
{
  // Ожидаем завершения работы паралельных потоков
  submapThread.wait();
  loopThread.wait();
}


void PointcloudMatching::callback(const sensor_msgs::PointCloud2::ConstPtr& cloudMsg, const nav_msgs::Odometry::ConstPtr& odomMsg)
{
  // Превели одометриию к tf формату, поскольку в нём проще считать
  tf2::Transform odomTfTransform = odomToTfTransform(odomMsg);

  // Если это первое сообщение, то провели инициализацию и больше сюда не заходим
  if (map.submapVector.size() == 0)
  {
    // Автоопределение фреймов
    sensorFrameId = cloudMsg->header.frame_id;  // Лидара
    odomChildFraimId = odomMsg->child_frame_id; // Робота в одометрии

    // Проинициализировали первую подкарту
    // Запомнили odom начала подкарты
    lastAddPointCloudTransform = odomTfTransform;

    // Преобразовали к PCL формату
    pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
    pcl::fromROSMsg(*cloudMsg, *cloud);

    // Получили преобразование из sensorFrameId в odomChildFraimId
    tf2::Transform sensorToOdom = getTransformFromTf(tfBuffer, sensorFrameId, odomChildFraimId, ros::Time(0));

    tf2::Transform zeroTransform;
    zeroTransform.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    zeroTransform.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

    // Созание подкарты
    std::shared_ptr<CloudStructure> cloudStr(new CloudStructure);
    cloudStr->sensorToOdom = sensorToOdom;
    cloudStr->odomTfTransform = odomTfTransform;

    std::shared_ptr<SubmapStructure> submapStr(new SubmapStructure);

    submapStr->submapStartOdomTf = odomTfTransform;
    submapStr->inMapPose = zeroTransform;
    submapStr->node = map.add_node(zeroTransform);
    submapStr->node->setFixed(true);
    submapStr->cloudRaw = boost::make_shared<pcl::PointCloud<PointType>>();

    // Преобразовали облако точек в координаты odomChildFraimId
    pcl::PointCloud<PointType>::Ptr cloudTransform(new pcl::PointCloud<PointType>());
    pcl::transformPointCloud(*cloud, *cloudTransform, tfTransformToEigenMatrix(sensorToOdom.inverse()));

    *(submapStr->cloudRaw) += *(cloudTransform);
    cloudStr->submapToSensor = sensorToOdom.inverse();
    cloudStr->cloudTransformToSubmap = cloudTransform;

    submapStr->cloudVector.push_back(cloudStr);

    map.submapVector.push_back(submapStr);

    // Опубликовали нулевую одометрию
    pubOdomAndTf(zeroTransform, odomMsg->header.stamp);

    // Инициализировали паралелный поток с замыканием цикла
    if (enableLoopClosing)
    {
      loopThread = std::async(std::launch::async, &PointcloudMatching::tryToOptimization, this);
    }

    return;
  }

  // Публикую одометрию
  // Сейчас  указывает на позицию поледней совемещённой подкарты (то есть на lastLast), submapToOdom на позицию одометрии в новой подкарте
  if(map.submapVector.size()>2)
  {
    pubOdomAndTf(map.submapVector[map.submapVector.size()-3]->inMapPose * map.getLastLastSubmapStartTf().inverse() * odomTfTransform, odomMsg->header.stamp);
  }
  else
  {
    pubOdomAndTf(map.submapVector[0]->inMapPose * map.getLastLastSubmapStartTf().inverse() * odomTfTransform, odomMsg->header.stamp);
  }


  // Проверка того, что новое облако точек удалено от предыдущего на минимальное расстояние
  tf2::Transform deltaPointcloudTransform = lastAddPointCloudTransform.inverse() * odomTfTransform;
  double deltaTransSqr = deltaPointcloudTransform.getOrigin().length2();
  double deltaRot = deltaPointcloudTransform.getRotation().getAngle(); // Даже модуль не нужен, так как угл от 0 до 2p
  if ((deltaTransSqr<minSqrTransformTrans) && (deltaRot<minTransformRotation))
  {
    return;
  }
  // Запомнили позицию добавленного облака точек
  lastAddPointCloudTransform = odomTfTransform;

  // Инициализация новой подкарты
  if (map.submapVector.back()->cloudVector.size() == 0)
  {
    // Запомнили odom начала подкарты
    map.submapVector.back()->submapStartOdomTf = odomTfTransform;
  }

  // Если подкарта инициплизирована то:
  // Получили преобразование между текущим положением сенсора и началом подкарты
  tf2::Transform submapToOdom = map.submapVector.back()->submapStartOdomTf.inverse() * odomTfTransform;
  tf2::Transform sensorToOdom = getTransformFromTf(tfBuffer, sensorFrameId, odomChildFraimId, ros::Time(0));
  tf2::Transform sensorToSubmap = sensorToOdom * submapToOdom.inverse();


  // Преобразовали облако точек к PCL формату
  pcl::PointCloud<PointType>::Ptr cloud(new pcl::PointCloud<PointType>());
  pcl::fromROSMsg(*cloudMsg, *cloud);

  // Преобразовали новое облако точек к подкарте
  pcl::PointCloud<PointType>::Ptr cloudTransform(new pcl::PointCloud<PointType>());
  pcl::transformPointCloud(*cloud, *cloudTransform, tfTransformToEigenMatrix(sensorToSubmap.inverse()));

  // Преобразованное облако точек добавили к подкарте
  *(map.submapVector.back()->cloudRaw) += *cloudTransform;

  // Увеличили счётчик облаков в подкарте на 1
  std::shared_ptr<CloudStructure> cloudStr(new CloudStructure);
  //cloudStr->cloud = cloud;
  cloudStr->cloudTransformToSubmap = cloudTransform;
  cloudStr->submapToSensor = sensorToSubmap.inverse();
  cloudStr->sensorToOdom = sensorToOdom;
  cloudStr->odomTfTransform = odomTfTransform;
  map.submapVector.back()->cloudVector.push_back(cloudStr);


  // Определение точек относящихся к динамическим
  if (enableDynamicFilter)
  {
    // Проверили, что в подкарте минимум 2 облака.
    if (map.submapVector.back()->cloudVector.size()>=2)
    {
      PointType castingPoint;
      castingPoint.x = static_cast<float>(map.submapVector.back()->cloudVector[map.submapVector.back()->cloudVector.size()-1]->submapToSensor.getOrigin().x());
      castingPoint.y = static_cast<float>(map.submapVector.back()->cloudVector[map.submapVector.back()->cloudVector.size()-1]->submapToSensor.getOrigin().y());
      castingPoint.z = static_cast<float>(map.submapVector.back()->cloudVector[map.submapVector.back()->cloudVector.size()-1]->submapToSensor.getOrigin().z());

      dynamicCloudFilter((map.submapVector.back()->cloudVector[map.submapVector.back()->cloudVector.size()-1]->cloudTransformToSubmap),
                         (map.submapVector.back()->cloudVector[map.submapVector.back()->cloudVector.size()-2]->cloudTransformToSubmap),
                         castingPoint, map.submapVector.back()->pointToFilter);

      // Для простоты первое облко со вторым фильтрую, а не с предыущей подкартой
      if (map.submapVector.back()->cloudVector.size()==2)
      {
        PointType castingPointInv;
        castingPointInv.x = static_cast<float>(map.submapVector.back()->cloudVector[map.submapVector.back()->cloudVector.size()-2]->submapToSensor.getOrigin().x());
        castingPointInv.y = static_cast<float>(map.submapVector.back()->cloudVector[map.submapVector.back()->cloudVector.size()-2]->submapToSensor.getOrigin().y());
        castingPointInv.z = static_cast<float>(map.submapVector.back()->cloudVector[map.submapVector.back()->cloudVector.size()-2]->submapToSensor.getOrigin().z());

        dynamicCloudFilter((map.submapVector.back()->cloudVector[map.submapVector.back()->cloudVector.size()-2]->cloudTransformToSubmap),
                           (map.submapVector.back()->cloudVector[map.submapVector.back()->cloudVector.size()-1]->cloudTransformToSubmap),
                           castingPointInv, map.submapVector.back()->pointToFilter);
      }
    }
  }


  // Если набрали достаточное количество облаков точек в подкарту, то опубликовали результат
  if (map.submapVector.back()->cloudVector.size()>maxNumOfCloudInSubmap)
  {
    ROS_INFO_STREAM("Submap");

    // Нашли предпологаемое преобразование между подкартами
    tf2::Transform deltaSubmapOdomTransform = map.getLastSubmapStartTf().inverse() * map.submapVector.back()->submapStartOdomTf;

    // Если регистрация подкарт была запущена, то дождался её завершения
    if (submapThread.valid())
    {
      submapThread.wait();
    }

    // К этому моменту паралельный поток закончил работать, так что обновили регистрацию
    if(map.submapVector.size()>2)
    {
      auto tmp = submapThread.get();
      tf2::Transform registrationSubmapTransform = std::get<0>(tmp);
      double fitnessScore = std::get<1>(tmp);
      double dynamicPointsPersent = std::get<2>(tmp);

      map.submapVector[map.submapVector.size()-2]->inMapPose = eigenMatrixToTfTransform(map.submapVector[map.submapVector.size()-3]->node->estimate().matrix().cast<float>()) * registrationSubmapTransform;
      map.submapVector[map.submapVector.size()-2]->node = map.add_node(map.submapVector[map.submapVector.size()-2]->inMapPose);
      map.add_edge(map.submapVector[map.submapVector.size()-3]->node, map.submapVector[map.submapVector.size()-2]->node, registrationSubmapTransform, fitnessScore, dynamicPointsPersent);

      testedMatching.insert({map.submapVector[map.submapVector.size()-3]->node->id(), map.submapVector[map.submapVector.size()-2]->node->id()});
      testedMatching.insert({map.submapVector[map.submapVector.size()-2]->node->id(), map.submapVector[map.submapVector.size()-3]->node->id()});

      // Если замыкание цикла включено, то проверяем работает ли он и перезапускаем если закончил, так как добавилось новое ребро.
      if (enableLoopClosing)
      {
        if (loopThread.wait_for(0ms) == std::future_status::ready)
        {
          loopThread = std::async(std::launch::async, &PointcloudMatching::tryToOptimization, this);
        }
      }

      // Публикую построекнную карту и граф
      pubMapAndGrapth();
    }

    if (map.submapVector.size()>1)
    {
      submapThread = std::async(std::launch::async, &PointcloudMatching::submapRegistration, this, map.submapVector[map.submapVector.size()-2], map.submapVector.back(), deltaSubmapOdomTransform);
    }
    else
    {
      submapThread = std::async(std::launch::async, &PointcloudMatching::submapRegistration, this, nullptr, map.submapVector.back(), deltaSubmapOdomTransform);
    }

    // Создали новую подкарту
    std::shared_ptr<SubmapStructure> submapStr(new SubmapStructure);
    submapStr->cloudRaw = boost::make_shared<pcl::PointCloud<PointType>>();
    map.submapVector.push_back(submapStr);
  }
}


// Паралельная функция обрабоки подкарт
std::tuple<tf2::Transform, double, double> PointcloudMatching::submapRegistration(const std::shared_ptr<SubmapStructure> lastSubmap, const std::shared_ptr<SubmapStructure> currentSubmap, const tf2::Transform initSubmapTransform)
{
  // Сначала сделали downsample, чтобы избавиться от точек, которые отражают одну и туже область пространства
  pcl::PointCloud<PointType>::Ptr submapPointCloudDownsample(new pcl::PointCloud<PointType>());

  pcl::VoxelGrid<PointType> sor;
  sor.setInputCloud(currentSubmap->cloudRaw);
  sor.setLeafSize(voxelFilterLeafSize, voxelFilterLeafSize, voxelFilterLeafSize);
  sor.filter(*submapPointCloudDownsample);

  double dynamicPointsPersent = 0.0;
  // Если включина фильтрация динамики, то фильтруем все точки близкие к динамическим
  if (enableDynamicFilter)
  {
    pcl::PointCloud<PointType>::Ptr submapFilter(new pcl::PointCloud<PointType>());
    dynamicPointsPersent = deletePointsFromSubmap(submapPointCloudDownsample, submapFilter, currentSubmap->pointToFilter);
    currentSubmap->cloudRaw = submapFilter;
  }
  else
  {
    currentSubmap->cloudRaw = submapPointCloudDownsample;
  }

  // Если это первая подкарта, то просто запомнили её
  if (lastSubmap==nullptr)
  {
    tf2::Transform registrationSubmapTransform;
    registrationSubmapTransform.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
    registrationSubmapTransform.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));
    return std::make_tuple(registrationSubmapTransform, 0.0, 0.0);
  }


  // Задание параметров и вызов алгоритма регистрации двух подкарт
  registrationMethod.setInputSource(currentSubmap->cloudRaw);
  registrationMethod.setInputTarget(lastSubmap->cloudRaw);

  pcl::PointCloud<PointType>::Ptr alignCloud(new pcl::PointCloud<PointType>());
  registrationMethod.align(*alignCloud, tfTransformToEigenMatrix(initSubmapTransform));

  tf2::Transform registrationSubmapTransform;

  // Вернули найденное преобразование
  if ( (registrationMethod.hasConverged()) && (registrationMethod.getFitnessScore() < registrationMaxFitnessScore))
  {
    registrationSubmapTransform = eigenMatrixToTfTransform(registrationMethod.getFinalTransformation());
  }
  else
  {
    registrationSubmapTransform = initSubmapTransform;
  }

  double fitnessScore = registrationMethod.getFitnessScore();

  std::cout << "Registration has converged:" << registrationMethod.hasConverged() << " score: " << registrationMethod.getFitnessScore() << std::endl;

  return std::make_tuple(registrationSubmapTransform, fitnessScore, dynamicPointsPersent);
}

// Публикация odom сообщения и tf (внутри происходит преобразование к publishFrame)
void PointcloudMatching::pubOdomAndTf(const tf2::Transform& transform, const ros::Time& time)
{
  // Опубликовали соответсвующее преобразование, как одометрию
  nav_msgs::Odometry odomMsg;
  odomMsg.header.stamp = time;
  odomMsg.header.frame_id = mapFrameId;
  odomMsg.child_frame_id = odomChildFraimId;
  odomMsg.pose.pose.position.x = transform.getOrigin().x();
  odomMsg.pose.pose.position.y = transform.getOrigin().y();
  odomMsg.pose.pose.position.z = transform.getOrigin().z();
  odomMsg.pose.pose.orientation.x = transform.getRotation().x();
  odomMsg.pose.pose.orientation.y = transform.getRotation().y();
  odomMsg.pose.pose.orientation.z = transform.getRotation().z();
  odomMsg.pose.pose.orientation.w = transform.getRotation().w();
  odomMsg.twist.twist.linear.x = 0.0;
  odomMsg.twist.twist.linear.y = 0.0;
  odomMsg.twist.twist.angular.z = 0.0;
  pubOdom.publish(odomMsg);

  // И tf опубликовали если необходимо
  if (enableTfPub)
  {
    // В данную функцию передаётся преобразование между mapFrameId и odomChildFraimId, но может быть необходимость к odom перевести
    // Изменили publish на odomChildFraimId
    tf2::Transform mapToPublishTransform;
    if (publishFraimId != odomChildFraimId)
    {
      tf2::Transform odomToPublishTransfor = getTransformFromTf(tfBuffer, odomChildFraimId, publishFraimId, ros::Time(0));
      mapToPublishTransform = transform * odomToPublishTransfor;
    }
    else
    {
      mapToPublishTransform = transform;
    }

    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = time;
    transformStamped.header.frame_id = mapFrameId;
    transformStamped.child_frame_id = publishFraimId;
    transformStamped.transform.translation.x = mapToPublishTransform.getOrigin().x();
    transformStamped.transform.translation.y = mapToPublishTransform.getOrigin().y();
    transformStamped.transform.translation.z = mapToPublishTransform.getOrigin().z();
    transformStamped.transform.rotation.x = mapToPublishTransform.getRotation().x();
    transformStamped.transform.rotation.y = mapToPublishTransform.getRotation().y();
    transformStamped.transform.rotation.z = mapToPublishTransform.getRotation().z();
    transformStamped.transform.rotation.w = mapToPublishTransform.getRotation().w();
    tfBroadcaster.sendTransform(transformStamped);
  }

  return;
}


// Публикация графа и карты
void PointcloudMatching::pubMapAndGrapth()
{
  // Так-как для других ещё submapRegistration не случился
  if(map.submapVector.size()>2)
  {
    // Публикация карты в виде облака точек
    if (pubMapPointCloud.getNumSubscribers()>0)
    {
      pcl::PointCloud<PointType> mapCloud;
      for(size_t i = 0; i< map.submapVector.size()-1; ++i)
      {
        pcl::PointCloud<PointType> cloudTransform;
        pcl::transformPointCloud(*(map.submapVector[i]->cloudRaw), cloudTransform, tfTransformToEigenMatrix(map.submapVector[i]->inMapPose));
        mapCloud += cloudTransform;
      }

      sensor_msgs::PointCloud2 MapPointCloudMsg;
      pcl::toROSMsg(mapCloud, MapPointCloudMsg);
      MapPointCloudMsg.header.frame_id = mapFrameId;
      MapPointCloudMsg.header.stamp =  ros::Time::now();
      pubMapPointCloud.publish(MapPointCloudMsg);
    }

    // Публикация графа карты
    if (pubGraphRviz.getNumSubscribers()>0)
    {
      // Узлы
      visualization_msgs::MarkerArray mapRvizGraph;
      mapRvizGraph.markers.reserve(1+map.submapVector.size());
      for(size_t i = 0; i< map.submapVector.size()-1; ++i)
      {
        Eigen::Vector3d pos = map.submapVector[i]->node->estimate().translation();

        visualization_msgs::Marker marker;
        marker.header.frame_id = mapFrameId;
        marker.header.stamp = ros::Time::now();
        marker.ns = "nodes";
        marker.id = static_cast<int>(i);
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = pos.x();
        marker.pose.position.y = pos.y();
        marker.pose.position.z = pos.z();
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        mapRvizGraph.markers.push_back(marker);
      }

      // Рёбра
      visualization_msgs::Marker markerEdge;
      markerEdge.header.frame_id = mapFrameId;
      markerEdge.header.stamp = ros::Time::now();
      markerEdge.ns = "edges";
      markerEdge.id = 100000; // Постоянный константный id, чтобы перезаписывать данные рёбер
      markerEdge.action = visualization_msgs::Marker::ADD;
      markerEdge.type = visualization_msgs::Marker::LINE_LIST;
      markerEdge.pose.orientation.x = 0.0;
      markerEdge.pose.orientation.y = 0.0;
      markerEdge.pose.orientation.z = 0.0;
      markerEdge.pose.orientation.w = 1.0;
      markerEdge.scale.x = 0.03;
      markerEdge.scale.y = 0.03;
      markerEdge.scale.z = 0.03;

      std_msgs::ColorRGBA color_nir;
      color_nir.a = 1.0;
      color_nir.r = 1.0;
      color_nir.g = 0.0;
      color_nir.b = 1.0;

      std_msgs::ColorRGBA color_loop;
      color_loop.a = 1.0;
      color_loop.r = 0.0;
      color_loop.g = 0.0;
      color_loop.b = 1.0;

      markerEdge.points.reserve(2 * map.graph->edges().size());
      for(auto edge_itr = map.graph->edges().begin(); edge_itr != map.graph->edges().end(); edge_itr++)
      {
        g2o::HyperGraph::Edge* edge = *edge_itr;
        g2o::EdgeSE3* edge_se3 = dynamic_cast<g2o::EdgeSE3*>(edge);
        if(edge)
        {
          g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[0]);
          g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(edge_se3->vertices()[1]);

          geometry_msgs::Point point1;
          point1.x = v1->estimate().translation().x();
          point1.y = v1->estimate().translation().y();
          point1.z = v1->estimate().translation().z();
          markerEdge.points.push_back(point1);

          geometry_msgs::Point point2;
          point2.x = v2->estimate().translation().x();
          point2.y = v2->estimate().translation().y();
          point2.z = v2->estimate().translation().z();
          markerEdge.points.push_back(point2);

          if (abs(v1->id()-v2->id())==1)
          {
            markerEdge.colors.push_back(color_nir);
            markerEdge.colors.push_back(color_nir);
          }
          else
          {
            markerEdge.colors.push_back(color_loop);
            markerEdge.colors.push_back(color_loop);
          }
        }
      }
      mapRvizGraph.markers.push_back(markerEdge);

      pubGraphRviz.publish(mapRvizGraph);
    }
  }

  map.graph->save("/home/baltic/graph_dynamic_slam_ws/my_graph.g2o");
}

void PointcloudMatching::tryToOptimization()
{
  size_t grSize = map.graph->vertices().size();
  if (grSize>loopMinDeltaId+1)
  {
    int currentOptim = 0;
    for(size_t i = grSize-1; i>=loopMinDeltaId; --i)
    {
      for(int j = static_cast<int>(i-loopMinDeltaId); j>=0; --j) // Бессмысленно искать замыкания цикла между слишком ближнемы узлами
      {
        g2o::VertexSE3* vert_i = dynamic_cast<g2o::VertexSE3*>(map.graph->vertex(static_cast<int>(i)));
        g2o::VertexSE3* vert_j = dynamic_cast<g2o::VertexSE3*>(map.graph->vertex(static_cast<int>(j)));

        // Если уже проверяли регистрацию
        if (testedMatching.find({vert_i->id(), vert_j->id()}) != testedMatching.end())
        {
          continue;
        }

        // Между узлами ддолжна накопится ошибка
        std::pair<double, int> dist = findShortestPathLength(map.graph, vert_i, vert_j);
        if ((dist.second>static_cast<int>(loopMinDeltaId))&&(dist.first>loopMinAccumulateDist))
        {
          double dx = vert_i->estimate().translation().x() - vert_j->estimate().translation().x();
          double dy = vert_i->estimate().translation().y() - vert_j->estimate().translation().y();
          double dz = vert_i->estimate().translation().z() - vert_j->estimate().translation().z();
          double directDist = sqrt(dx*dx + dy*dy + dz*dz);

          // Узлы не олжны быть слишком далеко расположены
          if (directDist < loopMaxDirectDist + loopKAccDist * dist.first + loopDistDynmic)
          {
            // Нашли кандидата
            std::cout << "Find candidat between " << i << " " << j << " "; //<< std::endl;
            //std::cout << "Try to match" << std::endl;
            //std::cout << "i = " << i << " j = " << j << " dist = " << dist.first << " dist_nodee = " << dist.second << " directDist = " << directDist << std::endl;

            // Пробуем сматчить
            Eigen::Matrix4f initTransform = (vert_j->estimate().inverse()*vert_i->estimate()).matrix().cast<float>();



            loopRegistrationMethod.setInputSource(map.submapVector[i]->cloudRaw);
            loopRegistrationMethod.setInputTarget(map.submapVector[static_cast<size_t>(j)]->cloudRaw);

            pcl::PointCloud<PointType>::Ptr alignCloud(new pcl::PointCloud<PointType>());
            loopRegistrationMethod.align(*alignCloud, initTransform);

            // Сообщили получилось ли
            std::cout << "Loop has converged:" << loopRegistrationMethod.hasConverged() << " score: " << loopRegistrationMethod.getFitnessScore() << std::endl;

            // Чтобы больше не пробовать даже если не получилось
            testedMatching.insert({vert_i->id(), vert_j->id()});
            testedMatching.insert({vert_j->id(), vert_i->id()});

            //std::cout << "-------------------------" << std::endl;
            //std::cout << initTransform << std::endl;
            //std::cout << loopRegistrationMethod.getFinalTransformation() << std::endl;

            // Создали ребро
            if((loopRegistrationMethod.hasConverged()) && (loopRegistrationMethod.getFitnessScore()<=loopMaxFitnessScore))
            {
              if (enableDynamicFilter)
              {
                double dynamicPointsPersent = 0.0; // submapFilter(map.submapVector[i], map.submapVector[static_cast<size_t>(j)], eigenMatrixToTfTransform(loopRegistrationMethod.getFinalTransformation().inverse()));
                if (dynamicPointsPersent > 0.75) // Фильтрация большей части сцены
                {
                    loopDistDynmic = loopDistDynmicMax;
                }
                else
                {
                  loopDistDynmic = 0.0;
                  map.add_edge(vert_i, vert_j, eigenMatrixToTfTransform(loopRegistrationMethod.getFinalTransformation().inverse()), loopRegistrationMethod.getFitnessScore(), dynamicPointsPersent);
                  optimize();
                }
              }
              else
              {
                map.add_edge(vert_i, vert_j, eigenMatrixToTfTransform(loopRegistrationMethod.getFinalTransformation().inverse()), loopRegistrationMethod.getFitnessScore(), 0.0);
                optimize();
              }
            }

            currentOptim++;
            if (currentOptim>=loopMaxNodeToOptimize)
            {
              return;
            }
          }
        }
      }
    }
  }
}


void PointcloudMatching::optimize()
{
 //map.graph->save("/home/baltic/graph_dynamic_slam_ws/before.g2o");

  if(map.graph->edges().size() < 4) // Просто, чтобы не оптимизировать слишком маленький граф
  {
    return;
  }

  std::cout << std::endl;
  std::cout << "--- pose graph optimization ---" << std::endl;
  std::cout << "nodes: " << map.graph->vertices().size() << "   edges: " << map.graph->edges().size() << std::endl;
  std::cout << "optimizing... " << std::flush;

  map.graph->initializeOptimization();
  map.graph->computeInitialGuess();
  map.graph->computeActiveErrors();
  map.graph->setVerbose(false);

  double chi2 = map.graph->chi2();

  map.optMutex.lock();
  int iterations = map.graph->optimize(loopMaxIterationsOptimize);
  map.optMutex.unlock();

 //map.graph->save("/home/baltic/graph_dynamic_slam_ws/after.g2o");
 //exit(0);
  std::cout << "Optimization done" << std::endl;
  std::cout << "iterations: " << iterations << " / " << loopMaxIterationsOptimize << std::endl;
  std::cout << "chi2: (before)" << chi2 << " -> (after)" << map.graph->chi2() << std::endl;
}

void PointcloudMatching::dynamicCloudFilter(const pcl::PointCloud<PointType>::Ptr castingCloud, const pcl::PointCloud<PointType>::Ptr mapCloud, const PointType castingPose, std::queue<PointType>& pointToFilter)
{
  std::vector<bool> dynamicPoints(mapCloud->points.size());

  // Нахоим массив расстояний второго облака
  std::vector<float> mapCloud_d02x(mapCloud->points.size());
  std::vector<float> mapCloud_d02y(mapCloud->points.size());
  std::vector<float> mapCloud_d02z(mapCloud->points.size());
  std::vector<float> mapCloud_Rsqr(mapCloud->points.size());

  for(size_t i=0; i< mapCloud->points.size(); i++)
  {
    mapCloud_d02x[i] = mapCloud->points[i].x - castingPose.x;
    mapCloud_d02y[i] = mapCloud->points[i].y - castingPose.y;
    mapCloud_d02z[i] = mapCloud->points[i].z - castingPose.z;
    mapCloud_Rsqr[i] = mapCloud_d02x[i] * mapCloud_d02x[i] + mapCloud_d02y[i] * mapCloud_d02y[i] + mapCloud_d02z[i] * mapCloud_d02z[i];
  }

  // castingPose (0)
  // Итерируемся по облаку точек, который кастим (1)
  #pragma omp parallel for num_threads(8)
  for(size_t j=0; j< castingCloud->points.size(); j++)
  {
    float d01x = castingCloud->points[j].x - castingPose.x;
    float d01y = castingCloud->points[j].y - castingPose.y;
    float d01z = castingCloud->points[j].z - castingPose.z;
    float castingCloud_Rsqr = d01x*d01x + d01y*d01y + d01z*d01z;

    // Защита от крайнемаловероятного деления на ноль
    if (fabsf(d01x) < 0.000001f)
    {
      continue;
    }

    float d01yx = d01y / d01x;
    float d01zx = d01z / d01x;

    // Итерируемся по облаку точек, которое чистим
    for(size_t i=0; i< mapCloud->points.size(); i++)
    {
      if (fabsf(mapCloud_d02y[i] - d01yx * mapCloud_d02x[i]) < dynamicPointDl) // Кандидат по оси y
      {
        if (fabsf(mapCloud_d02z[i] - d01zx * mapCloud_d02x[i]) < dynamicPointDl) // Кандидат по оси z
        {
          if (((mapCloud_d02x[i]>0)&&(d01x>0)) || ((mapCloud_d02x[i]<0)&&(d01x<0)))
          {
            if (mapCloud_Rsqr[i]+dynamicPointDr<castingCloud_Rsqr)
            {
              dynamicPoints[i] = true;
            }
          }
        }
      }
    }
  }

  // Заполняем очередь, чтобы потом отфильтровать
  for(size_t i =0; i < dynamicPoints.size(); ++i)
  {
    if (dynamicPoints[i])
    {
      pointToFilter.push(mapCloud->points[i]);
    }
  }
}

double PointcloudMatching::deletePointsFromSubmap(pcl::PointCloud<PointType>::Ptr cloudInput, pcl::PointCloud<PointType>::Ptr cloudOutput, std::queue<PointType>& pointToFilter)
{
  // Инициализировали KdTree для ускорения поиска
  pcl::KdTreeFLANN<PointType> kdtree;
  kdtree.setInputCloud(cloudInput);

  // Ищем соседей в заданном радиусе
  std::vector<int> indices;
  std::vector<float> sqrDist;
  // Чтобы избаится от повторов точек использую set
  std::set<int> indicesGlobal;

  // Нахожу все точки подлежащие удалению и фильтрую их
  while (!pointToFilter.empty())
  {
    kdtree.radiusSearch(pointToFilter.front(), dynamicPointRadToDelete, indices, sqrDist);
    pointToFilter.pop();
    for (auto& pointI : indices)
    {
      indicesGlobal.insert(pointI);
    }
  }

  // Запоминаю количество динамических точек
  double dynamicPointsPersent = static_cast<double>(indicesGlobal.size())/static_cast<double>(cloudInput->size());

  // Удаляю найденные точки
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  for (auto& pointI : indicesGlobal)
  {
    inliers->indices.push_back(pointI);
  }

  pcl::ExtractIndices<PointType> extract;
  extract.setInputCloud(cloudInput);
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*cloudOutput);

  return dynamicPointsPersent;
}

double PointcloudMatching::submapFilter(const std::shared_ptr<SubmapStructure> loopSubmapOne, const std::shared_ptr<SubmapStructure> loopSubmapSeccond, const tf2::Transform& foundTransform)
{
  // Поскольку я хочу понять только не произошла ли ошибка замыкания цикла, то мне достаточно проверить несколько лиддаров в поддкарте
  std::queue<PointType> pointToFilter;

  // Преобразую первые и последние данные лиддара второй подкарты в первую подкарту и фильтрую первую
  // Первое
  pcl::PointCloud<PointType>::Ptr cloudTransform1(new pcl::PointCloud<PointType>());
  pcl::transformPointCloud(*(loopSubmapSeccond->cloudVector.front()->cloudTransformToSubmap), *cloudTransform1, tfTransformToEigenMatrix(foundTransform));

  tf2::Transform castingPose1 = foundTransform * loopSubmapSeccond->cloudVector.front()->submapToSensor;

  PointType castingPoint1;
  castingPoint1.x = static_cast<float>(castingPose1.getOrigin().x());
  castingPoint1.y = static_cast<float>(castingPose1.getOrigin().y());
  castingPoint1.z = static_cast<float>(castingPose1.getOrigin().z());

  dynamicCloudFilter(cloudTransform1, (loopSubmapOne->cloudVector.front()->cloudTransformToSubmap), castingPoint1, pointToFilter);

  // Последние
  pcl::PointCloud<PointType>::Ptr cloudTransform2(new pcl::PointCloud<PointType>());
  pcl::transformPointCloud(*(loopSubmapSeccond->cloudVector.back()->cloudTransformToSubmap), *cloudTransform2, tfTransformToEigenMatrix(foundTransform));

  tf2::Transform castingPose2 = foundTransform * loopSubmapSeccond->cloudVector.back()->submapToSensor;

  PointType castingPoint2;
  castingPoint2.x = static_cast<float>(castingPose2.getOrigin().x());
  castingPoint2.y = static_cast<float>(castingPose2.getOrigin().y());
  castingPoint2.z = static_cast<float>(castingPose2.getOrigin().z());

  dynamicCloudFilter(cloudTransform2, (loopSubmapOne->cloudVector.back()->cloudTransformToSubmap), castingPoint2, pointToFilter);

  pcl::PointCloud<PointType>::Ptr cloudFilter(new pcl::PointCloud<PointType>());
  double dynamicPointsPersent = deletePointsFromSubmap(loopSubmapOne->cloudRaw, cloudFilter, pointToFilter);

  loopSubmapOne->cloudRaw = cloudFilter;

  return dynamicPointsPersent;
}


}
