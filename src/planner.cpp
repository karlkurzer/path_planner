#include "planner.h"
#include <tf2/utils.h>

using namespace HybridAStar;

Planner::Planner(rclcpp::Node::SharedPtr n) : n(n), path(n, false), smoothedPath(n, true), visualization(n) {
  pubStart = n->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/start", 1);

  if (Constants::manual) {
    subMap = n->create_subscription<nav_msgs::msg::OccupancyGrid>("/map", 1, std::bind(&Planner::setMap, this, std::placeholders::_1));
  } else {
    subMap = n->create_subscription<nav_msgs::msg::OccupancyGrid>("/occ_map", 1, std::bind(&Planner::setMap, this, std::placeholders::_1));
  }

  subGoal = n->create_subscription<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1, std::bind(&Planner::setGoal, this, std::placeholders::_1));
  subStart = n->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 1, std::bind(&Planner::setStart, this, std::placeholders::_1));

  tfBuffer = std::make_shared<tf2_ros::Buffer>(n->get_clock());
  listener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer, n, false);
};

void Planner::initializeLookups() {
  if (Constants::dubinsLookup) {
    Lookup::dubinsLookup(dubinsLookup);
  }
  Lookup::collisionLookup(collisionLookup);
}

void Planner::setMap(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
  if (Constants::coutDEBUG) {
    std::cout << "I am seeing the map..." << std::endl;
  }

  grid = map;
  configurationSpace.updateGrid(map);

  int height = map->info.height;
  int width = map->info.width;
  bool** binMap;
  binMap = new bool*[width];

  for (int x = 0; x < width; x++) { binMap[x] = new bool[height]; }

  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      binMap[x][y] = map->data[y * width + x] ? true : false;
    }
  }

  voronoiDiagram.initializeMap(width, height, binMap);
  voronoiDiagram.update();
  voronoiDiagram.visualize();
  for (int x = 0; x < width; x++) {
    delete[] binMap[x];
  }
  delete[] binMap;
//  delete[] binMap;

  if (!Constants::manual && tfBuffer->canTransform("map", "base_link", tf2::TimePointZero)) {
    try {
      transform = tfBuffer->lookupTransform("map", "base_link", tf2::TimePointZero);
      start.pose.pose.position.x = transform.transform.translation.x;
      start.pose.pose.position.y = transform.transform.translation.y;
      start.pose.pose.orientation = transform.transform.rotation;

      if (grid->info.height >= start.pose.pose.position.y && start.pose.pose.position.y >= 0 &&
          grid->info.width >= start.pose.pose.position.x && start.pose.pose.position.x >= 0) {
        validStart = true;
      } else  {
        validStart = false;
      }

      plan();
    } catch (const tf2::TransformException & ex) {
      RCLCPP_INFO(n->get_logger(), "Could not transform map to base_link: %s", ex.what());
    }
  }
}

void Planner::setStart(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initial) {
  float x = initial->pose.pose.position.x / Constants::cellSize;
  float y = initial->pose.pose.position.y / Constants::cellSize;
  tf2::Quaternion q;
  tf2::fromMsg(initial->pose.pose.orientation, q);
  float t = tf2::getYaw(q);

  geometry_msgs::msg::PoseStamped startN;
  startN.pose.position = initial->pose.pose.position;
  startN.pose.orientation = initial->pose.pose.orientation;
  startN.header.frame_id = "map";
  startN.header.stamp = n->now();

  std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid && grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validStart = true;
    start = *initial;

    if (Constants::manual) { plan();}

    pubStart->publish(startN);
  } else {
    std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

void Planner::setGoal(const geometry_msgs::msg::PoseStamped::SharedPtr end) {
  float x = end->pose.position.x / Constants::cellSize;
  float y = end->pose.position.y / Constants::cellSize;
  tf2::Quaternion q;
  tf2::fromMsg(end->pose.orientation, q);
  float t = tf2::getYaw(q);

  std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

  if (grid && grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
    validGoal = true;
    goal = *end;

    if (Constants::manual) { plan();}

  } else {
    std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
  }
}

void Planner::plan() {
  if (validStart && validGoal) {

    int width = grid->info.width;
    int height = grid->info.height;
    int depth = Constants::headings;
    int length = width * height * depth;
    Node3D* nodes3D = new Node3D[length]();
    Node2D* nodes2D = new Node2D[width * height]();

    float x = goal.pose.position.x / Constants::cellSize;
    float y = goal.pose.position.y / Constants::cellSize;
    tf2::Quaternion qGoal;
    tf2::fromMsg(goal.pose.orientation, qGoal);
    float t = tf2::getYaw(qGoal);
    t = Helper::normalizeHeadingRad(t);
    const Node3D nGoal(x, y, t, 0, 0, nullptr);

    x = start.pose.pose.position.x / Constants::cellSize;
    y = start.pose.pose.position.y / Constants::cellSize;
    tf2::Quaternion qStart;
    tf2::fromMsg(start.pose.pose.orientation, qStart);
    t = tf2::getYaw(qStart);
    t = Helper::normalizeHeadingRad(t);
    Node3D nStart(x, y, t, 0, 0, nullptr);

    rclcpp::Time t0 = n->now();

    visualization.clear();
    path.clear();
    smoothedPath.clear();

    Node3D* nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configurationSpace, dubinsLookup, visualization, n);

    if (nSolution != nullptr) {
      smoother.tracePath(nSolution);
      path.updatePath(smoother.getPath());
      smoother.smoothPath(voronoiDiagram);
      smoothedPath.updatePath(smoother.getPath());

      rclcpp::Time t1 = n->now();
      rclcpp::Duration d = t1 - t0;
      std::cout << "TIME in ms: " << d.seconds() * 1000 << std::endl;

      path.publishPath();
      path.publishPathNodes();
      path.publishPathVehicles();
      smoothedPath.publishPath();
      smoothedPath.publishPathNodes();
      smoothedPath.publishPathVehicles();
      visualization.publishNode3DCosts(nodes3D, width, height, depth);
      visualization.publishNode2DCosts(nodes2D, width, height);
    }

//    delete [] nodes3D;
//    delete [] nodes2D;

  } else {
    std::cout << "missing goal or start" << std::endl;
  }
}
