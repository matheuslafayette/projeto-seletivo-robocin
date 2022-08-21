#include "rrtstar.h"

RRTSTAR::RRTSTAR() {
  obstacles = new Obstacles;
  startPos.setX(START_POS_X);
  startPos.setY(START_POS_Y);
  endPos.setX(END_POS_X);
  endPos.setY(END_POS_Y);
  root = new Node;
  root->parent = NULL;
  root->position = startPos;
  root->orientation = START_ORIENT;
  root->cost = 0.0;
  lastNode = root;
  nodes.push_back(root);
  step_size = 18;
  max_iter = 3000;
}

/**
 * @brief Initialize root node of RRTSTAR.
 */
void RRTSTAR::initialize() {
  root = new Node;
  root->parent = NULL;
  root->position = startPos;
  root->orientation = START_ORIENT;
  root->cost = 0.0;
  lastNode = root;
  nodes.push_back(root);
}

void RRTSTAR::setInitPos(Point startPos) {

  this->startPos.setX(startPos.x());
  this->startPos.setY(startPos.y());
}

void RRTSTAR::setEndPos(Point endPos) {

  this->endPos.setX(endPos.x());
  this->endPos.setY(endPos.y());
}

/**
 * @brief Generate a random node in the field.
 * @return
 */
Node* RRTSTAR::getRandomNode() {
  Node* ret;
  Point point(drand48() * WORLD_WIDTH - WORLD_WIDTH / 2,
              drand48() * WORLD_HEIGHT - WORLD_HEIGHT / 2);
  float orient = drand48() * 2 * 3.142;
  ret = new Node;
  ret->position = point;
  ret->orientation = orient;
  return ret;
}

/**
 * @brief Helper method to find distance between two positions.
 * @param p
 * @param q
 * @return
 */
double RRTSTAR::distance(Point& p, Point& q) {
  Point v = p - q;
  return sqrt(powf(v.x(), 2) + powf(v.y(), 2));
}

/**
 * @brief Get nearest node from a given configuration/position.
 * @param point
 * @return
 */
Node* RRTSTAR::nearest(Point point) {
  float minDist = 1e9;
  Node* closest = NULL;
  for (int i = 0; i < (int) nodes.size(); i++) {
    double dist = distance(point, nodes[i]->position);
    if (dist < minDist) {
      minDist = dist;
      closest = nodes[i];
    }
  }
  return closest;
}

/**
 * @brief Get neighborhood nodes of a given configuration/position.
 * @param point
 * @param radius
 * @param out_nodes
 * @return
 */
void RRTSTAR::near(Point point, float radius, vector<Node*>& out_nodes) {
  for (int i = 0; i < (int) nodes.size(); i++) {
    double dist = distance(point, nodes[i]->position);
    if (dist < radius) {
      out_nodes.push_back(nodes[i]);
    }
  }
}

/**
 * @brief Find a configuration at a distance step_size from nearest node to random node.
 * @param q
 * @param qNearest
 * @return
 */
Point RRTSTAR::newConfig(Node* q, Node* qNearest) {
  Point to = q->position;
  Point from = qNearest->position;
  Point intermediate = to - from;
  intermediate = intermediate / intermediate.norm();
  Point pos = from + step_size * intermediate;
  Point ret(pos.x(), pos.y());
  return ret;
}

/**
 * @brief Find a configuration at a distance step_size from nearest node to random node
 * followin dynamic constraints of a nonholonomic robot (Dubins motion model).
 * @param q
 * @param qNearest
 * @return
 */
// Point RRTSTAR::newDubinConfig(Node* q, Node* qNearest, DubinsPath& path) {
//   double q0[] = {qNearest->position.x(), -qNearest->position.y(), qNearest->orientation};
//   double q1[] = {q->position.x(), -q->position.y(), q->orientation};
//   double turning_radius = BOT_TURN_RADIUS;
//   dubins_init(q0, q1, turning_radius, &path);
//   double qIntermediate[3] = {0};
//   dubins_path_sample(&path, step_size, qIntermediate);
//   Point ret(qIntermediate[0], -qIntermediate[1]);
//   return ret;
// }

/**
 * @brief Return trajectory cost.
 * @param q
 * @return
 */
double RRTSTAR::Cost(Node* q) {
  return q->cost;
}

/**
 * @brief Compute path cost.
 * @param qFrom
 * @param qTo
 * @return
 */
double RRTSTAR::PathCost(Node* qFrom, Node* qTo) {
  return distance(qTo->position, qFrom->position);
}

/**
 * @brief Add a node to the tree.
 * @param qNearest
 * @param qNew
 */
void RRTSTAR::add(Node* qNearest, Node* qNew) {
  qNew->parent = qNearest;
  qNew->cost = qNearest->cost + PathCost(qNearest, qNew);
  qNearest->children.push_back(qNew);
  nodes.push_back(qNew);
  lastNode = qNew;
}

/**
 * @brief Check if the last node is close to the end position.
 * @return
 */
bool RRTSTAR::reached() {
  if (distance(lastNode->position, endPos) < END_DIST_THRESHOLD)
    return true;
  return false;
}

void RRTSTAR::setStepSize(int step) {
  step_size = step;
}

void RRTSTAR::setMaxIterations(int iter) {
  max_iter = iter;
}

/**
 * @brief Delete all nodes using DFS technique.
 * @param root
 */
void RRTSTAR::deleteNodes(Node* root) {
  for (int i = 0; i < (int) root->children.size(); i++) {
    deleteNodes(root->children[i]);
  }
  delete root;
}

vector<Point> RRTSTAR::runRRTSTAR() {

  // RRTSTAR Algorithm
  for (int i = 0; i < max_iter; i++) {
    Node* q = getRandomNode();
    if (q) {
      Node* qNearest = nearest(q->position);
      if (distance(q->position, qNearest->position) > step_size) {
        Point newConfigPosOrient;
        // DubinsPath path;
        newConfigPosOrient = newConfig(q, qNearest);
        Point newConfigPos(newConfigPosOrient.x(), newConfigPosOrient.y());
        if (!obstacles->isSegmentInObstacle(newConfigPos, qNearest->position)) {
          Node* qNew = new Node;
          qNew->position = newConfigPos;
          qNew->orientation = newConfigPosOrient.angle();
          // qNew->path = path;
          vector<Node*> Qnear;
          near(qNew->position, step_size * RRTSTAR_NEIGHBOR_FACTOR, Qnear);
          // qDebug() << "Found Nearby " << Qnear.size() << "\n";
          Node* qMin = qNearest;
          double cmin = Cost(qNearest) + PathCost(qNearest, qNew);
          for (int j = 0; j < (int) Qnear.size(); j++) {
            Node* qNear = Qnear[j];
            if (!obstacles->isSegmentInObstacle(qNear->position, qNew->position) &&
                (Cost(qNear) + PathCost(qNear, qNew)) < cmin) {
              qMin = qNear;
              cmin = Cost(qNear) + PathCost(qNear, qNew);
            }
          }
          add(qMin, qNew);

          for (int j = 0; j < (int) Qnear.size(); j++) {
            Node* qNear = Qnear[j];
            if (!obstacles->isSegmentInObstacle(qNew->position, qNear->position) &&
                (Cost(qNew) + PathCost(qNew, qNear)) < Cost(qNear)) {
              Node* qParent = qNear->parent;
              // Remove edge between qParent and qNear
              qParent->children.erase(
                  std::remove(qParent->children.begin(), qParent->children.end(), qNear),
                  qParent->children.end());

              // Add edge between qNew and qNear
              qNear->cost = Cost(qNew) + PathCost(qNew, qNear);
              qNear->parent = qNew;
              qNew->children.push_back(qNear);
            }
          }
        }
      }
    }
    if (reached()) {
      std::cout << "reached!" << std::endl;
      break;
    }
  }

  Node* q;
  vector<Point> pathnodes;
  if (reached()) {
    q = lastNode;
  } else {
    // if not reached yet, then shortestPath will start from the closest node to end point.
    q = nearest(endPos);
    std::cout << "Exceeded max iterations!" << std::endl;
  }
  // generate shortest path to destination.
  while (q != NULL) {
    path.push_back(q);
    pathnodes.push_back(q->position);
    q = q->parent;
  }

  return pathnodes;
}
