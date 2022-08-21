#include "CustomPlayer.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include "./../PathPlanning/rrtstar.h"
//#include "./../PathPlanning/rrt.h"

CustomPlayer::CustomPlayer(int index, QThreadPool* threadPool) : Processing(index, threadPool) {
}

void CustomPlayer::buildParameters(Parameters::Handler& parameters) {
}

void CustomPlayer::connectModules(const Modules* modules) {
  connect(modules->vision(),
          &Vision::sendFrame,
          this,
          &CustomPlayer::receiveFrame,
          Qt::DirectConnection);

  connect(modules->vision(),
          &Vision::sendField,
          this,
          &CustomPlayer::receiveField,
          Qt::DirectConnection);
}

void CustomPlayer::init(const Modules* modules) {
}

void CustomPlayer::update() {
  shared->field.extract_to(field);
  if (auto f = shared->frame.get_optional_and_reset()) {
    if (auto it = f->allies().findById(index()); it != f->allies().end()) {
      robot = *it;
    }
    frame.emplace(*f);
  }
}

Point CustomPlayer::nextPointToGo(Point destiny) {

  vector<Point> pathnodes;
  int id = robot->id();

  if (destiny.distTo(robotpath[id].target) >= 200) {
    RRTSTAR* rrt = new RRTSTAR;
    rrt->setEndPos(destiny);
    rrt->setInitPos(robot->position());
    rrt->nodes.clear();
    rrt->initialize();

    for (Robot r : frame->allies()) {

      if (r.id() == robot->id())
        continue;

      Point topLeft = Point(r.position().x() + BOT_RADIUS, r.position().y() + BOT_RADIUS);
      Point bottomRight = Point(r.position().x() - BOT_RADIUS, r.position().y() - BOT_RADIUS);

      rrt->obstacles->addObstacle(topLeft, bottomRight);
    }

    for (Robot r : frame->enemies()) {

      Point topLeft = Point(r.position().x() + BOT_RADIUS, r.position().y() + BOT_RADIUS);
      Point bottomRight = Point(r.position().x() - BOT_RADIUS, r.position().y() - BOT_RADIUS);

      rrt->obstacles->addObstacle(topLeft, bottomRight);
    }

    rrt->setMaxIterations(2500);
    rrt->setStepSize(120);
    robotpath[id].pathNodes = rrt->runRRTSTAR();

    robotpath[id].currentNode = (int) robotpath[id].pathNodes.size() - 1;
    robotpath[id].nextPoint = robotpath[id].pathNodes.at(robotpath[id].currentNode);
    robotpath[id].target = robotpath[id].pathNodes.at(0);
    // target = destiny;
    delete rrt;
  } else if (robotpath[id].currentNode > 0 && robot->distTo(robotpath[id].nextPoint) <= 100) {

    robotpath[id].currentNode--;
    robotpath[id].nextPoint = robotpath[id].pathNodes.at(robotpath[id].currentNode);
  }

  return robotpath[id].nextPoint;
}

void CustomPlayer::exec() {
  if (!field || !frame || !robot || !frame->has_ball()) {
    return;
  }

  // TODO: here...
  // emit sendCommand(..

  std::vector<int> striker;
  striker.push_back(1);
  striker.push_back(3);
  std::vector<int> defense;
  defense.push_back(0);
  defense.push_back(2);
  defense.push_back(4);

  // goleiro
  const int idGoalkeeper = 5;
  bool isGoalkeeper = robot->id() == idGoalkeeper;
  bool goalkeeperInGoal = field->enemyPenaltyAreaContains(robot->position());

  int state = 5;
  Robot closestToBall = *frame->allies().removedById(5).closestTo(frame->ball().position());
  Robot closestAlly = *frame->allies().removedById(robot->id()).closestTo(robot->position());
  double distToClosestAlly = robot->distTo(closestAlly.position());
  double distXToBall = robot->position().x() - frame->ball().position().x();
  bool isClosestToBall = robot->id() == closestToBall.id();
  bool haveBall = closestToBall.distTo(frame->ball().position()) <= 120;
  // bool closeToGoal = closestToBall.position().isOnTheLeftOf(field->allyPenaltyAreaCenter());
  bool closeToGoal = closestToBall.distTo(field->allyGoalInsideCenter()) <= 3000;
  bool isStriker = std::find(striker.begin(), striker.end(), robot->id()) != striker.end();
  bool isDefender = std::find(defense.begin(), defense.end(), robot->id()) != defense.end();

  if (isClosestToBall) {
    if (haveBall) {
      if (closeToGoal)
        state = 0; // chuta para o gol
      else if (isStriker)
        state = 2; // vai para o gol == 2
      else
        state = 1; // toca aliado
    } else
      state = 3; // vai para a bola
  }

  else if (!isGoalkeeper) {
    state = 4; // vai para a bola (depedendo da posição vai devagar ou rapido)
  } else if (!goalkeeperInGoal)
    state = 7; // vai para o gol ser goleiro

  if (!field->contains(robot->position()) || field->allyPenaltyAreaContains(robot->position()))
    state = 8; // fora do campo entao volta pro campo

  switch (state) {

    case 0: {
      // chuta para o gol
      SSLMotion::RotateOnSelf kickGoal((field->allyGoalInsideCenter() - robot->position()).angle());
      SSLRobotCommand cKickGoal(kickGoal);
      cKickGoal.set_dribbler(true);
      const double tenDegrees = 0.17;
      if (abs(robot->angleTo(field->allyGoalInsideCenter())) <= tenDegrees) {

        cKickGoal.set_front(true);
        cKickGoal.set_kickSpeed(6);
      }

      emit sendCommand(sslNavigation.run(*robot, cKickGoal));
      break;
    }

    case 1: {
      // std::cout << "in" << std::endl;
      // pass to closest ally
      Robot closestRobot = *frame->allies().findById(1);
      double dist = 100000;
      for (int i : striker) {

        if (robot->distTo(*frame->allies().findById(i)) < dist) {

          dist = robot->distTo(*frame->allies().findById(i));
          closestRobot = *frame->allies().findById(i);
        }
      }
      SSLMotion::RotateOnSelf pass((closestRobot.position() - robot->position()).angle());
      SSLRobotCommand cPass(pass);
      cPass.set_dribbler(true);
      const double tenDegrees = 0.17;
      if (abs(robot->angleTo(closestRobot.position())) <= tenDegrees) {

        cPass.set_front(true);
        double kicksp = 1 + robot->distTo(closestRobot.position()) / 1500;
        cPass.set_kickSpeed(kicksp);
        // std::cout << robot->distTo(closestRobot.position()) << std::endl;
      }
      emit sendCommand(sslNavigation.run(*robot, cPass));
      break;
    }

    case 2: {
      // vai em direcao ao gol
      Point destiny = field->allyGoalOutsideCenter();
      Point nextP = nextPointToGo(destiny);
      // Point nextP = destiny;

      SSLMotion::GoToPoint goToGoal(nextP, (nextP - robot->position()).angle(), true);
      SSLRobotCommand cGoToGoal(goToGoal);
      cGoToGoal.set_dribbler(true);
      emit sendCommand(sslNavigation.run(*robot, cGoToGoal));

      break;
    }

    case 3: {
      // mais perto da bola vai em direcao a bola
      Point nextP;
      if (robot->distTo(frame->ball().position()) <= 500)
        nextP = frame->ball().position();
      else
        nextP = nextPointToGo(frame->ball().position());
      // nextP = frame->ball().position();

      SSLMotion::GoToPoint goToBall(nextP, (nextP - robot->position()).angle(), true);
      SSLRobotCommand cGoToBall(goToBall);
      emit sendCommand(sslNavigation.run(*robot, cGoToBall));
      break;
    }

    case 4: {
      // se tao longe vao em direcao a bola
      Point pontoEixoX(frame->ball().position().x(), robot->position().y());
      Point nextP = nextPointToGo(pontoEixoX);

      SSLMotion::GoToPoint goToBall(nextP, (nextP - robot->position()).angle(), true);
      if ((isDefender && robot->position().isOnTheRightOf(frame->ball().position())) ||
          (isStriker && robot->position().isOnTheLeftOf(frame->ball().position())))
        goToBall.set_maxVelocity(0.5);
      SSLRobotCommand cGoToBall(goToBall);
      emit sendCommand(sslNavigation.run(*robot, cGoToBall));
      break;
    }

    case 5: {
      // olha para a bola
      SSLMotion::RotateOnSelf lookToBall((frame->ball().position() - robot->position()).angle());
      SSLRobotCommand cLookToBall(lookToBall);
      emit sendCommand(sslNavigation.run(*robot, cLookToBall));
      break;
    }

    case 6: {
      // caso a bola esteja atras
      // Point pontoEixoX(frame->ball().position().x(), robot->position().y());
      // SSLMotion::GoToPoint goToBall(pontoEixoX, (pontoEixoX - robot->position()).angle(), true);
      // SSLRobotCommand cGoToBall(goToBall);
      // emit sendCommand(sslNavigation.run(*robot, cGoToBall));
      break;
    }

    case 7: {
      // goleiro ir pro gol
      SSLMotion::GoToPoint goalkeeperToGoal(
          field->enemyGoalOutsideCenter(),
          (field->enemyGoalOutsideCenter() - robot->position()).angle());
      SSLRobotCommand cGoalkeeperToGoal(goalkeeperToGoal);
      emit sendCommand(sslNavigation.run(*robot, cGoalkeeperToGoal));
      break;
    }

    case 8: {

      SSLMotion::GoToPoint goMid(field->center(), (field->center() - robot->position()).angle());
      SSLRobotCommand cGoMid(goMid);
      emit sendCommand(sslNavigation.run(*robot, cGoMid));
    }

    default: break;
  }
}

void CustomPlayer::receiveField(const Field& field) {
  shared->field = field;
}

void CustomPlayer::receiveFrame(const Frame& frame) {
  shared->frame = frame;
  runInParallel();
}

static_block {
  Factory::processing.insert<CustomPlayer>();
};