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

      rrt->obstacles->addObstacle(r.position(), r.position());
    }

    for (Robot r : frame->enemies())
      rrt->obstacles->addObstacle(r.position(), r.position());

    rrt->setMaxIterations(2500);
    rrt->setStepSize(120);
    robotpath[id].pathNodes = rrt->runRRTSTAR();

    robotpath[id].currentNode = (int) robotpath[id].pathNodes.size() - 1;
    robotpath[id].nextPoint = robotpath[id].pathNodes.at(robotpath[id].currentNode);
    robotpath[id].target = robotpath[id].pathNodes.at(0);
    delete rrt;
  } else if (robotpath[id].currentNode > 0 && robot->distTo(robotpath[id].nextPoint) <= 120) {

    robotpath[id].currentNode--;
    robotpath[id].nextPoint = robotpath[id].pathNodes.at(robotpath[id].currentNode);
  }

  return robotpath[id].nextPoint;
}

void CustomPlayer::exec() {
  if (!field || !frame || !robot || !frame->has_ball() || !frame.has_value()) {
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

  Robot closestToBall = *frame->allies().removedById(5).closestTo(frame->ball().position());
  Robot closestAlly = *frame->allies().removedById(robot->id()).closestTo(robot->position());
  bool isClosestToBall = robot->id() == closestToBall.id();
  bool haveBall = closestToBall.distTo(frame->ball().position()) <= 110;
  bool closeToGoal = closestToBall.distTo(field->allyGoalInsideCenter()) <= 3000;
  bool isStriker = std::find(striker.begin(), striker.end(), robot->id()) != striker.end();
  bool isDefender = std::find(defense.begin(), defense.end(), robot->id()) != defense.end();
  bool ballInGoalkeeperArea = field->allyPenaltyAreaContains(frame->ball().position()) ||
                              field->enemyPenaltyAreaContains(frame->ball().position());
  bool ballWithOtherTeam = false;

  Robot enemyClosestToBall = robot.value();

  for (Robot r : frame->enemies())
    if (r.distTo(frame->ball().position()) <= 120.0) {
      enemyClosestToBall = r;
      ballWithOtherTeam = true;
      break;
    }

  int state = 4;
  if (!isGoalkeeper) {

    if (isClosestToBall) {
      if (haveBall) {
        if (closeToGoal)
          state = 0; // chuta para o gol
        else if (isStriker)
          state = 2; // vai para o gol
        else
          state = 1; // toca para um atacante
      } else if (ballWithOtherTeam)
        state = 6; // rouba bola
      else
        state = 3; // vai para a bola
    } else
      state = 4; // anda pelo campo
  } else
    state = 5; // goleiro

  if (!field->contains(robot->position()) || field->allyPenaltyAreaContains(robot->position()) ||
      (!isGoalkeeper && field->enemyPenaltyAreaContains(robot->position())))
    state = 7; // fora do campo ou na area do goleiro entao vai pro meio

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

      emit sendCommand(sslNavigation.run(robot.value(), cKickGoal));
      break;
    }

    case 1: {
      // passa para o aliado mais perto
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
      }
      emit sendCommand(sslNavigation.run(robot.value(), cPass));
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
      emit sendCommand(sslNavigation.run(robot.value(), cGoToGoal));

      break;
    }

    case 3: {
      // mais perto da bola vai em direcao a bola
      Point nextP;
      // if (robot->distTo(frame->ball().position()) <= 500)
      //   nextP = frame->ball().position();
      // else
      //   nextP = nextPointToGo(frame->ball().position());
      nextP = frame->ball().position();

      SSLMotion::GoToPoint goToBall(nextP, (nextP - robot->position()).angle(), true);
      if (ballInGoalkeeperArea)
        goToBall.set_maxVelocity(0.3);
      SSLRobotCommand cGoToBall(goToBall);
      emit sendCommand(sslNavigation.run(robot.value(), cGoToBall));
      break;
    }

    case 4: {
      // anda pelo campo
      Point pontoEixoX(frame->ball().position().x(), robot->position().y());
      // if (pontoEixoX.x() - robot->position().x() > 1000)
      //   pontoEixoX.setX(robot->position().x() + 1000);
      // else if (pontoEixoX.x() - robot->position().x() < -1000)
      //   pontoEixoX.setX(robot->position().x() - 1000);
      // Point nextP = nextPointToGo(pontoEixoX);
      Point nextP = pontoEixoX;

      SSLMotion::GoToPoint goToBall(nextP, (nextP - robot->position()).angle(), true);
      if ((isDefender && robot->position().isOnTheRightOf(frame->ball().position())) ||
          (isStriker && robot->position().isOnTheLeftOf(frame->ball().position())))
        goToBall.set_maxVelocity(0.5);
      if (ballInGoalkeeperArea)
        goToBall.set_maxVelocity(0.3);
      SSLRobotCommand cGoToBall(goToBall);
      emit sendCommand(sslNavigation.run(robot.value(), cGoToBall));
      break;
    }

    case 5: {
      // goleiro
      int goalkeeperState = 0;
      bool goalkeeperInGoal = field->enemyPenaltyAreaContains(robot->position());

      if (!goalkeeperInGoal)
        goalkeeperState = 3;
      else if (!field->enemyPenaltyAreaContains(frame->ball().position()))
        goalkeeperState = 0;
      else if (haveBall)
        goalkeeperState = 1;
      else
        goalkeeperState = 2;

      switch (goalkeeperState) {

        case 0: {

          Point nextP(4300, frame->ball().position().y());

          if (nextP.y() > 450)
            nextP.setY(450);
          else if (nextP.y() < -450)
            nextP.setY(-450);

          SSLMotion::GoToPoint goToBall(nextP, (nextP - robot->position()).angle(), false);
          goToBall.set_maxVelocity(1);
          SSLRobotCommand cGoToBall(goToBall);
          emit sendCommand(sslNavigation.run(robot.value(), cGoToBall));
          break;
        }

        case 1: {

          // caso a bola esteja dentro da area, toca para aliado mais perto
          Robot closest = *frame->allies().findById(0);
          int dist = 10000;
          for (Robot i : frame->allies().removedById(5)) {

            if (closest.distTo(i) > dist) {

              dist = closest.distTo(i);
              closest = i;
            }
          }

          SSLMotion::RotateOnSelf pass((closest.position() - robot->position()).angle());
          SSLRobotCommand cPass(pass);
          cPass.set_dribbler(true);
          const double tenDegrees = 0.17;
          if (abs(robot->angleTo(closest.position())) <= tenDegrees) {

            cPass.set_front(true);
            double kicksp = 1 + robot->distTo(closest.position()) / 1500;
            cPass.set_kickSpeed(kicksp);
            // std::cout << robot->distTo(closestRobot.position()) << std::endl;
          }
          emit sendCommand(sslNavigation.run(robot.value(), cPass));
          break;
        }

        case 2: {

          // vai em direção a bola
          SSLMotion::GoToPoint goBall(frame->ball().position(),
                                      (frame->ball().position() - robot->position()).angle(),
                                      true);
          SSLRobotCommand cGoBall(goBall);
          emit sendCommand(sslNavigation.run(robot.value(), cGoBall));
          break;
        }

        case 3: {
          // goleiro ir pro gol
          SSLMotion::GoToPoint goalkeeperToGoal(
              field->enemyGoalOutsideCenter(),
              (field->enemyGoalOutsideCenter() - robot->position()).angle());
          SSLRobotCommand cGoalkeeperToGoal(goalkeeperToGoal);
          emit sendCommand(sslNavigation.run(robot.value(), cGoalkeeperToGoal));
          break;
        }

        default: break;
      }

      break;
    }

    case 6: {

      int stateStealBall = 0;

      double distXtoBall = enemyClosestToBall.position().x() - frame->ball().position().x();
      double distYtoBall = enemyClosestToBall.position().y() - frame->ball().position().y();

      bool enemyInRight = distXtoBall > 0;
      bool enemyInUp = distYtoBall > 0;
      bool allyInRight = (robot->position().x() - frame->ball().position().x()) > 0;
      bool allyInUp = (robot->position().y() - frame->ball().position().y()) > 0;

      bool canGoBall = ((enemyInUp && !allyInUp) || (!enemyInUp && allyInUp)) &&
                       ((enemyInRight && !allyInRight) || (!enemyInRight && allyInRight));

      int offsetX = 500, offsetY = 500;

      if (enemyInRight)
        offsetX *= -1;
      if (enemyInUp)
        offsetY *= -1;

      double pointXtoGo = frame->ball().position().x() + offsetX;
      double pointYtoGo = frame->ball().position().y() + offsetY;
      Point pointToGo(pointXtoGo, pointYtoGo);

      if (canGoBall) {
        stateStealBall = 1;
      } else
        stateStealBall = 0;

      cout << stateStealBall << endl;

      switch (stateStealBall) {

        case 0: {

          Point nextP = nextPointToGo(pointToGo);

          SSLMotion::GoToPoint goToGoal(nextP, (nextP - robot->position()).angle(), true);
          SSLRobotCommand cGoToGoal(goToGoal);
          emit sendCommand(sslNavigation.run(robot.value(), cGoToGoal));
          break;
        }

        case 1: {

          Point nextP = frame->ball().position();
          SSLMotion::GoToPoint goToGoal(nextP, (nextP - robot->position()).angle(), true);
          if (robot->distTo(frame->ball().position()) <= 200)
            goToGoal.set_maxVelocity(0.9);
          SSLRobotCommand cGoToGoal(goToGoal);
          cGoToGoal.set_dribbler(true);
          emit sendCommand(sslNavigation.run(robot.value(), cGoToGoal));
          break;
        }

        default: break;
      }

      break;
    }

    case 7: {
      SSLMotion::GoToPoint goMid(field->center(), (field->center() - robot->position()).angle());
      SSLRobotCommand cGoMid(goMid);
      emit sendCommand(sslNavigation.run(robot.value(), cGoMid));
      break;
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