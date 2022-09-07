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

  // roda o algoritmo de path-planning enquanto nao achou um caminho até o destino
  if (destiny.distTo(robotpath[id].getLastPoint()) > END_DIST_THRESHOLD) {

    RRTSTAR* rrt = new RRTSTAR;
    rrt->setInitPos(robot->position());
    rrt->setEndPos(destiny);
    rrt->setMaxIterations(2500);
    rrt->setStepSize(120);
    rrt->initialize();

    // adiciona obstaculos
    for (Robot r : frame->allies()) {

      if (r.id() == robot->id())
        continue;

      rrt->obstacles->addObstacle(r.position(), r.position());
    }

    for (Robot r : frame->enemies())
      rrt->obstacles->addObstacle(r.position(), r.position());

    // roda o algoritmo e salva o vetor com os nós até o caminho
    robotpath[id].setPathNodes(rrt->runRRTSTAR());

    robotpath[id].setCurrentNode(robotpath[id].getPathNodes().size() - 1);
    robotpath[id].setNextPoint(robotpath[id].getPathNodes().at(robotpath[id].getCurrentNode()));
    robotpath[id].setLastPoint(robotpath[id].getPathNodes().at(0));

    delete rrt;

  }
  // caso ja tenha achado um caminho, usa o vetor com os nós até tal caminho
  else if (robotpath[id].getCurrentNode() > 0 &&
           robot->distTo(robotpath[id].getNextPoint()) < 120) {

    robotpath[id].setCurrentNode(robotpath[id].getCurrentNode() - 1);
    robotpath[id].setNextPoint(robotpath[id].getPathNodes().at(robotpath[id].getCurrentNode()));
  }

  return robotpath[id].getNextPoint();
}

void CustomPlayer::exec() {
  if (!field || !frame || !robot || !frame->has_ball() || !frame.has_value()) {
    return;
  }

  // TODO: here...
  // emit sendCommand(..

  // goleiro
  const int idGoalkeeper = 5;
  const bool isGoalkeeper = robot->id() == idGoalkeeper;

  // variaveis e constantes auxiliares para o behavior
  const Robot closestToBall =
      *frame->allies().removedById(idGoalkeeper).closestTo(frame->ball().position());
  const Robot closestAlly = *frame->allies().removedById(robot->id()).closestTo(robot->position());
  Robot robotToPass = *frame->allies()
                           .removedById(idGoalkeeper)
                           .removedById(robot->id())
                           .closestTo(robot->position());

  const bool isClosestToBall = robot->id() == closestToBall.id();
  const bool haveBall = robot->distTo(frame->ball().position()) <= 110;
  const bool closeToGoal = robot->distTo(field->allyGoalInsideCenter()) <= 3000;
  const bool isStriker = std::find(striker.begin(), striker.end(), robot->id()) != striker.end();
  const bool isDefender = std::find(defense.begin(), defense.end(), robot->id()) != defense.end();
  const bool ballInGoalkeeperArea = field->allyPenaltyAreaContains(frame->ball().position()) ||
                                    field->enemyPenaltyAreaContains(frame->ball().position());
  bool ballWithOtherTeam = false;

  Robot enemyClosestToBall = robot.value();

  const double fiveDegrees = 0.087;

  for (Robot r : frame->enemies())
    if (r.distTo(frame->ball().position()) <= 120.0) {
      enemyClosestToBall = r;
      ballWithOtherTeam = true;
      break;
    }

  int state;
  if (!isGoalkeeper) {

    if (isClosestToBall) {
      if (haveBall) {
        if (closeToGoal)
          state = 0; // chuta para o gol
        else if (isStriker)
          state = 2; // move-se em direção ao gol
        else
          state = 1; // toca a bola para outro robô
      } else if (ballWithOtherTeam)
        state = 6; // rouba bola do jogador adversário
      else
        state = 3; // vai em direção a bola
    } else
      state = 4; // movimenta-se pelo campo
  } else
    state = 5; // goleiro

  if (!field->contains(robot->position()) || field->allyPenaltyAreaContains(robot->position()) ||
      (!isGoalkeeper && field->enemyPenaltyAreaContains(robot->position())))
    state = 7; // caso a bola esteja fora do campo ou na área do goleiro

  // verifica se é possível tocar a bola para robô, caso não, vai em direção ao gol
  if (state == 1) {

    bool canPass = false;

    for (int i : allPlayers) {

      if (robot->id() == i)
        continue;

      Robot r = *frame->allies().findById(i);

      int distLine = INT_MAX;
      for (Robot enemyRobot : frame->enemies()) {

        int aux =
            Geometry2D::distancePointLine(robot->position(), r.position(), enemyRobot.position());
        if (aux < distLine)
          distLine = aux;
      }
      // caso não haja robôs na linha de passe, pode tocar
      if (distLine > 100) {

        robotToPass = r;
        canPass = true;
        break;
      }
    }

    if (!canPass)
      state = 2;
  }

  if (state != 2 && state != 6 && state != 4 &&
      robotpath[robot->id()].getLastPoint() != Point(3000, -3000))
    robotpath[robot->id()].setLastPoint(Point(3000, -3000));

  switch (state) {

    // chuta para o gol
    case 0: {

      // rotaciona em direção ao gol
      SSLMotion::RotateOnSelf kickGoal((field->allyGoalInsideCenter() - robot->position()).angle());
      SSLRobotCommand cKickGoal(kickGoal);
      cKickGoal.set_dribbler(true);

      // caso esteja em ângulo de chute ao gol, chuta
      if (abs(robot->angleTo(field->allyGoalInsideCenter())) <= fiveDegrees) {

        cKickGoal.set_front(true);
        cKickGoal.set_kickSpeed(6);
      }

      emit sendCommand(sslNavigation.run(robot.value(), cKickGoal));
      break;
    }

    // toca a bola
    case 1: {

      // rotaciona em direção ao robô para quem vai tocar
      SSLMotion::RotateOnSelf pass((robotToPass.position() - robot->position()).angle());
      SSLRobotCommand cPass(pass);
      cPass.set_dribbler(true);

      // caso esteja em ângulo de passe, toca
      if (abs(robot->angleTo(robotToPass.position())) <= fiveDegrees) {

        cPass.set_front(true);
        double kicksp = 1 + robot->distTo(robotToPass.position()) / 1500;
        cPass.set_kickSpeed(kicksp);
      }
      emit sendCommand(sslNavigation.run(robot.value(), cPass));
      break;
    }

    case 2: {
      // vai em direcao ao gol inimigo, evitando colisões
      Point destiny = field->allyGoalOutsideCenter();
      Point nextP = nextPointToGo(destiny);

      SSLMotion::GoToPoint goToGoal(nextP, (nextP - robot->position()).angle(), true);
      SSLRobotCommand cGoToGoal(goToGoal);
      cGoToGoal.set_dribbler(true);
      emit sendCommand(sslNavigation.run(robot.value(), cGoToGoal));

      break;
    }

    // vai em direção a bola
    case 3: {
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

    // anda pelo campo, seguindo a posição X da bola
    case 4: {
      Point pontoEixoX(frame->ball().position().x(), robot->position().y());
      // if (pontoEixoX.x() - robot->position().x() > 1000)
      //   pontoEixoX.setX(robot->position().x() + 1000);
      // else if (pontoEixoX.x() - robot->position().x() < -1000)
      //   pontoEixoX.setX(robot->position().x() - 1000);
      // Point nextP = nextPointToGo(pontoEixoX);
      Point nextP = pontoEixoX;

      SSLMotion::GoToPoint goToBall(nextP, (nextP - robot->position()).angle(), true);

      // caso seja um atacante e a bola esteja atras dele ou caso seja um defensor e a bolsa esteja
      // na frente, diminui a velocidade
      if ((isDefender && robot->position().isOnTheRightOf(frame->ball().position())) ||
          (isStriker && robot->position().isOnTheLeftOf(frame->ball().position())))
        goToBall.set_maxVelocity(0.5);
      if (ballInGoalkeeperArea)
        goToBall.set_maxVelocity(0.3);
      SSLRobotCommand cGoToBall(goToBall);
      emit sendCommand(sslNavigation.run(robot.value(), cGoToBall));
      break;
    }

    // goleiro
    case 5: {

      int goalkeeperState = 0;
      bool goalkeeperInGoal = field->enemyPenaltyAreaContains(robot->position());

      if (!goalkeeperInGoal)
        goalkeeperState = 3; // goleiro fora da área dele, então volta para o gol
      else if (!field->enemyPenaltyAreaContains(frame->ball().position()))
        goalkeeperState = 0; // segue a posição Y da bola
      else if (robot->distTo(frame->ball().position()) <= 110)
        goalkeeperState = 1; // toca para robô aliado mais próximo
      else
        goalkeeperState = 2; // vai em direção a bola

      switch (goalkeeperState) {

        // segue a posição Y da bola
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

        // caso a bola esteja dentro da area, toca para aliado mais perto
        case 1: {
          Robot closest = *frame->allies().removedById(robot->id()).closestTo(robot->position());

          SSLMotion::RotateOnSelf pass((closest.position() - robot->position()).angle());
          SSLRobotCommand cPass(pass);
          cPass.set_dribbler(true);
          if (abs(robot->angleTo(closest.position())) <= fiveDegrees) {

            cPass.set_front(true);
            double kicksp = 1 + robot->distTo(closest.position()) / 1500;
            cPass.set_kickSpeed(kicksp);
          }
          emit sendCommand(sslNavigation.run(robot.value(), cPass));
          break;
        }

        // vai em direção a bola
        case 2: {

          SSLMotion::GoToPoint goBall(frame->ball().position(),
                                      (frame->ball().position() - robot->position()).angle(),
                                      true);
          SSLRobotCommand cGoBall(goBall);
          emit sendCommand(sslNavigation.run(robot.value(), cGoBall));
          break;
        }

        // vai para a área do goleiro caso esteja fora
        case 3: {
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

    // rouba a bola
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

      // calcula ponto para ir, sendo oposto em relação ao que o robô inimigo está para a bola
      double pointXtoGo = frame->ball().position().x() + offsetX;
      double pointYtoGo = frame->ball().position().y() + offsetY;
      Point pointToGo(pointXtoGo, pointYtoGo);

      if (canGoBall) {
        stateStealBall = 1; // está em condição de roubar a bola sem colidir com o robô inimigo
      } else
        stateStealBall = 0; // vai para uma posição que consiga roubar a bola sem colidir

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