#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>
#include <vector>

#include "../../Fido/include/Simulator/Simlink.h"
#include "../../Fido/include/Fido.h"
#include "../../Fido/include/Adadelta.h"

// ---------------- HELPER FUNCTIONS -----------------
void printStats(std::vector<double> iterations) {
  double sum = 0;
  for(auto a = iterations.begin(); a != iterations.end(); a++) sum += *a;
  std::cout << "Average iter: " << (sum / double(iterations.size())) << "\n";

  std::sort(iterations.begin(), iterations.end(), [=](double a, double b) {
    return a < b;
  });
  std::cout << "Median iter: " << iterations[iterations.size() / 2] << "\n";
}

void printStats(std::vector<int> iterations) {
  int sum = 0;
  for(auto a = iterations.begin(); a != iterations.end(); a++) sum += *a;
  std::cout << "Average iter: " << (sum / double(iterations.size())) << "\n";

  std::sort(iterations.begin(), iterations.end(), [=](int a, int b) {
    return a < b;
  });
  std::cout << "Median iter: " << iterations[iterations.size() / 2] << "\n";
}

// ---------------- EXPERIMENTS -----------------

// Set LED value proportional to visible light
void flash() {
	std::vector<int> iterations;
	std::vector<double> choosingTimes, updateTimes;

	rl::FidoControlSystem learner = rl::FidoControlSystem(1, {0}, {1}, 11);
	for(int a = 0; a < 200; a++) {
		learner.reset();

		int iter = 0;
		std::vector<double> errors;
		double average = 0;
		for(iter = 0; iter < 1000 && (errors.size() < 3 || average > 0.4); iter++) {
			double input = (rand() % 100) / 100.0;

			clock_t begin = clock();
			double output = (double)learner.chooseBoltzmanActionDynamic({input})[0];
			choosingTimes.push_back((clock() - begin) / (double)CLOCKS_PER_SEC);

			std::cout << "input: " << input << "; output: " << output << "; REWARD: " << 1 - 2*fabs(output - input) << "\n";

			begin = clock();
			learner.applyReinforcementToLastAction(1 - 2*fabs(output - input), {output});
			updateTimes.push_back((clock() - begin) / (double)CLOCKS_PER_SEC);

			errors.push_back(fabs(output - input));
			if(errors.size() > 3) errors.erase(errors.begin());
			average = 0;
			for(double e : errors) average += e;
			average /= (double)errors.size();
		}

		iterations.push_back(iter);

		printStats(iterations);
		printStats(choosingTimes);
    printStats(updateTimes);
	}
}

// Continuous line following with the FidoControlSystem
// If the robot strays too far from the line, he is put back on the line.
void lineFollowHoloContinuous() {
  std::vector<double> choosingTimes, updateTimes;
  double maxDistanceComponent = 20;
  std::vector<int> iterations;

  Simlink simulator;
  rl::FidoControlSystem learner = rl::FidoControlSystem(1, {-1, -1}, {1, 1}, 3);
  for(int a = 0; a < 200; a++) {
    learner.reset();
    simulator.robot.setPosition(400, 400);

    int goodIter = 0;
    int iter = 0;

    while(goodIter < 10 && iter < 1000) {
      rl::Action action;

      double lineValue = simulator.distanceFromLine();
      clock_t begin = clock();
      action = learner.chooseBoltzmanActionDynamic({ !simulator.isLeftOfLine() ? -1.0 : 1.0});
      choosingTimes.push_back(double(clock() - begin) / CLOCKS_PER_SEC);
      simulator.robot.setPosition(simulator.robot.getPosition() + sf::Vector2f(action[0]*maxDistanceComponent, action[1]*maxDistanceComponent));

      double newLineValue = simulator.distanceFromLine();
      rl::State newState = { !simulator.isLeftOfLine() ? -1.0 : 1.0};

      begin = clock();
      learner.applyReinforcementToLastAction((fabs(lineValue) - fabs(newLineValue)) / sqrt(2*pow(maxDistanceComponent, 2)), newState);
      updateTimes.push_back(double(clock() - begin) / CLOCKS_PER_SEC);

      if(simulator.distanceFromLine() < 50) goodIter++;
      else {
        simulator.robot.setPosition(400, 400);
        goodIter = 0;
      }

      iter++;
    }

    iterations.push_back(iter);

    printStats(iterations);
    printStats(choosingTimes);
    printStats(updateTimes);
  }
}

// Continuous line following. One input to the net is completely random
void lineFollowHoloContinuousRand() {
  std::vector<double> choosingTimes, updateTimes;
  double maxDistanceComponent = 20;
  std::vector<int> iterations;

  Simlink simulator;
  rl::FidoControlSystem learner = rl::FidoControlSystem(2, {-1, -1}, {1, 1}, 3);
  learner.trainer = new net::Adadelta(0.95, 0.01, 10000);
  for(int a = 0; a < 1000; a++) {
    learner.reset();
    simulator.robot.setPosition(400, 400);

    int goodIter = 0;
    int iter = 0;

    while(goodIter < 10 && iter < 1000) {
      rl::Action action;

      double lineValue = simulator.distanceFromLine();
      clock_t begin = clock();
      action = learner.chooseBoltzmanActionDynamic({ !simulator.isLeftOfLine() ? -1.0 : 1.0, (double)rand() / (double)RAND_MAX});
      choosingTimes.push_back(double(clock() - begin) / CLOCKS_PER_SEC);
      simulator.robot.setPosition(simulator.robot.getPosition() + sf::Vector2f(action[0]*maxDistanceComponent, action[1]*maxDistanceComponent));

      double newLineValue = simulator.distanceFromLine();
      rl::State newState = { !simulator.isLeftOfLine() ? -1.0 : 1.0, (double)rand() / (double)RAND_MAX};
      begin = clock();
      learner.applyReinforcementToLastAction((fabs(lineValue) - fabs(newLineValue)) / sqrt(2*pow(maxDistanceComponent, 2)), newState);
      updateTimes.push_back(double(clock() - begin) / CLOCKS_PER_SEC);

      if(simulator.distanceFromLine() < 80) goodIter++;
      else {
        goodIter = 0;
        simulator.robot.setPosition(400, 400);
      }

      iter++;
    }

    iterations.push_back(iter);

    printStats(iterations);
    printStats(choosingTimes);
    printStats(updateTimes);
  }
}


// Line follow with kiwi drive. Uses the FidoControlSystem
void lineFollowHoloContinuousKiwi() {
  double maxMove = 20;
  double exploration = 0.2;
  std::vector<int> iterations;

  Simlink simulator;

  rl::WireFitQLearn learner = rl::WireFitQLearn(1, 3, 1, 6, 4, {-1, -1, -1}, {1, 1, 1}, 6, new rl::LSInterpolator(), new net::Backpropagation(0.01, 0.9, 0.1, 5000), 1, 0);
  learner.reset();

  std::cout << "Done with initialization\n";
  for(int a = 0; a < 200; a++) {
	  learner.reset();
	  simulator.robot.setPosition(400, 400);

	  int goodIter = 0;
	  int iter = 0;

	  while(goodIter < 20) {
	    rl::Action action;

	    action = learner.chooseBoltzmanAction({!simulator.isLeftOfLine() ? -1.0 : 1.0}, exploration);
	    double lineValue = simulator.distanceFromLine();
	    simulator.robot.inverseGoKiwi(action[0]*maxMove, action[1]*maxMove, action[2]*maxMove, 1);
	    double newLineValue = simulator.distanceFromLine();


	    learner.applyReinforcementToLastAction((fabs(lineValue) - fabs(newLineValue)) / 142.0, { !simulator.isLeftOfLine() ? -1.0 : 1.0});

	    if(simulator.distanceFromLine() < 80) goodIter++;
	    else goodIter = 0;

	    if(iter % 10 == 0) {
	      simulator.robot.setPosition(400, 400);
	    }

	    iter++;
	  }

    iterations.push_back(iter);

    printStats(iterations);
  }
}

void driveToPointDifferential(rl::FidoControlSystem *learner) {
  std::vector<int> iterations;

  Simlink simulator;
  for(int a = 0; a < 400; a++) {
    learner->reset();
    simulator.placeRobotInRandomPosition();
    simulator.placeEmitterInRandomPosition();

    int iter = 0;

    while(simulator.getDistanceOfRobotFromEmitter() > 100) {
      double x, y;

      for(int a = 0; a < 4; a++) {
        simulator.getRobotDisplacementFromEmitter(&x, &y);
        rl::Action action = learner->chooseBoltzmanAction({x / (fabs(x) + fabs(y)), y / (fabs(x) + fabs(y)), (double)simulator.robot.getRotation() / 360.0}, 0);
        simulator.robot.go(action[0] * 100, action[1] * 100, 3, 20);

        while(simulator.getDistanceOfRobotFromEmitter() > 400) {
          simulator.placeRobotInRandomPosition();
        }
      }

      simulator.getRobotDisplacementFromEmitter(&x, &y);
      rl::Action action = learner->chooseBoltzmanActionDynamic({x / (fabs(x) + fabs(y)), y / (fabs(x) + fabs(y)), (double)simulator.robot.getRotation() / 360.0});

      double previousDistance = simulator.getDistanceOfRobotFromEmitter();

      simulator.robot.go(action[0] * 100, action[1] * 100, 3, 20);
      simulator.getRobotDisplacementFromEmitter(&x, &y);

      simulator.robot.setRotation(-iter * 45);
      double atan2Output = (180.0 * atan2(y, -x) / 3.1415926);
      std::cout << "Rot: " << (360 - simulator.robot.getRotation() >= 180 ?  -simulator.robot.getRotation() : 360 - simulator.robot.getRotation()) << " " << (atan2Output) << "\n";

      std::this_thread::sleep_for(std::chrono::milliseconds(5000));

      //if(fabs((double)(previousDistance - simulator.getDistanceOfRobotFromEmitter()) / 30) > 1) std::cout << ((double)(previousDistance - simulator.getDistanceOfRobotFromEmitter()) / 30) << "\n";
      //learner->applyReinforcementToLastAction((double)(previousDistance - simulator.getDistanceOfRobotFromEmitter()) / 30, {x / (abs(x) + abs(y)), y / (abs(x) + abs(y)), (double)simulator.robot.getRotation() / 360.0});

      while(simulator.getDistanceOfRobotFromEmitter() > 400) {
        simulator.placeRobotInRandomPosition();
      }

      iter++;
    }

    iterations.push_back(iter);

    printStats(iterations);
  }
}

void driveToPointDiscrete() {
  net::NeuralNet *net = new net::NeuralNet(3, 2, 1, 1, "sigmoid");
  net->setOutputActivationFunction("simpleLinear");

  std::vector<std::vector<double>> possibleActions(0);
  int baseOfDimensions = 15;
  for(int a = 0; a < baseOfDimensions; a++) {
    for(int b = 0; b < baseOfDimensions; b++) {
      possibleActions.push_back({-1+(a*2/double(baseOfDimensions-1)), -1+(b*2/double(baseOfDimensions-1))});
    }
  }

  rl::QLearn learner = rl::QLearn(net, new net::Backpropagation(0.01, 0.9, 0.1, 35000), 0.95, 0.4, possibleActions);
  //driveToPointDifferential(&learner);
}

void driveToPointContinuous() {
  std::vector<int> iterations;

  rl::FidoControlSystem learner = rl::FidoControlSystem(3, {0, 0}, {1, 1}, 6);
  //rl::WireFitQLearn learner(3, 2, 1, 12, 5, {-1, -1}, {1, 1}, 6, new rl::LSInterpolator(), new net::Backpropagation(0.01, 0.9, 0.01, 5000), 1, 0);
  driveToPointDifferential(&learner);
}

void driveToPointHolo() {
  double maxDistanceComponent = 30;
  std::vector<double> choosingTimes, updateTimes;
  std::vector<int> iterations;

  Simlink simulator;

  rl::FidoControlSystem learner = rl::FidoControlSystem(2, {-1, -1}, {1, 1}, 6);
  //rl::WireFitQLearn learner(3, 2, 1, 12, 5, {-1, -1}, {1, 1}, 6, new rl::LSInterpolator(), new net::Backpropagation(0.01, 0.9, 0.1, 5000), 1, 0);

  for(int a = 0; a < 200; a++) {
    learner.reset();
    simulator.placeRobotInRandomPosition();
    simulator.placeEmitterInRandomPosition();

    int iter = 0;

    while(simulator.getDistanceOfRobotFromEmitter() > 100) {
      double x, y;
      simulator.getRobotDisplacementFromEmitter(&x, &y);

      clock_t begin = clock();
      rl::Action action = learner.chooseBoltzmanActionDynamic({x / (abs(x) + abs(y)), y / (abs(x) + abs(y))});
      choosingTimes.push_back((clock() - begin) / (double)CLOCKS_PER_SEC);

      double previousDistance = simulator.getDistanceOfRobotFromEmitter();

      simulator.robot.setPosition(simulator.robot.getPosition() + sf::Vector2f(action[0] * maxDistanceComponent, action[1] * maxDistanceComponent));

      simulator.getRobotDisplacementFromEmitter(&x, &y);

      begin = clock();
      learner.applyReinforcementToLastAction((double)(previousDistance - simulator.getDistanceOfRobotFromEmitter()) / sqrt(2*pow(maxDistanceComponent, 2)), {x / (abs(x) + abs(y)), y / (abs(x) + abs(y))});
      updateTimes.push_back((clock() - begin) / (double)CLOCKS_PER_SEC);

      if(simulator.getDistanceOfRobotFromEmitter() > 700) {
        simulator.placeRobotInRandomPosition();
      }

      iter++;
    }

    iterations.push_back(iter);

    printStats(iterations);
    printStats(choosingTimes);
    printStats(updateTimes);
  }
}

void goStraight() {
  double maxRotate = 5;
  std::vector<int> iterations;

  Simlink simulator;
  rl::FidoControlSystem learner = rl::FidoControlSystem(1, {-1}, {1}, 11);
  //rl::WireFitQLearn learner = rl::WireFitQLearn(1, 1, 1, 3, 4, {-1}, {1}, 11, new rl::LSInterpolator(), new net::Backpropagation(0.01, 0.9, 0.1, 35000), 0.95, 0.4);
  for(int a = 0; a < 50; a++) {
    learner.reset();
    simulator.placeRobotInRandomPosition();

    int iter = 0;

    std::vector<double> rotations(0);
    double average = 0;

    while((rotations.size() < 5 || average > 0.2) && iter < 40) {
      rl::Action action;

      action = learner.chooseBoltzmanAction({ 1 }, 0.4);
      //action = learner.chooseBoltzmanAction({ 1 }, exploration);
      simulator.robot.rotate(action[0]*maxRotate);
      //simulator.robot.go(10, 10, 5, 5);

      learner.applyReinforcementToLastAction(1-fabs(action[0]*2), {1});

      iter++;

      rotations.push_back(action[0]);
      if(rotations.size() > 5) {
        rotations.erase(rotations.begin());
      }

      average = 0;
      for(unsigned int b = 0; b < rotations.size(); b++) {
        average += fabs(rotations[b]);
      }
      average /= (double)rotations.size();
    }

    iterations.push_back(iter);

    printStats(iterations);
  }
}

void simulatorTest() {
  Simlink simulator;
  while(true) {
    simulator.setMotors(10, 5, 10, 10);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void changingAction() {
  std::vector<int> iterations;
  std::vector<double> rewards;

  rl::FidoControlSystem learner = rl::FidoControlSystem(1, {-1}, {1}, 6);
  //learner.trainer = new net::Pruner(0.01, 0.95, 0.005, 20000, 600);
  //learner.trainer = new net::Adadelta(0.95, 0.01, 10000);
  for(int a = 0; a < 100; a++) {
    learner.reset();

    int iter = 0;
    while(iter < 40) {
      rl::Action action = learner.chooseBoltzmanActionDynamic({1});

      if(iter == 20) std::cout << "---------CHANGE-----------\n";

      double reward;
      if(iter < 20) {
        reward = 1-fabs(action[0]);
      } else {
        reward = 1-fabs(action[0]);
      }
      learner.applyReinforcementToLastAction(reward, {1});
      rewards.push_back(reward);

      std::cout << "-----------Reward-----------\n";
      printStats(rewards);
      std::cout << "----------------------\n";

      iter++;
    }

    iterations.push_back(iter);
    printStats(iterations);

    std::cout << "-----------Reward-----------\n";
    printStats(rewards);
    std::cout << "----------------------\n";
  }
}

int main() {
  srand(time(NULL));
  driveToPointHolo();
}
