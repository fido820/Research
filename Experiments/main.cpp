#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>
#include <vector>

#include "../../Fido/include/Simulator/Simlink.h"
#include "../../Fido/include/Fido.h"
#include "../../Fido/include/Adadelta.h"

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

void lineFollowDrive() {
	Simlink simulator;
	rl::WireFitQLearn learner = rl::WireFitQLearn(2, 2, 3, 10, 5, {-1, -1}, {1, 1}, 6, new rl::LSInterpolator(), new net::Backpropagation(), 1, 0.4);
	for(int a = 0; a < 10000; a++) {
		double exploration = 0.8;
		learner.reset();
		simulator.placeRobotInRandomPosition();
		int goodIter = 0;
		while(goodIter < 100) {
			exploration *= 0.99;
			std::cout << "exploration: " << exploration << "\n";

			rl::Action action;
			double lineValue = simulator.distanceFromLine();
			action = learner.chooseBoltzmanAction({ (double)simulator.isLeftOfLine(), simulator.robot.getRotation() / 360.0 }, exploration);
			simulator.setMotors(action[0]*100, action[1]*100, 2, 20);

			double newLineValue = simulator.distanceFromLine();
			rl::State newState = { (double)simulator.isLeftOfLine(), simulator.robot.getRotation() / 360.0 };
			learner.applyReinforcementToLastAction((fabs(lineValue) - fabs(newLineValue)) / 20.0, newState);
			std::this_thread::sleep_for(std::chrono::milliseconds(10));

			if(simulator.distanceFromLine() < 100) goodIter++;
			else goodIter = 0;
		}
	}
}

void lineFollowHoloContinuous() {
	std::vector<double> choosingTimes, updateTimes;
	double maxDistanceComponent = 20;
	double exploration = 0.1;
	std::vector<int> iterations;

	Simlink simulator;
	rl::FidoControlSystem learner = rl::FidoControlSystem(1, {-1, -1}, {1, 1}, 3);
	learner.trainer = new net::Adadelta(0.95, 0.015, 10000);
	for(int a = 0; a < 2000; a++) {
		learner.reset();
		simulator.robot.setPosition(400, 400);

		int goodIter = 0;
		int iter = 0;

		while(goodIter < 10 && iter < 1000) {
			rl::Action action;

			double lineValue = simulator.distanceFromLine();
			clock_t begin = clock();
			action = learner.chooseBoltzmanAction({ !simulator.isLeftOfLine() ? -1 : 1}, exploration);
			choosingTimes.push_back(double(clock() - begin) / CLOCKS_PER_SEC);
			simulator.robot.setPosition(simulator.robot.getPosition() + sf::Vector2f(action[0]*maxDistanceComponent, action[1]*maxDistanceComponent));

			double newLineValue = simulator.distanceFromLine();
			rl::State newState = { !simulator.isLeftOfLine() ? -1 : 1};

			begin = clock();
			learner.applyReinforcementToLastAction((fabs(lineValue) - fabs(newLineValue)) / sqrt(2*pow(maxDistanceComponent, 2)), newState);
			updateTimes.push_back(double(clock() - begin) / CLOCKS_PER_SEC);

			//std::this_thread::sleep_for(std::chrono::milliseconds(100));

			if(simulator.distanceFromLine() < 50) goodIter++;
			else {
				simulator.robot.setPosition(400, 400);
				goodIter = 0;
			}

			iter++;
		}

		iterations.push_back(iter);

		printStats(iterations);
	}

	printStats(choosingTimes);
	printStats(updateTimes);
}

// Give noisy input
void lineFollowHoloContinuousRand() {
	std::vector<double> choosingTimes, updateTimes;
	double maxDistanceComponent = 20;
	double exploration = 0.2;
	std::vector<int> iterations;

	Simlink simulator;
	rl::FidoControlSystem learner = rl::FidoControlSystem(2, {-1, -1}, {1, 1}, 3);
	//learner.trainer = new net::Backpropagation(0.01, 0.2, 0.05, 50000);
	//learner.trainer = new net::Backpropagation(0.01, 0.9, 0.01, 5000);
	learner.trainer = new net::Adadelta(0.95, 0.01, 10000);
	for(int a = 0; a < 1000; a++) {
		learner.reset();
		simulator.robot.setPosition(400, 400);

		int goodIter = 0;
		int iter = 0;

		while(goodIter < 10 && iter < 1000) {
			rl::Action action;
			for(int a = 0; a < 2; a++) {
				action = learner.chooseBoltzmanAction({ !simulator.isLeftOfLine() ? -1 : 1, (double)rand() / (double)RAND_MAX}, exploration);
				simulator.robot.setPosition(simulator.robot.getPosition() + sf::Vector2f(action[0]*maxDistanceComponent, action[1]*maxDistanceComponent));
			}

			double lineValue = simulator.distanceFromLine();
			clock_t begin = clock();
			action = learner.chooseBoltzmanAction({ !simulator.isLeftOfLine() ? -1 : 1, (double)rand() / (double)RAND_MAX}, exploration);
			choosingTimes.push_back(double(clock() - begin) / CLOCKS_PER_SEC);
			simulator.robot.setPosition(simulator.robot.getPosition() + sf::Vector2f(action[0]*maxDistanceComponent, action[1]*maxDistanceComponent));

			double newLineValue = simulator.distanceFromLine();
			rl::State newState = { !simulator.isLeftOfLine() ? -1 : 1, (double)rand() / (double)RAND_MAX};
			begin = clock();
			learner.applyReinforcementToLastAction((fabs(lineValue) - fabs(newLineValue)) / sqrt(2*pow(maxDistanceComponent, 2)), newState);
			updateTimes.push_back(double(clock() - begin) / CLOCKS_PER_SEC);

			//std::this_thread::sleep_for(std::chrono::milliseconds(100));

			if(simulator.distanceFromLine() < 80) goodIter++;
			else {
				goodIter = 0;
				simulator.robot.setPosition(400, 400);
			}

			iter++;
		}

		iterations.push_back(iter);

		printStats(iterations);
	}

	printStats(choosingTimes);
	printStats(updateTimes);
}

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
			/*for(int a = 0; a < 2; a++) {
				action = learner.chooseBoltzmanAction({ !simulator.isLeftOfLine() ? -1 : 1, simulator.robot.getRotation() / 360.0}, exploration);
				simulator.robot.inverseGoKiwi(action[0]*maxMove, action[1]*maxMove, action[2]*maxMove, 1);
			}*/

			action = learner.chooseBoltzmanAction({!simulator.isLeftOfLine() ? -1 : 1}, exploration);
        	double lineValue = simulator.distanceFromLine();
        	simulator.robot.inverseGoKiwi(action[0]*maxMove, action[1]*maxMove, action[2]*maxMove, 1);
        	double newLineValue = simulator.distanceFromLine();

        	//if((fabs(lineValue) - fabs(newLineValue)) / 142.0 > 1) std::cout << (fabs(lineValue) - fabs(newLineValue)) << "\n";

	        learner.applyReinforcementToLastAction((fabs(lineValue) - fabs(newLineValue)) / 142.0, { !simulator.isLeftOfLine() ? -1 : 1});

			//std::this_thread::sleep_for(std::chrono::milliseconds(100));

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

void driveToPointDiscrete() {
	double maxDistanceComponent = 20;
	double exploration = 0.2;
	std::vector<int> iterations;

	Simlink simulator;

	net::NeuralNet *net = new net::NeuralNet(3, 2, 1, 1, "sigmoid");
	net->setOutputActivationFunction("simpleLinear");

	std::vector<std::vector<double>> possibleActions(0);
	int baseOfDimensions = 15;
	for(int a = 0; a < baseOfDimensions; a++) {
		for(int b = 0; b < baseOfDimensions; b++) {
			possibleActions.push_back({-1+(a*2/double(baseOfDimensions-1)), -1+(b*2/double(baseOfDimensions-1))});
		}
	}

	rl::QLearn learner = rl::QLearn(net, net::Backpropagation(0.01, 0.9, 0.1, 35000), 0.95, 0.4, possibleActions);
	for(int a = 0; a < 400; a++) {
		learner.reset();
		simulator.placeRobotInRandomPosition();
		simulator.placeEmitterInRandomPosition();

		int goodIter = 0;
		int iter = 0;

		while(simulator.getDistanceOfRobotFromEmitter() > 80) {
			double x, y;
			simulator.getRobotDisplacementFromEmitter(&x, &y);
			rl::Action action = learner.chooseBoltzmanAction({x / (abs(x) + abs(y)), y / (abs(x) + abs(y)), (double)simulator.robot.getRotation() / 360.0}, 0.2);

			sf::Vector2f previousRobotPosition = simulator.robot.getPosition();
			double previousDistance = simulator.getDistanceOfRobotFromEmitter();

			simulator.robot.go(action[0] * 100, action[1] * 100, 3, 20);

			simulator.getRobotDisplacementFromEmitter(&x, &y);
			learner.applyReinforcementToLastAction((double)(previousDistance - simulator.getDistanceOfRobotFromEmitter()) / 1.415, {x / (abs(x) + abs(y)), y / (abs(x) + abs(y)), (double)simulator.robot.getRotation() / 360.0});

			iter++;
		}

		iterations.push_back(iter);

		printStats(iterations);
	}
}

void goStraight() {
	double maxRotate = 5;
	double exploration = 0.2;
	std::vector<int> iterations;

	Simlink simulator;
	rl::WireFitQLearn learner = rl::WireFitQLearn(1, 1, 1, 3, 4, {-1}, {1}, 11, new rl::LSInterpolator(), new net::Backpropagation(0.01, 0.9, 0.1, 35000), 0.95, 0.4);
	for(int a = 0; a < 200; a++) {
		learner.reset();
		simulator.placeRobotInRandomPosition();

		int iter = 0;

		std::vector<double> rotations(0);
		double average = 0;

		while(rotations.size() < 5 || average > 0.2) {
			rl::Action action;

			action = learner.chooseBoltzmanAction({ 1 }, exploration);
			simulator.robot.rotate(action[0]*maxRotate);
			//simulator.robot.go(10, 10, 5, 5);

			learner.applyReinforcementToLastAction(1-fabs(action[0]*2), {1});

			iter++;

			rotations.push_back(action[0]);
			if(rotations.size() > 5) {
				rotations.erase(rotations.begin());
			}

			average = 0;
			for(int b = 0; b < rotations.size(); b++) {
				average += fabs(rotations[b]);
			}
			average /= (double)rotations.size();
		}

		iterations.push_back(iter);

		printStats(iterations);
	}
}

void changingAction() {
	double maxDistanceComponent = 40;
	double exploration = 0.3;
	std::vector<int> iterations;

	Simlink simulator;
	rl::FidoControlSystem learner = rl::FidoControlSystem(2, {-1, -1}, {1, 1}, 3);
	learner.trainer = new net::Adadelta(0.95, 0.01, 10000);
	for(int a = 0; a < 500; a++) {
		learner.reset();
		simulator.emitter.set(sf::Vector2i(600, 600));
		simulator.robot.setPosition(400, 400);

		int iter = 0;
		while(simulator.getDistanceOfRobotFromEmitter() > 50 && iter < 1000) {
			double x, y;
			simulator.getRobotDisplacementFromEmitter(&x, &y);
			rl::Action action = learner.chooseBoltzmanAction({x / (abs(x) + abs(y)), y / (abs(x) + abs(y))}, exploration);

			double previousDistance = simulator.getDistanceOfRobotFromEmitter();

			simulator.robot.setPosition(simulator.robot.getPosition()+sf::Vector2f(maxDistanceComponent*action[0], maxDistanceComponent*action[1]));

			simulator.getRobotDisplacementFromEmitter(&x, &y);

			std::vector< std::vector< std::vector<double> > > weights = learner.network->getWeights3D();
			learner.applyReinforcementToLastAction((double)(previousDistance - simulator.getDistanceOfRobotFromEmitter()) / sqrt(2*pow(maxDistanceComponent, 2)), {x / (abs(x) + abs(y)), y / (abs(x) + abs(y))});
			std::vector< std::vector< std::vector<double> > > newWeights = learner.network->getWeights3D();

			//std::cout << "Uncert: " << ((net::Adadelta *)learner.trainer)->averageChangeInWeight << "\n";
			if(simulator.getDistanceOfRobotFromEmitter() > sqrt(2*pow(220, 2))) {
				simulator.robot.setPosition(400, 400);
			}

			iter++;
		}

		iterations.push_back(iter);

		printStats(iterations);
	}
}

int main() {
	srand(time(NULL));
	changingAction();
}
