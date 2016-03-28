#include <iostream>
#include <chrono>
#include <thread>
#include <algorithm>
#include <vector>

#include "../../Fido/include/WireFitQLearn.h"
#include "../../Fido/include/QLearn.h"
#include "../../Fido/include/NeuralNet.h"
#include "../../Fido/include/Backpropagation.h"
#include "../../Fido/include/Simulator/Simlink.h"
#include "../../Fido/include/LSInterpolator.h"
#include "../../Fido/include/FidoControlSystem.h"

void printStats(std::vector<int> iterations) {
	double sum = 0;
	for(auto a = iterations.begin(); a != iterations.end(); a++) sum += *a;
	std::cout << "Average iter: " << (sum / double(iterations.size())) << "\n";

	std::sort(iterations.begin(), iterations.end(), [=](int a, int b) {
		return a < b;
	});
	std::cout << "Median iter: " << iterations[iterations.size() / 2] << "\n";
}

void lineFollowDrive() {
	Simlink simulator;
	rl::WireFitQLearn learner = rl::WireFitQLearn(2, 2, 3, 10, 5, {-1, -1}, {1, 1}, 6, new rl::LSInterpolator(), net::Backpropagation(), 1, 0.4);
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
	double maxDistanceComponent = 20;
	double exploration = 0.2;
	std::vector<int> iterations;

	Simlink simulator;
	rl::WireFitQLearn learner = rl::WireFitQLearn(1, 2, 1, 12, 4, {-1, -1}, {1, 1}, 11, new rl::LSInterpolator(), net::Backpropagation(0.01, 0.9, 0.1, 35000), 0.95, 0.4);
	for(int a = 0; a < 200; a++) {
		learner.reset();
		simulator.placeRobotInRandomPosition();
		
		int goodIter = 0;
		int iter = 0;

		while(goodIter < 3 && iter < 1000) {
			rl::Action action;
			for(int a = 0; a < 2; a++) {
				action = learner.chooseBoltzmanAction({ simulator.isLeftOfLine() }, exploration);
				simulator.robot.setPosition(simulator.robot.getPosition() + sf::Vector2f(action[0]*maxDistanceComponent, action[1]*maxDistanceComponent));
			}

			double lineValue = simulator.distanceFromLine();
			action = learner.chooseBoltzmanAction({ simulator.isLeftOfLine() }, exploration);
			simulator.robot.setPosition(simulator.robot.getPosition() + sf::Vector2f(action[0]*maxDistanceComponent, action[1]*maxDistanceComponent));

			double newLineValue = simulator.distanceFromLine();
			rl::State newState = { simulator.isLeftOfLine() };
			learner.applyReinforcementToLastAction((fabs(lineValue) - fabs(newLineValue)) / sqrt(2*pow(maxDistanceComponent, 2)), newState);

			//std::this_thread::sleep_for(std::chrono::milliseconds(10));

			if(simulator.distanceFromLine() < 80) goodIter++;
			else goodIter = 0;

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
	rl::WireFitQLearn learner = rl::WireFitQLearn(1, 1, 1, 2, 3, {-1}, {1}, 11, new rl::LSInterpolator(), net::Backpropagation(0.01, 0.9, 0.1, 35000), 0.95, 0.4);
	for(int a = 0; a < 200; a++) {
		learner.reset();
		simulator.placeRobotInRandomPosition();

		int iter = 0;
		
		std::vector<double> rotations(0);
		double average = 0;

		while(rotations.size() < 5 || average > 0.4) {
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

int main() {
	goStraight();
}
