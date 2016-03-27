#include <iostream>

#include <chrono>
#include <thread>

#include "../../Fido/Software/WireFitQLearn.h"
#include "../../Fido/Software/NeuralNet.h"
#include "../../Fido/Software/Backpropagation.h"
#include "../../Fido/Software/FidoSim/Simlink.h"
#include "../../Fido/Software/LSInterpolator.h"

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

void lineFollowHolo() {
	double component = 20;
	double averageIterations = 0;

	Simlink simulator;
	rl::WireFitQLearn learner = rl::WireFitQLearn(1, 2, 4, 10, 4, {-1, -1}, {1, 1}, 11, new rl::LSInterpolator(), net::Backpropagation(), 0.95, 0.4);
	for(int a = 0; a < 200; a++) {
		std::cout << a << "; average " << averageIterations << "\n";

		double exploration = 0.2;
		learner.reset();
		simulator.placeRobotInRandomPosition();
		
		int goodIter = 0;
		int iter = 0;

		while(goodIter < 5 && iter < 1000) {
			//exploration *= 0.99;

			rl::Action action;
			for(int a = 0; a < 2; a++) {
				action = learner.chooseBoltzmanAction({ simulator.isLeftOfLine() }, exploration);
				simulator.robot.setPosition(simulator.robot.getPosition() + sf::Vector2f(action[0]*component, action[1]*component));
			}

			double lineValue = simulator.distanceFromLine();
			action = learner.chooseBoltzmanAction({ simulator.isLeftOfLine() }, exploration);
			simulator.robot.setPosition(simulator.robot.getPosition() + sf::Vector2f(action[0]*component, action[1]*component));
			
			double newLineValue = simulator.distanceFromLine();
			rl::State newState = { simulator.isLeftOfLine() };
			learner.applyReinforcementToLastAction((fabs(lineValue) - fabs(newLineValue)) / sqrt(2*pow(component, 2)), newState);

			//std::cout << "left?, rotation, distance: " << simulator.isLeftOfLine() << " " << simulator.robot.getRotation() / 360.0 << " " << lineValue << " " << ((fabs(lineValue) - fabs(newLineValue)) / 50.0) << "\n"; 

			//std::this_thread::sleep_for(std::chrono::milliseconds(10));

			if(simulator.distanceFromLine() < 80) goodIter++;
			else goodIter = 0;

			iter++;
		}

		averageIterations = (averageIterations*a + iter) / double(a+1);
	}

	std::cout << "Average iter: " << averageIterations << "\n";
}

int main() {
	lineFollowHolo();
}
