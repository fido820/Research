# Fido: A Universal Robot Control System using Reinforcement Learning with Limited Feedback

Researched and built by [Josh Gruenstein](https://github.com/joshuagruenstein "@joshuagruenstein") and [Michael Truell](https://github.com/truell20 "@truell20").

## Abstract

A robot control system was developed that could be taught tasks through reinforcement learning. The system, nicknamed “Fido”, was designed to be universal regardless of inputs and outputs, robot kinematics, and processing capability.   In addition, Fido was built to learn with limited feedback, allowing humans to train Fido in a minimal amount of time. This was achieved through the training of artificial neural networks with a wire-fitted interpolator following the Q-learning reinforcement learning algorithm and an intelligent action selection policy that utilizes a probabilistic approach to exploration.  Functionality was first tested and evaluated in simulation.  Next, hardware implementations of differing kinematics, sensors, and central processors were constructed.  Fido successfully converged on all given tasks in simulation and in hardware within very few reward iterations while maintaining impressively low latency, demonstrating its potential as a comprehensive robot control system.

## Project Summary

Robotic intelligence can be simplified to a black box, with inputs such as sensor data and outputs such as motor control.  Most robotic software today operates as an "expert system," using preprogrammed logic to execute a set routine.  These implementations are sufficient for the specific purpose and platform that they are designed for but lack the ability to perform other tasks or work on other robots.

The purpose of this project was to develop a universal robot control system that does not require preprogrammed logic to accomplish specific tasks, can be adapted to any robot hardware, and can be trained in a short amount of time using positive and negative reinforcement.  The control system we developed (nicknamed Fido) is lightweight and resource-efficient, making it a practical solution for mobile robots.  This was achieved by employing an artificial neural network and a novel learning algorithm.  The algorithm takes a well-tested learning algorithm called Q-learning and modifies it to be better suited for robotic tasks.

Fido was tested on a computer simulated robot with a motor control system, a large sensor array, and some additional outputs.  The system performed well doing a variety of tasks, such as learning to drive to a radio beacon and following a line.  Two hardware implementations were also created for additional testing.  These implementations used low cost and low power embedded Linux systems to run the Fido software, and were succesfully trained to perform complex tasks such as line following, learning a kiwi holonomic drive system, following a ball, and adapting to mechanical failures.

Further details can be found in the (https://github.com/FidoProject/Research/raw/master/Paper/Paper.pdf)[research paper] and (https://github.com/FidoProject/Research/raw/master/Paper/Poster.pdf)[poster].

## Awards

Fido advanced to the finals of the New York City Science and Engineering Fair.  There it won the Intel Excellence in Computer Science award, placed first in the Computer Science category, and was selected to represent NYC at the Intel International Science and Engineering Fair.

Fido will compete at ISEF at the beginning of May 2016.
