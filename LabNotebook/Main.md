# Lab Notebook

This document contains the research logs of [Michael Truell](https://github.com/truell20) and [Joshua Gruenstein](https://github.com/joshuagruenstein). Entries are not stored chronological order, but by topic. This document is version-ed using git, so chronological changes may be viewed.

## Neural Network Structure Optimization

### Pruning
Could be done using pruning. A neural network in the Fido control system would have one layer and many neurons. These would be removed using a common pruning algorithm as described [in this survey](http://axon.cs.byu.edu/~martinez/classes/678/Papers/Reed_PruningSurvey.pdf).

### Search
Since we are using a limited neural network architecture, Fido's neural network can only have so many architectures. Because of this, maybe just a stochastic search on the possible architectures in real time would be the best way to adapt Fido's structure to the problem at hand. Each possible architecture would be trained on a sample of Fido's recent history and error for each architecture would be calculated as the error on a set of histories not shown to the network during training.

Drawback: there would be considerable latency, but this process could be run in the background during pauses in training; humans are slow, so latency shouldn't be an issue, but it might be a problem for a bot that is training Fido.

Advantage: retrain-ability - pruning by itself wont add neurons if Fido switches to a more difficult task.

### GAs
Similar to the above, a genetic algorithm could be used to generate architectures. Error would be assessed in the same way as above. Latency would still be a drawback. However, this method could generate non-MLP architectures like RNNs, which might result in better NN performance. The [NEAT](http://nn.cs.utexas.edu/downloads/papers/stanley.ec02.pdf) is a popular way of doing things similar to this.

## Exploration Optimization

This refers to the changing Fido's exploration constant.

This could be done simply by calculating the change in the NN's weights. Higher change means Fido is being presented with new information, and so exploration should go up. This should be a generally good heuristic. However, will be affected by the current neural network structure and our structure optimization algorithm

## History Sampling

Because Fido is retrainable, history sampling is an interesting problem. Unlike in [deepmind atari](https://www.cs.toronto.edu/~vmnih/docs/dqn.pdf), the parameters of Fido's task is changing. However, when not retrained, history sampling helps considerably. Therefore, weight needs to be given to more recent histories.

Available History Selection Heuristics
- Use the softmax equation - exploration could change in accordance with Fido's action selection exploration or just be left constant
- Set window of histories from which to sample

## Memory

Many of Fido's tasks are non-Markovian. Fido needs to know more than just the current state. An example would be moving in a square.

### Recurrency
Use an NN architecture with a hidden state (like LTSM). That way past states have an indirect effect on an action, and the effect of each past state decreases with time.

Issue - History sampling cannot be used, because it would effect the hidden state. This could be avoided by recording the hidden state as part of a history and inserting that hidden state into the network during training. However, if that is the case, we cannot evolve neural network architecture, since the dimensions of the hidden state will change from reward iteration to reward iteration.

### Proprioception paired with time as an output
A method particularly suited for tasks that require sequences of actions (like moving in a square) would be to make time of action an output of Fido's NN. Ex. Fido chooses to move forward for 2 seconds. Memory could be simulated by passing in Fido's last action as part of the state.

Problems
- Sort of non-general method especially if you are feeding in more than just the previous action for some tasks, unless you do this for tasks that don't require memory as well.
- If action sequences require more than just the previous action, how would you do weighting of older actions? This might not be an issue. The NN could figure out which actions to prioritize.
