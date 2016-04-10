# Lab Notebook

This document contains the research logs of [Michael Truell](https://github.com/truell20) and [Joshua Gruenstein](https://github.com/joshuagruenstein). Entries are not stored chronological order, but by topic. This document is version-ed using git, so chronological changes may be viewed.

## Neural Network Structure Optimization

Could be done using pruning. A neural network in the Fido control system would have one layer and many neurons. These would be removed using a common pruning algorithm as described [in this survey](http://axon.cs.byu.edu/~martinez/classes/678/Papers/Reed_PruningSurvey.pdf).

Since we are using a limited neural network architecture, Fido's neural network can only have so many architectures. Because of this, maybe just a stochastic search on the possible architectures in real time would be the best way to adapt Fido's structure to the problem at hand. Each possible architecture would be trained on a sample of Fido's recent history and error for each architecture would be calculated as the error on a set of histories not shown to the network during training. Drawback: there would be considerable latency, but this process could be run in the background during pauses in training; humans are slow, so latency shouldn't be an issue, but it might be a problem for a bot that is training Fido.
