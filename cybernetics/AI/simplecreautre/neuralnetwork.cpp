#include "neuralnetwork.h"

NeuralNetwork::NeuralNetwork()
{

}

NeuralNetwork::NeuralNetwork(vector<Neuron *> neurons)
{
    this->neurons = neurons;
}

void NeuralNetwork::addNeuron(Neuron *n)
{
    neurons.push_back(n);
}
