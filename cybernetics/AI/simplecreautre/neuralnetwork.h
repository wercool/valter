#ifndef NEURALNETWORK_H
#define NEURALNETWORK_H

#include "neuron.h"

using namespace std;

class NeuralNetwork
{
public:
    NeuralNetwork();
    NeuralNetwork(vector<Neuron *> neurons);

    void addNeuron(Neuron* n);

private:
    vector<Neuron *> neurons;
};

#endif // NEURALNETWORK_H
