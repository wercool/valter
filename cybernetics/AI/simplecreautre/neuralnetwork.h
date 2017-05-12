#ifndef NEURALNETWORK_H
#define NEURALNETWORK_H

#include "neuron.h"

#include <QDebug>

using namespace std;

class NeuralNetwork
{
public:
    NeuralNetwork();
    NeuralNetwork(vector<Neuron *> neurons);

    void addNeuron(Neuron* n);

    vector<Neuron *> getInputNeurons();
    vector<Neuron *> getHiddenNeurons();
    vector<Neuron *> getOutputNeurons();

    void feedForward(vector<double> inputs);

private:
    vector<Neuron *> neurons;
};

#endif // NEURALNETWORK_H
