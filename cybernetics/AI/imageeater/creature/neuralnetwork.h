#ifndef NEURALNETWORK_H
#define NEURALNETWORK_H

#include <chrono>

#include "neuron.h"

#include <QDebug>

using namespace std;

class NeuralNetwork
{
public:
    NeuralNetwork();
    NeuralNetwork(vector<Neuron *> neurons);
    NeuralNetwork(NeuralNetwork *nn);
    virtual ~NeuralNetwork();

    void addNeuron(Neuron* n);
    void mutateNeurons();

    vector<Neuron *> getInputNeurons();
    vector<Neuron *> getHiddenNeurons();
    vector<Neuron *> getOutputNeurons();

    void feedForward(vector<double> inputs);

    vector<Neuron *> getNeurons() const;

private:
    vector<Neuron *> neurons;
};

#endif // NEURALNETWORK_H
