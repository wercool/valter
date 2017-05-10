#ifndef NEURON_H
#define NEURON_H

#include <iostream>
#include <vector>

using namespace std;
class Neuron
{
public:
    enum NeuronType
    {
        Input,
        Hidden,
        Output
    };

    enum NeuronFunction
    {
        Perceptron,
        Sigmoid
    };

    Neuron(NeuronType nt, NeuronFunction nf);

private:
    vector<double> inputWeights;
    double bias;
};

#endif // NEURON_H
