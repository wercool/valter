#ifndef NEURON_H
#define NEURON_H

#include <iostream>
#include <vector>

using namespace std;
class Neuron
{
public:
    enum Type
    {
        Input,
        Hidden,
        Output
    };

    enum TransferFunction
    {
        Perceptron,
        Sigmoid
    };

    Neuron(Type nt, TransferFunction ntf);

private:
    vector<double> inputWeights;
};

#endif // NEURON_H
