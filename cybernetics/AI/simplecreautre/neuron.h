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
        None,
        Perceptron,
        Sigmoid
    };

    Neuron(Type nt, TransferFunction ntf);
    Type neuronType;
    TransferFunction neuronTransferFunction;

    vector<double> getInputWeights() const;
    void setInputWeights(const vector<double> &value);

    double getOutput() const;
    void setOutput(double value);

private:
    vector<double> inputWeights;
    double output = 0.0;
};

#endif // NEURON_H
