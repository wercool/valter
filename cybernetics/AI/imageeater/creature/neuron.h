#ifndef NEURON_H
#define NEURON_H


#include <iostream>
#include <vector>
#include <math.h>

#include <QDebug>

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
        Sigmoid,
        HyperbolicTangent
    };
    Neuron();
    Neuron(Type nt, TransferFunction ntf);
    virtual ~Neuron();
    Type neuronType;
    TransferFunction neuronTransferFunction;

    void setInputs(vector<double> inputs);

    vector<double> getInputWeights() const;
    void setInputWeights(const vector<double> &value);

    double getOutput() const;
    void setOutput(double value);

    int id = 0;

private:
    vector<double> inputWeights;
    double output = 0.0;
};

#endif // NEURON_H
