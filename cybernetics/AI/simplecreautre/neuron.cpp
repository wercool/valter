#include "neuron.h"

Neuron::Neuron(Type nt, TransferFunction ntf)
{
    neuronType = nt;
    neuronTransferFunction = ntf;
}

vector<double> Neuron::getInputWeights() const
{
    return inputWeights;
}

void Neuron::setInputWeights(const vector<double> &value)
{
    inputWeights = value;
}

double Neuron::getOutput() const
{
    return output;
}

void Neuron::setOutput(double value)
{
    output = value;
}
