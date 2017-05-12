#include "neuron.h"

Neuron::Neuron(Type nt, TransferFunction ntf)
{
    neuronType = nt;
    neuronTransferFunction = ntf;
}

void Neuron::setInputs(vector<double> inputs)
{
    this->output = 0.0;
    for (unsigned int i = 0; i < inputs.size(); i++)
    {
        this->output += inputWeights[i] * inputs[i];
    }

    switch (neuronTransferFunction)
    {
        case None:
            //;
        break;
        case Perceptron:
            this->output = (this->output > 1.0) ? 1.0 : 0.0;
        break;
        case Sigmoid:
            this->output = 1.0 / (1.0 + exp(-this->output));
        break;
        case HyperbolicTangent:
            this->output = (exp(this->output) - exp(-this->output)) / (exp(this->output) + exp(-this->output));
        break;
    }
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
