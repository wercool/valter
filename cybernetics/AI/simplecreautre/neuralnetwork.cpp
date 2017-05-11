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

vector<Neuron *> NeuralNetwork::getInputNeurons()
{
    vector <Neuron *> inputNeurons;
    for (unsigned int ni = 0; ni < neurons.size(); ni++)
    {
        Neuron *n = neurons[ni];
        if (n->neuronType == Neuron::Type::Input)
        {
            inputNeurons.push_back(n);
        }
    }
    return inputNeurons;
}

vector<Neuron *> NeuralNetwork::getHiddenNeurons()
{
    vector <Neuron *> hiddenNeurons;
    for (unsigned int ni = 0; ni < neurons.size(); ni++)
    {
        Neuron *n = neurons[ni];
        if (n->neuronType == Neuron::Type::Hidden)
        {
            hiddenNeurons.push_back(n);
        }
    }
    return hiddenNeurons;
}

vector<Neuron *> NeuralNetwork::getOutputNeurons()
{
    vector <Neuron *> outputNeurons;
    for (unsigned int ni = 0; ni < neurons.size(); ni++)
    {
        Neuron *n = neurons[ni];
        if (n->neuronType == Neuron::Type::Output)
        {
            outputNeurons.push_back(n);
        }
    }
    return outputNeurons;
}

void NeuralNetwork::feedForward(vector<double> inputs)
{

}
