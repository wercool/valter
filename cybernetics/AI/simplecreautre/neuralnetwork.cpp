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
    vector<double> intputLayerOutput;
    vector <Neuron *> inputNeurons = getInputNeurons();
    for (unsigned int ni = 0; ni < inputNeurons.size(); ni++)
    {
        Neuron *n = inputNeurons[ni];
        n->setInputs({ inputs[ni] });
        intputLayerOutput.push_back(n->getOutput());
    }

    vector<double> hiddenLayerOutput;
    vector <Neuron *> hiddenNeurons = getHiddenNeurons();
    for (unsigned int ni = 0; ni < hiddenNeurons.size(); ni++)
    {
        Neuron *n = hiddenNeurons[ni];
        n->setInputs(intputLayerOutput);
        hiddenLayerOutput.push_back(n->getOutput());
    }

    vector <Neuron *> outputNeurons = getOutputNeurons();
    for (unsigned int ni = 0; ni < outputNeurons.size(); ni++)
    {
        Neuron *n = outputNeurons[ni];
        n->setInputs(hiddenLayerOutput);
    }
}
