#include "neuralnetwork.h"

NeuralNetwork::NeuralNetwork()
{

}

NeuralNetwork::NeuralNetwork(vector<Neuron *> neurons)
{
    this->neurons = neurons;
}

NeuralNetwork::NeuralNetwork(NeuralNetwork *nn)
{
    vector<Neuron *> pNeurons = nn->getNeurons();
    for (unsigned int i = 0; i < pNeurons.size(); i++)
    {
        Neuron *n = new Neuron(pNeurons[i]);
        addNeuron(n);
    }
}

NeuralNetwork::~NeuralNetwork()
{
    qDebug("        Neural network is being deleted...");

    Neuron *neuron;
    for (unsigned int i = 0; i < neurons.size(); i++)
    {
        neuron = neurons[i];
        delete neuron;
    }

    qDebug("        Neural network deleted");
}

void NeuralNetwork::addNeuron(Neuron *n)
{
    neurons.push_back(n);
}

void NeuralNetwork::mutateNeurons()
{
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::normal_distribution<double> distribution(/*mean=*/0.0, /*stddev=*/0.2);

    vector<Neuron *> hiddenNeurons = getHiddenNeurons();

    for (unsigned int i = 0; i < hiddenNeurons.size(); i++)
    {
        Neuron *hn = hiddenNeurons[i];
        vector<double> weights = hn->getInputWeights();
//        for (unsigned int wi = 0; wi < weights.size(); wi++)
//        {
//            double randDouble = distribution(generator);
//            weights[wi] += randDouble;
//        }
        double randDouble = distribution(generator);
        int randWIdx = std::rand() % weights.size();
        weights[randWIdx] += randDouble;
        hn->setInputWeights(weights);
    }

    vector<Neuron *> outputNeurons = getOutputNeurons();

    for (unsigned int i = 0; i < outputNeurons.size(); i++)
    {
        Neuron *on = outputNeurons[i];
        vector<double> weights = on->getInputWeights();
//        for (unsigned int wi = 0; wi < weights.size(); wi++)
//        {
//            double randDouble = distribution(generator);
//            weights[wi] += randDouble;
//        }
        double randDouble = distribution(generator);
        int randWIdx = std::rand() % weights.size();
        weights[randWIdx] += randDouble;
        on->setInputWeights(weights);
    }
}

vector<Neuron *> NeuralNetwork::getInputNeurons()
{
    vector <Neuron *> inputNeurons;
    for (unsigned int ni = 0; ni < neurons.size(); ni++)
    {
        Neuron *n = neurons[ni];
        if (n != nullptr)
        {
            if (n->neuronType == Neuron::Type::Input)
            {
                inputNeurons.push_back(n);
            }
        }
        else
        {
            ni--;
            continue;
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

vector<Neuron *> NeuralNetwork::getNeurons() const
{
    return neurons;
}
