#include "colony.h"

Colony::Colony()
{

}

void Colony::addCreature(Creature *pCreature)
{
    creatures.push_back(pCreature);
}

vector<Creature *> Colony::getColony()
{
    return creatures;
}

Colony *Colony::getNextGenerationColony()
{
    int curColonySize = getColonySize();
    Colony *nextGenerationColony = new Colony();
    vector<Creature *> susrvivedCreatures = getSurvived();

    Creature *survivedCreature1, *survivedCreature2, *survivedCreature3;

    std::sort(susrvivedCreatures.begin(), susrvivedCreatures.end(), Colony::sortByVitality);

    QGraphicsItem *pCreatureNG1GI, *pCreatureNG2GI, *pCreatureNG3GI;
    Creature *pCreatureNG1, *pCreatureNG2, *pCreatureNG3;
    for (unsigned int i = 0; i < susrvivedCreatures.size() - 2; i++)
    {
        survivedCreature1 = susrvivedCreatures[i];
        survivedCreature2 = susrvivedCreatures[i + 1];
        survivedCreature3 = susrvivedCreatures[i + 2];

        pCreatureNG1GI = new CreatureA(20.0, 20.0, QColor(0, 255, 0, 255), false);
        pCreatureNG1 = dynamic_cast<Creature*>(pCreatureNG1GI);

        pCreatureNG2GI = new CreatureA(20.0, 20.0, QColor(0, 255, 0, 255), false);
        pCreatureNG2 = dynamic_cast<Creature*>(pCreatureNG2GI);

        pCreatureNG3GI = new CreatureA(20.0, 20.0, QColor(0, 255, 0, 255), false);
        pCreatureNG3 = dynamic_cast<Creature*>(pCreatureNG3GI);

        vector<Neuron *>survivedCreature1Neurons = survivedCreature1->nn->getNeurons();
        Neuron *survivedCreature1Neuron;
        vector<Neuron *>survivedCreature2Neurons = survivedCreature2->nn->getNeurons();
        Neuron *survivedCreature2Neuron;
        vector<Neuron *>survivedCreature3Neurons = survivedCreature3->nn->getNeurons();
        Neuron *survivedCreature3Neuron;

        for (unsigned int ni = 0; ni < survivedCreature1Neurons.size(); ni++)
        {
            survivedCreature1Neuron = survivedCreature1Neurons[ni];
            survivedCreature2Neuron = survivedCreature2Neurons[ni];
            survivedCreature3Neuron = survivedCreature3Neurons[ni];

            Neuron *n1 = new Neuron();
            n1->neuronType = survivedCreature1Neuron->neuronType;
            n1->neuronTransferFunction = survivedCreature1Neuron->neuronTransferFunction;
            n1->setInputWeights(survivedCreature1Neuron->getInputWeights());

            Neuron *n2 = new Neuron();
            n2->setInputWeights(survivedCreature2Neuron->getInputWeights());

            Neuron *n3 = new Neuron();
            n3->setInputWeights(survivedCreature3Neuron->getInputWeights());

            Neuron *nNG1 = new Neuron();
            Neuron *nNG2 = new Neuron();
            Neuron *nNG3 = new Neuron();

            nNG1->neuronType = survivedCreature1Neuron->neuronType;
            nNG1->neuronTransferFunction = survivedCreature1Neuron->neuronTransferFunction;

            nNG2->neuronType = survivedCreature1Neuron->neuronType;
            nNG2->neuronTransferFunction = survivedCreature1Neuron->neuronTransferFunction;

            nNG3->neuronType = survivedCreature1Neuron->neuronType;
            nNG3->neuronTransferFunction = survivedCreature1Neuron->neuronTransferFunction;

            vector<double> nNG1InputWeights;
            vector<double> nNG2InputWeights;
            vector<double> nNG3InputWeights;
            for (unsigned w = 0; w < n1->getInputWeights().size(); w++)
            {
                nNG1InputWeights.push_back((n1->getInputWeights()[w] + n2->getInputWeights()[w]) / 2.0);
                nNG2InputWeights.push_back((n1->getInputWeights()[w] + n3->getInputWeights()[w]) / 2.0);
                nNG3InputWeights.push_back((n2->getInputWeights()[w] + n3->getInputWeights()[w]) / 2.0);
            }
            nNG1->setInputWeights(nNG1InputWeights);
            pCreatureNG1->nn->addNeuron(nNG1);
            nNG2->setInputWeights(nNG2InputWeights);
            pCreatureNG2->nn->addNeuron(nNG2);
            nNG3->setInputWeights(nNG3InputWeights);
            pCreatureNG3->nn->addNeuron(nNG3);
        }

        if (nextGenerationColony->getColonySize() < curColonySize)
            nextGenerationColony->addCreature(pCreatureNG1);
        if (nextGenerationColony->getColonySize() < curColonySize)
            nextGenerationColony->addCreature(pCreatureNG2);
        if (nextGenerationColony->getColonySize() < curColonySize)
            nextGenerationColony->addCreature(pCreatureNG3);
    }

    return nextGenerationColony;
}

void Colony::killColony()
{
    for (unsigned int i = 0 ; i < creatures.size(); i++)
    {
        Creature *c = creatures[i];
        c->setLifeSuspended(true);
        c->setLifeStopped(true);
    }
    creatures.clear();
}

int Colony::getColonySize()
{
    return (int)creatures.size();
}

int Colony::getStillAliveNumber()
{
    int sillAlive = 0;
    for (unsigned int i = 0 ; i < creatures.size(); i++)
    {
        Creature *c = creatures[i];
        if (c->getVitality() > 0.0)
        {
            sillAlive++;
        }
    }
    return sillAlive;
}

void Colony::stopLife()
{
    for (unsigned int i = 0 ; i < creatures.size(); i++)
    {
        Creature *c = creatures[i];
        c->setLifeStopped(true);
    }
}

void Colony::suspendLife()
{
    for (unsigned int i = 0; i < creatures.size(); i++)
    {
        Creature *pCreature = dynamic_cast<Creature *>(creatures[i]);
        pCreature->setLifeSuspended(true);
    }
}

void Colony::resumeLife()
{
    for (unsigned int i = 0; i < creatures.size(); i++)
    {
        Creature *pCreature = dynamic_cast<Creature *>(creatures[i]);
        pCreature->setLifeSuspended(false);
    }
}

vector<Creature *> Colony::getSurvived()
{
    vector<Creature *> survivedCreatures;

    for (unsigned int i = 0 ; i < creatures.size(); i++)
    {
        Creature *c = creatures[i];
        if (c->getVitality() > 0.0)
        {
            survivedCreatures.push_back(c);
        }
    }

    return survivedCreatures;
}

bool Colony::sortByVitality(Creature *c1, Creature *c2)
{
    return (c1->getVitality() > c2->getVitality());
}

