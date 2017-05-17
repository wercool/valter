#include "receptor.h"

Receptor::Receptor()
{

}

Receptor::~Receptor()
{
    qDebug("    Receptor #%d deleted", id);
}

void Receptor::addInput(double value)
{
    intputs.push_back(value);
}

vector<double> Receptor::getIntputs() const
{
    return intputs;
}

void Receptor::setIntputs(const vector<double> &value)
{
    intputs = value;
}

void Receptor::receptorFunction()
{
    // simple summuray
    double output = 0.0;
    for (unsigned int i = 0; i < intputs.size(); i++)
    {
        output += intputs[i];
    }
    setOutput(output);
}

double Receptor::getOutput() const
{
    return output;
}

void Receptor::setOutput(double value)
{
    output = value;
}



