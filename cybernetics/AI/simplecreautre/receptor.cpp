#include "receptor.h"

Receptor::Receptor()
{

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
    // stub
}

double Receptor::getOutput() const
{
    return output;
}

void Receptor::setOutput(double value)
{
    output = value;
}



