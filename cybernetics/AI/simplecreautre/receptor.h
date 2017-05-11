#ifndef RECEPTOR_H
#define RECEPTOR_H

#include <iostream>
#include <vector>

using namespace std;

class Receptor
{
public:
    Receptor();

    void addInput(double value = 0.0);

    vector<double> getIntputs() const;
    void setIntputs(const vector<double> &value);

    virtual void receptorFunction();

    double getOutput() const;
    void setOutput(double value);

    double lx = 0.0;
    double ly = 0.0;

    double gx = 0.0;
    double gy = 0.0;

private:
    vector<double> intputs;
    double output = 0.0;
};

#endif // RECEPTOR_H
