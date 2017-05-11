#include "creaturea.h"

#include <random>

CreatureA::CreatureA(double w, double h, const QColor &color) : Creature(w, h, color)
{
    Receptor *r;

    r = new Receptor(); //Left Sensor  R#0
    r->addInput();
    receptors.push_back(r);

    r = new Receptor(); //Right Sensor R#1
    r->addInput();
    receptors.push_back(r);

    r = new Receptor(); //Creature vitality R#2
    r->addInput(1.0);
    receptors.push_back(r);

    nn = new NeuralNetwork();

    int hiddenNeuronsNum = 5;
    int outputNeuronsNum = 3;

    Neuron *n;

    std::default_random_engine generator;
    std::normal_distribution<double> distribution(/*mean=*/0.0, /*stddev=*/1.0);

    // Input neurons in respect to Receptors
    for (unsigned int ri = 0; ri < receptors.size(); ri++)
    {
        n = new Neuron(Neuron::Type::Input, Neuron::TransferFunction::None);
        vector<double> weights;
        weights.push_back(1.0);
        n->setInputWeights(weights);
        nn->addNeuron(n);
    }
    // other Input neurons
    // ...

    int inputNeuronNum = (int) nn->getInputNeurons().size();

    // Hidden neurons
    for (int hln = 0; hln < hiddenNeuronsNum; hln++)
    {
        n = new Neuron(Neuron::Type::Hidden, Neuron::TransferFunction::Sigmoid);
        vector<double> weights;
        for (int inl = 0; inl < inputNeuronNum; inl++)
        {
            double randDouble = distribution(generator);
            weights.push_back(randDouble);
        }
        n->setInputWeights(weights);
        nn->addNeuron(n);
    }

    // Output neurons
    for (int oln = 0; oln < outputNeuronsNum; oln++)
    {
        n = new Neuron(Neuron::Type::Output, Neuron::TransferFunction::Sigmoid);
        vector<double> weights;
        for (int hnl = 0; hnl < hiddenNeuronsNum; hnl++)
        {
            double randDouble = distribution(generator);
            weights.push_back(randDouble);
        }
        n->setInputWeights(weights);
        nn->addNeuron(n);
    }

    lifeThread = new std::thread(&CreatureA::lifeThreadProcess, this);
}

void CreatureA::lifeThreadProcess()
{
    while (true)
    {
        double aRad = a * M_PI / 180.0;
        Receptor *lR = receptors[0];
        lR->gx = x + (lR->lx * cos(aRad)) - (lR->ly * sin(aRad));
        lR->gy = y + (lR->ly * cos(aRad)) + (lR->lx * sin(aRad));

        Receptor *rR = receptors[1];
        rR->gx = x + (rR->lx * cos(aRad)) - (rR->ly * sin(aRad));
        rR->gy = y + (rR->ly * cos(aRad)) + (rR->lx * sin(aRad));

        this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

void CreatureA::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Creature::paint(painter, option, widget);

    double centerX = (double)w / 2.0;
    double centerY = (double)h / 2.0;

    double lReceptorAlpha = -45.0 * M_PI / 180.0;
    double rReceptorAlpha = 45.0 * M_PI / 180.0;
    double lReceptorLength = h + 25.0;
    double rReceptorLength = h + 25.0;
    double tailLength = 150.0;

    double sinLReceptorAlpha = sin(lReceptorAlpha);
    double cosLReceptorAlpha = cos(lReceptorAlpha);

    double sinRReceptorAlpha = sin(rReceptorAlpha);
    double cosRReceptorAlpha = cos(rReceptorAlpha);

    double lReceptorX1 = centerX * sinLReceptorAlpha;
    double lReceptorY1 = -centerY * cosLReceptorAlpha;

    double lReceptorX2 = lReceptorLength * sinLReceptorAlpha;
    double lReceptorY2 = -lReceptorLength * cosLReceptorAlpha;

    double rReceptorX1 = centerX * sinRReceptorAlpha;
    double rReceptorY1 = -centerY * cosRReceptorAlpha;

    double rReceptorX2 = rReceptorLength * sinRReceptorAlpha;
    double rReceptorY2 = -rReceptorLength * cosRReceptorAlpha;

    Receptor *lR, *rR;
    lR = receptors[0];
    rR = receptors[1];

    lR->lx = centerX + lReceptorX2;
    lR->ly = centerY + lReceptorY2;

    rR->lx = centerX + rReceptorX2;
    rR->ly = centerY + rReceptorY2;

    painter->setPen(QPen(Qt::black, 2.0));
    painter->drawLine(centerX + lReceptorX1, centerY + lReceptorY1, lR->lx, lR->ly);
    painter->drawLine(centerX + rReceptorX1, centerY + rReceptorY1, rR->lx, rR->ly);

    painter->setBrush(Qt::black);
    painter->drawEllipse((centerX - 4) + lReceptorX2, (centerY - 4) + lReceptorY2, 8, 8);
    painter->drawEllipse((centerX - 4) + rReceptorX2, (centerY - 4) + rReceptorY2, 8, 8);

    // Affector tail
    double tailEndX = sin(a * M_PI / 180.0) * (h + tailLength);
    double tailEndY = cos(a * M_PI / 180.0) * (h + tailLength);
    QPen pen = QPen(Qt::green, 4.0);
    pen.setStyle(Qt::PenStyle::DotLine);
    painter->setPen(pen);
    painter->drawLine(centerX, h + 2.0, centerX + tailEndX, tailEndY);

    painter->setPen(QPen(Qt::lightGray, 0.5));
    painter->setBrush(fillColor);
    painter->drawEllipse(0, 0, w, h);

    double neuronSize = 4.0;
    double neuronSizeHalf = neuronSize / 2.0;
    const qreal lod = option->levelOfDetailFromTransform(painter->worldTransform());
    if (lod > 1.0)
    {
        QFont font = painter->font() ;
        /* twice the size than the current font size */
        font.setPointSize(font.pointSize() * 0.5);
        /* set the modified font to the painter */
        painter->setFont(font);

        painter->setPen(QPen(fillColor, 0.1));
        painter->drawText(centerX + lReceptorX2 - 2.0, centerY + lReceptorY2 + 1.0, "R");
        painter->drawText(centerX + rReceptorX2 - 1.0, centerY + rReceptorY2 + 1.0, "R");

        painter->setPen(QPen(Qt::gray, 0.2));
        painter->setBrush(Qt::transparent);

        int inputNeuronNum = (int) nn->getInputNeurons().size();
        vector<vector<double>> inputNeuronsCoords;
        for (int inl = 0; inl < inputNeuronNum; inl++)
        {
            vector<double> nCoords;
            nCoords.push_back(centerX - (centerX / 3) * (inputNeuronNum / 2 - inl) - neuronSizeHalf);
            nCoords.push_back(centerY - centerY / 2 - neuronSizeHalf);
            painter->drawEllipse(nCoords[0], nCoords[1], neuronSize, neuronSize);
            inputNeuronsCoords.push_back(nCoords);
        }

        // Left Receptor
        painter->drawLine(centerX + lReceptorX1, centerY + lReceptorY1, inputNeuronsCoords[0][0] + neuronSizeHalf, inputNeuronsCoords[0][1]);
        // Right Receptor
        painter->drawLine(centerX + rReceptorX1, centerY + rReceptorY1, inputNeuronsCoords[1][0] + neuronSizeHalf, inputNeuronsCoords[1][1]);

        int hiddenNeuronNum = (int) nn->getHiddenNeurons().size();
        vector<vector<double>> hiddenNeuronsCoords;
        for (int hnl = 0; hnl < hiddenNeuronNum; hnl++)
        {
            vector<double> nCoords;
            nCoords.push_back(centerX - (centerX / 4) * (hiddenNeuronNum / 2 - hnl)  - neuronSizeHalf);
            nCoords.push_back(centerY - neuronSizeHalf);
            painter->drawEllipse(nCoords[0], nCoords[1], neuronSize, neuronSize);
            hiddenNeuronsCoords.push_back(nCoords);
        }

        for (int inl = 0; inl < inputNeuronNum; inl++)
        {
            for (int hnl = 0; hnl < hiddenNeuronNum; hnl++)
            {
                painter->drawLine(inputNeuronsCoords[inl][0] + neuronSizeHalf, inputNeuronsCoords[inl][1] + neuronSize, hiddenNeuronsCoords[hnl][0] + neuronSizeHalf, hiddenNeuronsCoords[hnl][1]);
            }
        }

        int outputNeuronNum = (int) nn->getOutputNeurons().size();
        vector<vector<double>> outputNeuronsCoords;
        for (int onl = 0; onl < outputNeuronNum; onl++)
        {
            vector<double> nCoords;
            nCoords.push_back(centerX - (centerX / 4) * (outputNeuronNum / 2 - onl)  - neuronSizeHalf);
            nCoords.push_back(centerY  + centerY / 2 - neuronSizeHalf);
            painter->drawEllipse(nCoords[0], nCoords[1], neuronSize, neuronSize);
            outputNeuronsCoords.push_back(nCoords);
        }

        for (int hnl = 0; hnl < hiddenNeuronNum; hnl++)
        {
            for (int onl = 0; onl < outputNeuronNum; onl++)
            {
                painter->drawLine(hiddenNeuronsCoords[hnl][0] + neuronSizeHalf, hiddenNeuronsCoords[hnl][1] + neuronSize, outputNeuronsCoords[onl][0] + neuronSizeHalf, outputNeuronsCoords[onl][1]);
            }
        }

        // Affector tail
        for (int onl = 0; onl < outputNeuronNum; onl++)
        {
            painter->drawLine(outputNeuronsCoords[onl][0] + neuronSizeHalf, outputNeuronsCoords[onl][1] + neuronSize, centerX, h);
        }

    }
}

vector<Receptor *> CreatureA::getReceptors() const
{
    return receptors;
}
