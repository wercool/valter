#include "creatureb.h"

CreatureB::CreatureB(double rx, double ry, const QColor &color): Creature(rx, ry, color)
{
    Receptor *r;

    r = new Receptor(); //Left Sensor       R#0
    r->addInput();
    r->id = 0;
    receptors.push_back(r);

    r = new Receptor(); //Right Sensor      R#1
    r->addInput();
    r->id = 1;
    receptors.push_back(r);
}

void CreatureB::initNN(int hiddenNeuronsNum, int outputNeuronsNum)
{
    Neuron *n;

    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::normal_distribution<double> distribution(/*mean=*/0.0, /*stddev=*/3.0);

    int nid = 0;

    // Input neurons in respect to Receptors
    for (unsigned int ri = 0; ri < receptors.size(); ri++)
    {
        n = new Neuron(Neuron::Type::Input, Neuron::TransferFunction::None);
        vector<double> weights;
        weights.push_back(1.0);
        n->setInputWeights(weights);
        n->id = nid++;
        nn->addNeuron(n);
    }
    // other Input neurons
    // ...

    int inputNeuronNum = (int) nn->getInputNeurons().size();

string weightsStringified = "Hidden Neuron Weights: \n";

    // Hidden neurons
    for (int hln = 0; hln < hiddenNeuronsNum; hln++)
    {
        n = new Neuron(Neuron::Type::Hidden, Neuron::TransferFunction::Sigmoid);
        vector<double> weights;

weightsStringified += format_string("N#%d: ", hln);

        for (int inl = 0; inl < inputNeuronNum; inl++)
        {
            double randDouble = distribution(generator);
weightsStringified += format_string("%.4f ", randDouble);
            weights.push_back(randDouble);
        }
weightsStringified += "\n";

        n->setInputWeights(weights);
        n->id = nid++;
        nn->addNeuron(n);
    }

qDebug("%s", weightsStringified.c_str());

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
        n->id = nid++;
        nn->addNeuron(n);
    }
}

void CreatureB::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(widget);
    Q_UNUSED(painter);

    Creature::paint(painter, option, widget);

    double tailLength = 30.0 + pathLength * 0.0001;



    // RECEPTORS

    double lReceptorAngle = -45.0 * M_PI / 180.0;
    double rReceptorAngle = 45.0 * M_PI / 180.0;
    double lReceptorLength = ry * 4.0;
    double rReceptorLength = ry * 4.0;

    double sinLReceptorAngle = sin(lReceptorAngle);
    double cosLReceptorAngle = cos(lReceptorAngle);
    double sinRReceptorAngle = sin(rReceptorAngle);
    double cosRReceptorAngle = cos(rReceptorAngle);

    double lReceptorX1 = rx * sinLReceptorAngle;
    double lReceptorY1 = -ry * cosLReceptorAngle;
    double lReceptorX2 = lReceptorLength * sinLReceptorAngle;
    double lReceptorY2 = -lReceptorLength * cosLReceptorAngle;

    double rReceptorX1 = rx * sinRReceptorAngle;
    double rReceptorY1 = -ry * cosRReceptorAngle;
    double rReceptorX2 = rReceptorLength * sinRReceptorAngle;
    double rReceptorY2 = -rReceptorLength * cosRReceptorAngle;

    Receptor *lR, *rR;
    lR = receptors[0];
    rR = receptors[1];

    lR->lx = lReceptorX2;
    lR->ly = lReceptorY2;

    rR->lx = rReceptorX2;
    rR->ly = rReceptorY2;

    painter->setPen(QPen(Qt::black, 1.0));
    painter->drawLine(QPointF(lReceptorX1, lReceptorY1), QPointF(lR->lx, lR->ly));
    painter->drawLine(QPointF(rReceptorX1, rReceptorY1), QPointF(rR->lx, rR->ly));

    // Receptors
    painter->setPen(QPen(fillColor, 0.2));
    painter->setBrush(Qt::black);
    painter->drawEllipse(QPointF(lR->lx, lR->ly), 4, 4); //Left Sensor   R#0
    painter->drawEllipse(QPointF(rR->lx, rR->ly), 4, 4); //Right Sensor  R#1



    // Affector tail
    double tailEndX = sin(a * M_PI / 180.0) * (ry + tailLength);
    double tailEndY = cos(a * M_PI / 180.0) * (ry + tailLength);
    QPen pen = QPen(fillColor, 4.0);
    pen.setStyle(Qt::PenStyle::DotLine);
    painter->setPen(pen);
    painter->drawLine(QPointF(0.0, ry + 2.0), QPointF(tailEndX, tailEndY));

    // Body
    painter->setPen(QPen(Qt::black, 0.5));
    painter->setBrush(fillColor);
    painter->drawEllipse(QPointF(0.0, 0.0), rx, ry);

    double neuronRadius = 0.2 * rx;
    const qreal lod = option->levelOfDetailFromTransform(painter->worldTransform());
    if (lod > 1.0)
    {
        QFont font = painter->font() ;
        /* twice the size than the current font size */
        font.setPointSizeF(font.pointSizeF() * 0.5);
        /* set the modified font to the painter */
        painter->setFont(font);

        // Receptor labels
        painter->setPen(QPen(Qt::white, 0.1));
        painter->drawText(QPointF(lR->lx - 1.75, lR->ly + 1.75), "R");
        painter->drawText(QPointF(rR->lx - 1.75, rR->ly + 1.75), "R");

        double neuronsPosY, neuronsPosHorde, neuronPosXStep;

        painter->setPen(QPen(Qt::black, 0.2));
        painter->setBrush(Qt::transparent);

        int inputNeuronNum = (int) nn->getInputNeurons().size();
        vector<QPointF> inputNeuronsCoords;
        neuronsPosY = -ry * 0.5;
        neuronsPosHorde = (sqrt(pow(ry, 2) - pow(neuronsPosY, 2)) * 2.0) * 0.5;
        neuronPosXStep = neuronsPosHorde / ((double) inputNeuronNum - 1.0);
        for (int inl = 0; inl < inputNeuronNum; inl++)
        {
            QPointF nCoords;
            nCoords.setX(-neuronsPosHorde / 2 + neuronPosXStep * inl);
            nCoords.setY(neuronsPosY);
            painter->drawEllipse(nCoords, neuronRadius, neuronRadius);
            inputNeuronsCoords.push_back(nCoords);
        }

        // Left Receptor
        QPointF n;
        n = inputNeuronsCoords[0];
        n.setY(n.y() - neuronRadius);
        painter->drawLine(QPointF(lReceptorX1, lReceptorY1), n);
        // Right Receptor
        n = inputNeuronsCoords[1];
        n.setY(n.y() - neuronRadius);
        painter->drawLine(QPointF(rReceptorX1, rReceptorY1), n);

        int hiddenNeuronNum = (int) nn->getHiddenNeurons().size();
        vector<QPointF> hiddenNeuronsCoords;
        neuronsPosY = 0.0;
        neuronsPosHorde = ry * 2 * 0.5;
        neuronPosXStep = neuronsPosHorde / ((double) hiddenNeuronNum - 1.0);
        for (int hnl = 0; hnl < hiddenNeuronNum; hnl++)
        {
            QPointF nCoords;
            nCoords.setX(-neuronsPosHorde / 2 + neuronPosXStep * hnl);
            nCoords.setY(neuronsPosY);
            painter->drawEllipse(nCoords, neuronRadius, neuronRadius);
            hiddenNeuronsCoords.push_back(nCoords);
        }

        for (int inl = 0; inl < inputNeuronNum; inl++)
        {
            for (int hnl = 0; hnl < hiddenNeuronNum; hnl++)
            {
                QPointF n1 = inputNeuronsCoords[inl];
                n1.setY(n1.y() + neuronRadius);
                QPointF n2 = hiddenNeuronsCoords[hnl];
                n2.setY(n2.y() - neuronRadius);
                painter->drawLine(n1, n2);
            }
        }

        int outputNeuronNum = (int) nn->getOutputNeurons().size();
        vector<QPointF> outputNeuronsCoords;
        neuronsPosY = ry * 0.5;
        neuronsPosHorde = (sqrt(pow(ry, 2) - pow(neuronsPosY, 2)) * 2.0) * 0.5;
        neuronPosXStep = neuronsPosHorde / ((double) outputNeuronNum - 1.0);
        for (int onl = 0; onl < outputNeuronNum; onl++)
        {
            QPointF nCoords;
            nCoords.setX(-neuronsPosHorde / 2 + neuronPosXStep * onl);
            nCoords.setY(neuronsPosY);
            painter->drawEllipse(nCoords, neuronRadius, neuronRadius);
            outputNeuronsCoords.push_back(nCoords);
        }

        for (int hnl = 0; hnl < hiddenNeuronNum; hnl++)
        {
            for (int onl = 0; onl < outputNeuronNum; onl++)
            {
                QPointF n1 = hiddenNeuronsCoords[hnl];
                n1.setY(n1.y() + neuronRadius);
                QPointF n2 = outputNeuronsCoords[onl];
                n2.setY(n2.y() - neuronRadius);
                painter->drawLine(n1, n2);
            }
        }

        // Affector tail connections
        for (int onl = 0; onl < outputNeuronNum; onl++)
        {
            QPointF n = outputNeuronsCoords[onl];
            n.setY(n.y() + neuronRadius);
            painter->drawLine(n, QPointF(0.0, ry));
        }

    }
}

void CreatureB::lifeThreadProcess()
{
    int cnt = 0;

    while (alive)
    {
        cnt++;

        cv::Rect rect(cv::Point(), envMapMat->size());

        double aRad = getA() * M_PI / 180.0;

        Receptor *lR = receptors[0];
        lR->gx = getX() + lR->lx * cos(aRad) - lR->ly * sin(aRad);
        lR->gy = getY() + lR->lx * sin(aRad) + lR->ly * cos(aRad);
        cv::Point plR((int)lR->gx, (int)lR->gy);
        if (rect.contains(plR))
        {
            lR->setIntputs({ (255.0 - (double) envMapMat->at<uchar>(plR) / 255.0) });
        }
        else
        {
            lR->setIntputs({ 0.0 });
        }
        lR->receptorFunction();

        Receptor *rR = receptors[1];
        rR->gx = getX() + rR->lx * cos(aRad) - rR->ly * sin(aRad);
        rR->gy = getY() + rR->lx * sin(aRad) + rR->ly * cos(aRad);
        cv::Point prR((int)rR->gx, (int)rR->gy);
        if (rect.contains(prR))
        {
            rR->setIntputs({ (255.0 - (double) envMapMat->at<uchar>(prR) / 255.0) });
        }
        else
        {
            rR->setIntputs({ 0.0 });
        }
        rR->receptorFunction();

        vitality -= 0.1;

        double inputIntensity = 0.0;
        cv::Point pc(getIntX(), getIntY());
        if (rect.contains(pc))
        {
            for (int xs = -rx; xs < rx; xs++)
            {
                for (int ys = -ry; ys < ry; ys++)
                {
                    if((xs*xs + ys*ys) < rx*ry)
                    {
                        cv::Point pcs(getIntX() + xs, getIntY() + ys);
                        if (rect.contains(pcs))
                        {
                            double intensity = (double) envMapMat->at<uchar>(pcs);
                            if (intensity + 1 < 255.0)
                            {
                                inputIntensity += (255.0 - intensity) / 25500.0;
                            }
                        }
                    }
                }
            }
        }
        vitality += (vitality >= 1.0) ? 0.0 : inputIntensity;

        if (vitality > 0)
        {
            nn->feedForward({ lR->getOutput(), rR->getOutput() });

            vector <Neuron *> outputNeurons = nn->getOutputNeurons();
            Neuron *n0 = outputNeurons[0];
            Neuron *n1 = outputNeurons[1];
            Neuron *n2 = outputNeurons[2];

            double stimulatorXY = rx;
            double stimulatorA  = 1.0 / vitality;

            double cA = (n2->getOutput() - n0->getOutput()) * stimulatorA * 180.0 / M_PI;
            double cX = getX() + n1->getOutput() * sin(getA()) * stimulatorXY;
            double cY = getY() + n1->getOutput() * cos(getA()) * stimulatorXY;

            double dPath = sqrt(pow((getX() - cX), 2) + pow((getY() - cY), 2)) * 0.01;
            pathLength += (dPath > 0.001) ? dPath : 0.0;

            setA(cA);
            setX(cX);
            setY(cY);
        }
        else
        {
            color = QColor(0, 0, 0, 255);
            break;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(dLifeTime + 1));
    }
    lifeThread = nullptr;
}

