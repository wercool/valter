#ifndef PERLINNOISEENVIRONMENT_H
#define PERLINNOISEENVIRONMENT_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <math.h>
#include <time.h>

class PerlinNoiseEnvironment
{
public:
    PerlinNoiseEnvironment();

    cv::Mat generate(const cv::Size &size, const double &scale);

private:
    int p[512];
    double noise(const double &src_x, const double &src_y, const double &src_z);

protected:
    double fade(double t)
    {
        return t * t * t * (t * (t * 6 - 15) + 10);
    }

    double lerp(double t, double a, double b)
    {
        return a + t * (b - a);
    }

    double grad(int hash, double x, double y, double z)
    {
        int h = hash & 15;
        double u = h < 8 ? x : y;
        double v = h < 4 ? y : h == 12 || h == 14 ? x : z;
        return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
    }
};

#endif // PERLINNOISEENVIRONMENT_H
