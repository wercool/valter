#include "perlinnoisemat.h"

PerlinNoiseMat::PerlinNoiseMat()
{
    srand(time(NULL));

    for (int i = 0; i < 256; i++)
    {
        p[256 + i] = p[i] = rand() % 200;
    }
}

cv::Mat PerlinNoiseMat::generate(const cv::Size &size, const double &scale)
{
    cv::Mat img = cv::Mat(size, CV_8UC1);

    for (int y = 0; y < size.height; ++y)
    {
        for (int x = 0; x < size.width; ++x)
        {
            double p = noise(x  * scale, y * scale, 0.0); // -1.0`1.0
            p = (p + 1.0) / 2.0; // 0.0`1.0
            img.at<uchar>(cv::Point(x, y)) = (uchar)(p * 255);
        }
    }

    return img;
}

double PerlinNoiseMat::noise(const double &src_x, const double &src_y, const double &src_z)
{
    int X = (int)floor(src_x) & 255;
    int Y = (int)floor(src_y) & 255;
    int Z = (int)floor(src_z) & 255;

    double x = src_x - floor(src_x);
    double y = src_y - floor(src_y);
    double z = src_z - floor(src_z);

    double u = fade(x);
    double v = fade(y);
    double w = fade(z);

    int A = p[X] + Y;
    int AA = p[A] + Z;
    int AB = p[A + 1] + Z;
    int B = p[X + 1] + Y;
    int BA = p[B] + Z;
    int BB = p[B + 1] + Z;

    double r = lerp(
        w,
        lerp(v,
            lerp(u, grad(p[AA], x, y, z), grad(p[BA], x - 1, y, z)),
            lerp(u, grad(p[AB], x, y - 1, z),
            grad(p[BB], x - 1, y - 1, z))),
        lerp(v,
            lerp(u, grad(p[AA + 1], x, y, z - 1),
            grad(p[BA + 1], x - 1, y, z - 1)),
            lerp(u, grad(p[AB + 1], x, y - 1, z - 1),
            grad(p[BB + 1], x - 1, y - 1, z - 1))));

    return r;
}
