#ifndef OBJDETECTION_H
#define OBJDETECTION_H

#include <vector>

using namespace std;

namespace ORB_SLAM2
{
class ObjDetectionHelper
{

public:
    ObjDetectionHelper();
    void ReadFile(const string filePath);
    vector<float> GetNthEntry(int n);
    vector<vector<float > > GetBBoxes();

private:
    vector<int> classes;
    vector<int> x;
    vector<int> y;
    vector<float> height;
    vector<float> width;
    vector<float> confidence;
};
}

#endif