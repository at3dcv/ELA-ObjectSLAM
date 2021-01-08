#include <iostream>
#include <fstream>
#include <vector>

#include "ObjDetectionHelper.h"

using namespace std;

namespace ORB_SLAM2
{
ObjDetectionHelper::ObjDetectionHelper() {}

void ObjDetectionHelper::ReadFile(const string filePath) {
    ifstream objDetectFile;
    objDetectFile.open(filePath);
    char output[20];
    int counter = 1;
    if (objDetectFile.is_open()) {
        while(!ObjDetectFile.eof()) {
            objDetectFile >> output;
            switch(counter % 6) {
                case 1:
                    classes.push_back(stoi(output));
                    break;
                case 2:
                    x.push_back(stoi(output));
                    break;
                case 3:
                    y.push_back(stoi(output));
                    break;
                case 4:
                    height.push_back(stoi(output));
                    break;
                case 5:
                    width.push_back(stoi(output));
                    break;
                case 0:
                    confidence.push_back(stoi(output));
                    break;
            }
            counter+;
        }
        // Always appends the last entry two times...
        classes.pop_back();
    }
    objDetectFile.close();
}

vector<int> ObjDetectionHelper::GetClasses() {
    return classes;
}
}