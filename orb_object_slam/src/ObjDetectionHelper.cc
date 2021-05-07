#include <iostream>
#include <fstream>
#include <vector>
#include "ObjDetectionHelper.h"

using namespace std;

namespace ORB_SLAM2
{
ObjDetectionHelper::ObjDetectionHelper() {}

void ObjDetectionHelper::ReadFile(const string filePath) {
  ifstream myReadFile;
  myReadFile.open(filePath);
  char output[20];
  char oldOutput[20];
  int counter = 1;
  if (myReadFile.is_open()) {
    while (!myReadFile.eof()) {
      myReadFile >> output;
      // if file is empty...
      if (!isdigit(output[0])) {
        myReadFile.close();
        return;
      }
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
      counter++;
    }
    // Always appends the last element to class again...
    classes.pop_back();
  }
  myReadFile.close();
}

vector<float> ObjDetectionHelper::GetNthEntry(int n) {
  vector<float> entry;
  entry.push_back(classes[n]);
  entry.push_back(x[n]);
  entry.push_back(y[n]);
  entry.push_back(height[n]);
  entry.push_back(width[n]);
  entry.push_back(confidence[n]);
  return entry;
}

vector<vector<float> > ObjDetectionHelper::GetBBoxes() {
  vector<vector<float> > bboxes;
  for (int i = 0; i < classes.size(); i++) {
    vector<float> bbox;
    bbox = GetNthEntry(i);
    bboxes.push_back(bbox);
  }
  cout << "Found " << bboxes.size() << " objects" << endl;
  return bboxes;
}
}