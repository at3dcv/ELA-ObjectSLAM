#include <iostream>
#include <fstream>
#include <vector>
#include "objects.h"

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

vector<vector<float> > ObjDetectionHelper::GetBBoxesWithPerson() {
  vector<vector<float> > bboxes;
  for (int i = 0; i < classes.size(); i++) {
    vector<float> bbox;
    if (classes[i] == 1) {
      bbox = GetNthEntry(i);
      bboxes.push_back(bbox);
    }
  }
  return bboxes;
}
}

int main()
{
  TEST::Objects test;

  test = TEST::Objects();
  test.ReadFile("MRCNN_renamed/513.txt");
  int entryCount = test.GetEntryCount();

  cout << entryCount << endl;
  cout << test.GetBBoxesWithPerson().size() << endl;
}