#include <vector>

using namespace std;

namespace TEST
{
class Objects
{

public:
    Objects();
    void ReadFile(const string filePath);
    vector<float> GetNthEntry(int n);
    vector<vector<float > > GetBBoxesWithPerson();

private:
    vector<int> classes;
    vector<int> x;
    vector<int> y;
    vector<float> height;
    vector<float> width;
    vector<float> confidence;
};
}