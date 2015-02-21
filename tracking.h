#include <vector>
#include <string>
using namespace std;

#define START &start;

//structure for node to store points received from sensor.
struct node{
	double x, y, z;
	node* next;
};
node* tail;
node start;

//current camera position.
double cx = 400.0f, cy = 400.0f, cz = 400.0f, angle = 0.0f;
double lx = 0.0f, lz = 0.0f, ly = 0.0f;

//data string
vector<string> data_buff(5,"\n");