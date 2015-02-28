#include <vector>
#include <string>
using namespace std;

#define USB_PORT "/dev/tty.usbmodem1421" 
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

//file descriptor
int fd;

//double buffer for usb reading
string prev_buff;

bool start_flag=false;
//current speed vector
double v[3];

//current quaternion(orientation)
double q[4];


//read usb data and then calculate position.
void parse_data(string & buff,int n)
{
  buff=prev_buff+buff;
  n+=prev_buff.size();
  prev_buff="";
  string tmp_num="";
  int count=0;
  int data[6];
  for (int i=0;i<n;i++)
    {
      switch (buff[i])
	{
	case '\n':
	  {
	    node* temp = new node;
	    temp->x=data[0];
	    temp->y=data[1];
	    temp->z=data[2];
	    temp->next = NULL;
	    tail->next = temp;
	    tail = temp;
	    prev_buff="";
	    count=0;
	    //cout << "build new node." << endl;
	    //cout << "coordinate for new node are:" << temp->x << " " << temp->y << " " << temp->z << endl;
	    for (int j = 0; j < 4; j++)
	      data_buff[j] = data_buff[j + 1];
	    data_buff[4] = to_string(temp->x) + "," + to_string(temp->y) + "," + to_string(temp->z) + "\n";
	    break;
	  }
	case ' ':
	  {
	    data[count]=stoi(tmp_num);
	    count++;
	    tmp_num="";
	    prev_buff+=buff[i];
	    break;
	  }
	default:
	  {
	    tmp_num+=buff[i];
	    prev_buff+=buff[i];
	    break;
	  }
	}
    }
}
