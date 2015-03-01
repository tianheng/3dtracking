#include <vector>
#include <string>
using namespace std;

//const parameters
#define USB_PORT "/dev/tty.usbmodem1421" 
#define START &start
#define FREQ 100.00

//structure for node to store points received from sensor.
struct node{
	double x, y, z;
	node* next;
};
//linked list start and end
node* tail;
node start;

// helper function to calculate position
void track_position(int data[], node* temp);
double quick_normal(double q[]);

//current camera position.
double cx = 400.0f, cy = 400.0f, cz = 400.0f;
double lx = 0.0f, lz = 0.0f, ly = 0.0f;

//data string
vector<string> data_buff(5,"\n");

//file descriptor
int fd;

//double buffer for usb reading
string prev_buff;

bool start_flag=false;
//current speed vector
double v[3]={0.00,0.00,0.00};

//current quaternion(orientation)
double q[4]={1.00,0.00,0.00,0.00};

//current angle
double angle[3]={0,0,90};

//start read flag
bool flag_start=false;

//read usb data and then calculate position.
void parse_data(string & buff,int n)
{
  //shitty design to fix usb shitty design
  buff=prev_buff+buff;
  n+=prev_buff.size();
  prev_buff="";
  string tmp_num="";
  int count=0;
  int data[6];
  //sample time interval
  double t=1.00/FREQ;

  //loop data and parse value
  for (int i=0;i<n;i++)
    {
      switch (buff[i])
	{
	case '\n':
	  {
	    //build new node and calculate position
	    node* temp = new node;
	    //get current angle
	    angle[0]+=((double)data[3])*t*250.00/32768.00;
	    angle[1]+=((double)data[4])*t*250.00/32768.00;
	    angle[2]+=((double)data[5])*t*250.00/32768.00;
	    temp->next = NULL;
	    tail->next = temp;
	    tail = temp;
	    prev_buff="";
	    count=0;
	    track_position(data,temp);
	    //cout << "build new node." << endl;
	    //cout << "coordinate for new node are:" << temp->x << " " << temp->y << " " << temp->z << endl;
	    
	    //update information array
	    for (int j = 0; j < 4; j++)
	      data_buff[j] = data_buff[j + 1];
	    data_buff[4] = to_string(temp->x) + "," + to_string(temp->y) + "," + to_string(temp->z) + "\n";
	    break;
	  }
	case ' ':
	  {
	    //get value
	    data[count]=stoi(tmp_num);
	    count++;
	    tmp_num="";
	    prev_buff+=buff[i];
	    break;
	  }
	default:
	  {
	    //prevent loss of data
	    tmp_num+=buff[i];
	    prev_buff+=buff[i];
	    break;
	  }
	}
    }
}


//calcuate position
void track_position(int data[], node* temp)
{
  double t=1.00/FREQ;
  //matrix M
  double m[3][3]={
    {q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3],2*q[1]*q[2]+2*q[0]*q[3],2*q[1]*q[3]-2*q[0]*q[2]},
    {2*q[1]*q[2]-2*q[0]*q[3],q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3],2*q[2]*q[3]+2*q[0]*q[1]},
    {2*q[1]*q[3]+2*q[0]*q[2],2*q[2]*q[3]-2*q[0]*q[1],q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]}
  };
  //delta velocity 
  double delta_v[3]={((double)data[0])*t*2.00*9.81/32768.00,
		     ((double)data[1])*t*2.00*9.81/32768.00,
		     ((double)data[2]-32768.00/2.00)*t*2.00*9.81/32768.00};
  //only for test
  delta_v[2]=0.0;
  delta_v[1]=0.0;
  double inertial_delta_v[3];
  //delta velocity under world frame
  inertial_delta_v[0]=m[0][0]*delta_v[0]+m[0][1]*delta_v[1]+m[0][2]*delta_v[2];
  inertial_delta_v[1]=m[1][0]*delta_v[0]+m[1][1]*delta_v[1]+m[1][2]*delta_v[2];
  inertial_delta_v[2]=m[2][0]*delta_v[0]+m[2][1]*delta_v[1]+m[2][2]*delta_v[2];
  //new velocity
  v[0]+=inertial_delta_v[0];
  v[1]+=inertial_delta_v[1];
  v[2]+=inertial_delta_v[2];
  //distance
  temp->x=v[0]*t*100.00;
  temp->y=v[1]*t*100.00;
  temp->z=v[2]*t*100.00;
  double a=q[0],b=q[1],c=q[2],d=q[3];
  double gx=((double)data[3])*t*250.00/32768.00/2.00;
  double gy=((double)data[4])*t*250.00/32768.00/2.00;
  double gz=((double)data[5])*t*250.00/32768.00/2.00;
  //comment quaternion for test purpose
  /*q[0]+=-b*gx-c*gy-d*gz;
  q[1]+=a*gx+c*gz-d*gz;
  q[2]+=a*gy-b*gz+d*gx;
  q[3]+=a*gz+b*gy-c*gx;*/

  //TODO: need test.
  double norm=quick_normal(q);
  /*q[0]*=norm;
  q[1]*=norm;
  q[2]*=norm;
  q[3]*=norm;*/
}


//Use Newton iteration to calcuate 1/sqrt(). Should be fast.
double quick_normal(double q[])
{
  float sum=q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3];
  long i;
  float x2,y;
  const float threehalfs=1.5;
  
  x2=sum*0.5;
  y=sum;
  i=*(long *) &y;
  i=0x5f3759df - (i>>1);
  y=*(float *) &i;
  y*=(threehalfs-(x2*y*y));
  return (double)y;
  
}
