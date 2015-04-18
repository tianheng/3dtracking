#include <vector>
#include <string>
#include <math.h>
using namespace std;

//const parameters
#define USB_PORT "/dev/tty.usbserial-DA017XRQ"
#define START &start
#define FREQ 100.00
#define ZERO_V_THRESHOLD 0.00035
#define GRAVITY_FIX 9.81
#define PI 3.14159265
#define AX data[0]
#define AY data[1]
#define AZ data[2]
#define GX data[3]
#define GY data[4]
#define GZ data[5]
#define MX data[6]
#define MY data[7]
#define MZ data[8]
#define BETA 0.4f
#define twoKp 5.0f
#define twoKi 0.0f
#define MAG_X_ADJ 1.0625
#define MAG_Y_ADJ 1.0625
#define MAG_Z_ADJ 1.0469
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
void velocity_adjust(double delta_v[]);
void update_quaternion(double data[]);
double get_angular(double q_sub);
//current camera position.
double cx = 400.0f, cy = 400.0f, cz = 400.0f;
double lx = 0.0f, lz = 0.0f, ly = 0.0f;
double alpha = 45.0f;
double theta = 45.0f;
double r = 500;


//filter
double integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
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
double angle[3]={0,0,0};

//start read flag
bool flag_start=false;

//trash count


//read usb data and then calculate position.
void parse_data(string & buff,int n)
{
    //shitty design to fix usb shitty design
    buff=prev_buff+buff;
    n+=prev_buff.size();
    prev_buff="";
    string tmp_num="";
    int count=0;
    //sample time interval
    double t=1.00/FREQ;
    int data[20]={0};
    //loop data and parse value
    for (int i=0;i<n;i++)
    {
        cout <<buff[i];
        switch (buff[i])
        {
            case '\n':
            {
                //build new node and calculate position
                if (count !=6 && count!=9)
                {
                    count=0;
                    prev_buff="";
                    data[6]=data[7]=data[8]=0;
                    break;
                }
                node* temp = new node;
                //get current angle
                temp->next = NULL;
                tail->next = temp;
                tail = temp;
                prev_buff="";
                count=0;
                track_position(data,temp);
                data[6]=data[7]=data[8]=0;
                //cout << "build new node." << endl;
                //cout << "coordinate for new node are:" << temp->x << " " << temp->y << " " << temp->z << endl;
            }
            case ' ':
            {
                //get value
                if (tmp_num=="")
                    break;
                try
                {
                    cout<<"now count "<<count<<":"<<tmp_num<<endl;
                    data[count]=stoi(tmp_num);
                }
                catch (exception& e)
                {
                    cout <<e.what()<<endl;
                    cout <<"string is"<<tmp_num<<endl;
                }
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
    double temp_data[9]={0};
    //calculate angle
    /*
     angle[0]=get_angular(atan2(q[2]*q[3]+q[0]*q[1], 0.5f-(q[1]*q[1]+q[2]*q[2])));
     angle[1]=get_angular(asin(2.0f*(-q[1]*q[3]+q[0]*q[2])));
     angle[2]=get_angular(atan2(q[1]*q[2]+q[0]*q[3], 0.5f-(q[2]*q[2]+q[3]*q[3])));
     */
    
    angle[2]=get_angular(atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]));
    angle[1]=get_angular(asin(2.0f*(-q[1]*q[3]+q[0]*q[2])));
    angle[0]=get_angular(atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]));
    
    //matrix M
    //TODO: Test queternion update.
    double m[3][3]={
        {q[0]*q[0]+q[1]*q[1]-q[2]*q[2]-q[3]*q[3],2*q[1]*q[2]+2*q[0]*q[3],2*q[1]*q[3]-2*q[0]*q[2]},
        {2*q[1]*q[2]-2*q[0]*q[3],q[0]*q[0]-q[1]*q[1]+q[2]*q[2]-q[3]*q[3],2*q[2]*q[3]+2*q[0]*q[1]},
        {2*q[1]*q[3]+2*q[0]*q[2],2*q[2]*q[3]-2*q[0]*q[1],q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]}
    };
    
    //delta velocity
    double delta_v[3]={((double)AX)*2.00*9.81/32768.00,
        ((double)AY)*2.00*9.81/32768.00,
        ((double)AZ)*2.00*9.81/32768.00};
    
    double inertial_delta_v[3];
    //delta velocity under world frame
    inertial_delta_v[0]=(m[0][0]*delta_v[0]+m[0][1]*delta_v[1]+m[0][2]*delta_v[2])*t;
    inertial_delta_v[1]=(m[1][0]*delta_v[0]+m[1][1]*delta_v[1]+m[1][2]*delta_v[2])*t;
    inertial_delta_v[2]=(m[2][0]*delta_v[0]+m[2][1]*delta_v[1]+m[2][2]*delta_v[2]-GRAVITY_FIX)*t;
    cout <<"inertial delta velocity:"<<inertial_delta_v[0]<<" "<<inertial_delta_v[1]<<" "<<inertial_delta_v[2]<<endl;
    //inertial_delta_v[2]=0.0;
    //new velocity
    temp_data[0]=delta_v[0];
    temp_data[1]=delta_v[1];
    temp_data[2]=delta_v[2];
    for (int j = 0; j < 4; j++)
        data_buff[j] = data_buff[j + 1];
    data_buff[4] = to_string(inertial_delta_v[0]) + "," + to_string(inertial_delta_v[1]) + "," + to_string(inertial_delta_v[2]) + "\n";

    velocity_adjust(inertial_delta_v);
    v[0]+=inertial_delta_v[0];
    v[1]+=inertial_delta_v[1];
    v[2]+=inertial_delta_v[2];
    cout <<"velocity:"<<v[0]<<" "<<v[1]<<" "<<v[2]<<endl;
    //distance
    temp->x=v[0]*t*10.0;
    temp->y=v[1]*t*10.0;
    temp->z=v[2]*t*10.0;
    //update information array
    temp_data[3]=((double)GX)*250.00/32768.00/180.00*PI;
    temp_data[4]=((double)GY)*250.00/32768.00/180.00*PI;
    temp_data[5]=((double)GZ)*250.00/32768.00/180.00*PI;
    temp_data[6]=((double)MX)*4800.00/32768.00*MAG_X_ADJ;
    temp_data[7]=((double)MY)*4800.00/32768.00*MAG_Y_ADJ;
    temp_data[8]=((double)MZ)*4800.00/32768.00*MAG_Z_ADJ;
    update_quaternion(temp_data);
}

void velocity_adjust(double delta_v[])
{
    for (int i=0;i<3;i++)
    {
        if (delta_v[i]<ZERO_V_THRESHOLD && delta_v[i]>-ZERO_V_THRESHOLD)
            v[i]=0.00;
    }
}

//Use Newton iteration to calcuate 1/sqrt(). Should be fast.
double quick_normal(float sum)
{
    long i;
    float x2,y;
    const float threehalfs=1.5;
    
    x2=sum*0.5;
    y=sum;
    i=*(long *) &y;
    i=0x5f3759df - (i>>1);
    y=*(float *) &i;
    y*=(threehalfs-(x2*y*y));
    return (double)abs(y);
}

void update_quaternion(double data[])
{
    double t=1.00/FREQ;
    double recipNorm;
    double halfvx, halfvy, halfvz;
    double halfex, halfey, halfez;
    double norm;
    
    // Normalise accelerometer measurement
    cout <<"Acceleration: "<<AX<<" "<<AY<<" "<<AZ<<endl;
    recipNorm = quick_normal((float)(AX*AX+AY*AY+AZ*AZ));
    AX *= recipNorm;
    AY *= recipNorm;
    AZ *= recipNorm;
    
    double s1,s2,s3,s4;
    if (MX==0 && MY==0 && MZ==0)
    {
        s1 = 4.0f*q[0] * q[2]*q[2] + 2.0f*q[2] * AX + 4.0f*q[0] * q[1]*q[1] - 2.0f*q[1] * AY;
        s2 = 4.0f*q[1] * q[3]*q[3] - 2.0f*q[3] * AX + 4.0f * q[0]*q[0] * q[1] - 2.0f*q[0] * AY - 4.0f*q[1] + 8.0f*q[1] * q[1]*q[1] + 8.0f*q[1] * q[2]*q[2] + 4.0f*q[1] * AZ;
        s3 = 4.0f * q[0]*q[0] * q[2] + 2.0f*q[0] * AX + 4.0f*q[2] * q[3]*q[3] - 2.0f*q[3] * AY - 4.0f*q[2] + 8.0f*q[2] * q[1]*q[1] + 8.0f*q[2] * q[2]*q[2] + 4.0f*q[2] * AZ;
        s4 = 4.0f * q[1]*q[1] * q[3] - 2.0f*q[1] * AX + 4.0f * q[2]*q[2] * q[3] - 2*q[2] * AY;
    }
    else
    {
        norm = quick_normal(MX * MX + MY * MY + MZ * MZ);
        MX *= norm;
        MY *= norm;
        MZ *= norm;
        
        // Reference direction of Earth's magnetic field
        double _2q0mx = 2.0f * q[0] * MX;
        double _2q0my = 2.0f * q[0] * MY;
        double _2q0mz = 2.0f * q[0] * MZ;
        double _2q1mx = 2.0f * q[1] * MX;
        double hx = MX * q[0]*q[0] - _2q0my * q[3] + _2q0mz * q[2] + MX * q[1]*q[1] + 2.0f*q[1] * MY * q[2] + 2.0f*q[1] * MZ * q[3] - MX * q[2]*q[2] - MX * q[3]*q[3];
        double hy = _2q0mx * q[3] + MY * q[0]*q[0] - _2q0mz * q[1] + _2q1mx * q[2] - MY * q[1]*q[1] + MY * q[2]*q[2] + 2.0f*q[2] * MZ * q[3] - MY * q[3]*q[3];
        double _2bx = sqrt(hx * hx + hy * hy);
        double _2bz = -_2q0mx * q[2] + _2q0my * q[1] + MZ * q[0]*q[0] + _2q1mx * q[3] - MZ * q[1]*q[1] + 2.0f*q[2] * MY * q[3] - MZ * q[2]*q[2] + MZ * q[3]*q[3];
        double _4bx = 2.0f * _2bx;
        double _4bz = 2.0f * _2bz;
        
        // Gradient decent algorithm corrective step
        s1 = -2.0f*q[2] * (2.0f * q[1]*q[3] - 2.0f*q[0]*q[2] - AX) + 2.0f*q[1] * (2.0f * q[0]*q[1] + 2.0f*q[2]*q[3] - AY) - _2bz * q[2] * (_2bx * (0.5f - q[2]*q[2] - q[3]*q[3]) + _2bz * (q[1]*q[3] - q[0]*q[2]) - MX) + (-_2bx * q[3] + _2bz * q[1]) * (_2bx * (q[1]*q[2] - q[0]*q[3]) + _2bz * (q[0]*q[1] + q[2]*q[3]) - MY) + _2bx * q[2] * (_2bx * (q[0]*q[2] + q[1]*q[3]) + _2bz * (0.5f - q[1]*q[1] - q[2]*q[2]) - MZ);
        s2 = 2.0f*q[3] * (2.0f * q[1]*q[3] - 2.0f*q[0]*q[2] - AX) + 2.0f*q[0] * (2.0f * q[0]*q[1] + 2.0f*q[2]*q[3] - AY) - 4.0f * q[1] * (1.0f - 2.0f * q[1]*q[1] - 2.0f * q[2]*q[2] - AZ) + _2bz * q[3] * (_2bx * (0.5f - q[2]*q[2] - q[3]*q[3]) + _2bz * (q[1]*q[3] - q[0]*q[2]) - MX) + (_2bx * q[2] + _2bz * q[0]) * (_2bx * (q[1]*q[2] - q[0]*q[3]) + _2bz * (q[0]*q[1] + q[2]*q[3]) - MY) + (_2bx * q[3] - _4bz * q[1]) * (_2bx * (q[0]*q[2] + q[1]*q[3]) + _2bz * (0.5f - q[1]*q[1] - q[2]*q[2]) - MZ);
        s3 = -2.0f*q[0] * (2.0f * q[1]*q[3] - 2.0f*q[0]*q[2] - AX) + 2.0f*q[3] * (2.0f * q[0]*q[1] + 2.0f*q[2]*q[3] - AY) - 4.0f * q[2] * (1 - 2.0f * q[1]*q[1] - 2.0f * q[2]*q[2] - AZ) + (-_4bx * q[2] - _2bz * q[0]) * (_2bx * (0.5f - q[2]*q[2] - q[3]*q[3]) + _2bz * (q[1]*q[3] - q[0]*q[2]) - MX) + (_2bx * q[1] + _2bz * q[3]) * (_2bx * (q[1]*q[2] - q[0]*q[3]) + _2bz * (q[0]*q[1] + q[2]*q[3]) - MY) + (_2bx * q[0] - _4bz * q[2]) * (_2bx * (q[0]*q[2] + q[1]*q[3]) + _2bz * (0.5f - q[1]*q[1] - q[2]*q[2]) - MZ);
        s4 = 2.0f*q[1] * (2.0f * q[1]*q[3] - 2.0f*q[0]*q[2] - AX) + 2.0f*q[2] * (2.0f * q[0]*q[1] + 2.0f*q[2]*q[3] - AY) + (-_4bx * q[3] + _2bz * q[1]) * (_2bx * (0.5f - q[2]*q[2] - q[3]*q[3]) + _2bz * (q[1]*q[3] - q[0]*q[2]) - MX) + (-_2bx * q[0] + _2bz * q[2]) * (_2bx * (q[1]*q[2] - q[0]*q[3]) + _2bz * (q[0]*q[1] + q[2]*q[3]) - MY) + _2bx * q[1] * (_2bx * (q[0]*q[2] + q[1]*q[3]) + _2bz * (0.5f - q[1]*q[1] - q[2]*q[2]) - MZ);
    }
    
    norm = quick_normal((float)(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4));    // normalise step magnitude
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;
    
    double qDot1, qDot2, qDot3, qDot4;
    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q[1] * GX - q[2] * GY - q[3] * GZ) - BETA * s1;
    qDot2 = 0.5f * (q[0] * GX + q[2] * GZ - q[3] * GY) - BETA * s2;
    qDot3 = 0.5f * (q[0] * GY - q[1] * GZ + q[3] * GX) - BETA * s3;
    qDot4 = 0.5f * (q[0] * GZ + q[1] * GY - q[2] * GX) - BETA * s4;
    
    q[0] += qDot1 * t;
    q[1] += qDot2 * t;
    q[2] += qDot3 * t;
    q[3] += qDot4 * t;
    norm=quick_normal((float)(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]));
    q[0]*=norm;
    q[1]*=norm;
    q[2]*=norm;
    q[3]*=norm;
    cout <<"norm:"<<norm<<endl;
    cout <<"quaternion: "<<q[0]<<" "<<q[1]<<" "<<q[2]<<" "<<q[3]<<endl;
}

double get_angular(double q_sub)
{
    double sin_angle=q_sub*180.00/PI;
    return sin_angle;
}
