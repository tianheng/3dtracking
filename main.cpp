#include <GLUT/glut.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <iostream>
#include <random>
#include "tracking.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace std;


int open_port()
{
  cout <<"open usb...."<<endl;
  int fd = open(USB_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fd<0)
    cout << "usb doens't work"<<endl;
  struct termios tty;
  cout <<"usb connected, setting usb port."<<endl;
  memset (&tty, 0, sizeof tty);
  tcgetattr(fd, &tty);
  cfsetospeed (&tty, B115200);
  cfsetispeed (&tty, B115200);
  
  tty.c_iflag &= ~IGNBRK;         
  tty.c_lflag = 0;
                  
  tty.c_oflag = 0; 
  tty.c_cc[VMIN]  = 0;
  tty.c_cc[VTIME] = 5;   
  
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); 
  
  tty.c_cflag |= (CLOCAL | CREAD);
  // enable reading, odd parity, 7bits. only for microcontroller. 
  // need recode for XBee module.
  tty.c_cflag |= PARENB;
  tty.c_cflag |= PARODD;
  tty.c_cflag |= CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS7;
  tcsetattr(fd, TCSANOW, &tty);

  //flush first 4096 bytes garbage data
  char temp_buff[4096];
  int n =read(fd, temp_buff, sizeof temp_buff);
  return fd;
}


//print info text to screen, need recode for parameters. Shitty design
void output_info(string info, int x, int y, string input_string)
{
	//store original matrix and then set matrix to identity
	glPushMatrix();
	glLoadIdentity();
	glOrtho(-1000.0, 1000.0, -1000.0, 1000.0, -1000.0, 1000.0);
	glColor3f(1.0f, 1.0f, 1.0f);
	string temp = info + input_string;
	//set text's beginning position
	glRasterPos2f(x, y);
	int curr_y=y;	
	//output text on screen.
	for (int i = 0; i < temp.size(); i++) {
		//check newline
		if (temp[i] == '\n')
			glRasterPos2f(x, curr_y -= 45);
		else
			glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, temp[i]);
	}
	glPopMatrix(); 
}

//main display function
void display(void)
{
	/*  clear all pixels  */
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glColor3f(1.0, 1.0, 1.0);
	glBegin(GL_LINE_STRIP);
		node* curr = START;
		double x = 0.0f, y = 0.0f, z = 0.0f;
		//iterate all points and draw points
   		while (curr != NULL)
		{
		  //ATTENTION. OpenGL's coordinate is different. 
			x += curr->y;
			y += curr->z;
			z += curr->x;
			glVertex3f(x,y,z);
			curr = curr->next;
		}
	// Done drawing points
	glEnd();


	//print coordinates.
	//x-axis(red)
	glBegin(GL_LINES);
		glColor3f(1.0f, 0.0f, 0.0f);
		glVertex3f(0.0f, 0.0f, 0.0f);
		glVertex3f(1000.0f, 0.0f, 0.0f);
	glEnd();

	//y-axis(green)
	glBegin(GL_LINES);
		glColor3f(0.0f, 1.0f, 0.0f);
		glVertex3f(0.0f, 0.0f, 0.0f);
		glVertex3f(0.0f, 1000.0f, 0.0f);
	glEnd();

	//z-axis(blue)
	glBegin(GL_LINES);
		glColor3f(0.0f, 0.0f, 1.0f);
		glVertex3f(0.0f, 0.0f, 0.0f);
		glVertex3f(0.0f, 0.0f, 1000.0f);
	glEnd();

	//read usb content. 
	char temp_buff[4096];
	int n =read(fd, temp_buff, sizeof temp_buff);
	//flush again. just in case.
	if (!flag_start)
	  {
	    n=read(fd,temp_buff, sizeof temp_buff);
	    flag_start=true;
	  }
	//use string to simplify process.
	string buff(temp_buff);
	//parse data and calcuate position.
	parse_data(buff,n);
	/*  don't wait!
	*  start processing buffered OpenGL routines
	*/
	//output information on screen. 
	output_info("Current camera position:\n", 0, 500, to_string(cx).substr(0,6) + "," + to_string(cy).substr(0,6) + "," + to_string(cz).substr(0,6));
	output_info("Node added:\n", -900, 900, data_buff[0] + data_buff[1] + data_buff[2] + data_buff[3] + data_buff[4]);
	output_info("Current angle:\n",-900,500, to_string(angle[0]).substr(0,5) + "," + to_string(angle[1]).substr(0,5) + "," + to_string(angle[2]).substr(0,5));
	output_info("Current speed:\n",-900,400, to_string(v[0]).substr(0,5) + "," + to_string(v[1]).substr(0,5) + "," + to_string(v[2]).substr(0,5));
	glutSwapBuffers();
}

//key interrupt handler
void processspace(unsigned char key, int xx, int yy)
{
	double fraction = 0.1f;

	//random generate lines, merely for test purpose.
	/*if (key == ' ')
	{
		node* temp = new node;
		temp->x = rand() % 50 - 25;
		temp->y = rand() % 50 - 25;
		temp->z = rand() % 50 - 25;
		temp->next = NULL;
		tail->next = temp;
		tail = temp;
		cout << "build new node." << endl;
		cout << "coordinate for new node are:" << temp->x << " " << temp->y << " " << temp->z << endl;
		for (int i = 0; i < 4; i++)
			data_buff[i] = data_buff[i + 1];
		data_buff[4] = to_string(temp->x) + "," + to_string(temp->y) + "," + to_string(temp->z) + "\n";
		}*/

	//change view
	switch (key) {
	case 'a':
		cx -= 10.0f;
		break;
	case 'd':
		cx += 10.0f;
		break;
	case 'w':
		cy += 10.0f;
		break;
	case 's':
		cy -= 10.0f;
		break;
	}

	//change view matrix
	glLoadIdentity();
	glOrtho(-1000.0, 1000.0, -1000.0, 1000.0, -1000.0, 1000.0);
	gluLookAt(cx, cy, cz,
		0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f);
}

//init function
void init(void)
{
	/*  select clearing (background) color       */
	glClearColor(0.0, 0.0, 0.0, 0.0);
	/*  initialize viewing values  */
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glMatrixMode(GL_MODELVIEW);
	//set coordinate size
	glOrtho(-1000.0, 1000.0, -1000.0, 1000.0, -1000.0, 1000.0);

	//set camera position
	gluLookAt(400.0f, 400.0f, 400.0f,
		0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f);
}


//function for sync clock
void runMainLoop(int val)
{
	//Frame logic
	display();
	//Run frame one more time
	glutTimerFunc(1000 / 30, runMainLoop, val);
}



/*
*  Declare initial window size, position, and display mode
*  (single buffer and RGBA).  Open window with "hello"
*  in its title bar.  Call initialization routines.
*  Register callback function to display graphics.
*  Enter main loop and process events.
*/
int main(int argc, char** argv)
{
  //set for random tests
	srand(time(NULL));
	start.x = 0.0f;
	start.y = 0.0f;
	start.z = 0.0f;
	start.next = NULL;
	tail = START;
	//open usb port
	fd=open_port();
	//init OpenGL window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
	glutInitWindowSize(1000, 1000);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("ece445-3D drawing");
	init();
	glutDisplayFunc(display);
	glutKeyboardFunc(processspace);
	//glutIdleFunc(display);
	glutTimerFunc(1000 / 30, runMainLoop, 0);
	glEnable(GL_DEPTH_TEST);
	glutMainLoop();
	return 0;   /* ISO C requires main to return int. */
}
