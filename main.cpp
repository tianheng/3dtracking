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

float difamb[] = { 1.0, 0.5, 0.3, 1.0 };
int main_x = 2000;	//size of main window
int main_y = 1800;
float R = 10.0;

int mainWindow, subWindow1, subWindow2, subWindow3, subWindow4;
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
  // currently using XBee module setting.
  tty.c_cflag &= ~PARENB;
  //tty.c_cflag |= PARNONE;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
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

//mainWindow prints Info
void printInfo() {
    glutSetWindow(mainWindow);
    glClear(GL_COLOR_BUFFER_BIT);
    
    output_info("Current camera position:\n", -900, 600, to_string(alpha).substr(0, 6) + "," + to_string(theta).substr(0, 6) + "," + to_string(r).substr(0, 6));
    output_info("Node added:\n", -900, 900, data_buff[0] + data_buff[1] + data_buff[2] + data_buff[3] + data_buff[4]);
    output_info("Current angle:\n", -900, 500, to_string(angle[0]).substr(0, 5) + "," + to_string(angle[1]).substr(0, 5) + "," + to_string(angle[2]).substr(0, 5));
    output_info("Current speed:\n", -900, 400, to_string(v[0]).substr(0, 5) + "," + to_string(v[1]).substr(0, 5) + "," + to_string(v[2]).substr(0, 5));
    glutSwapBuffers();
}

void polortoxyz(double r, double theta, double alpha,
                double* x, double* y, double* z) {
    theta = theta / 180.0f * PI;
    alpha = alpha / 180.0f * PI;
    *z = r*sin(theta);
    *y = r*cos(theta)*sin(alpha);
    *x = r*cos(theta)*cos(alpha);
}

void renderCylinder(float x1, float y1, float z1, float x2, float y2, float z2, float radius, int subdivisions, GLUquadricObj *quadric)
{
    float vx = x2 - x1;
    float vy = y2 - y1;
    float vz = z2 - z1;
    float v = sqrt(vx*vx + vy*vy + vz*vz);
    double ax = 0.0;
    double zero = 1.0e-3;
    if (fabs(vz) < zero)
    {
        ax = 57.2957795*acos(vx / v); // rotation angle in x-y plane
        if (vx <= 0.0) ax = -ax;
    }
    else
    {
        ax = 57.2957795*acos(vz / v); // rotation angle
        if (vz <= 0.0) ax = -ax;
    }
    float rx = -vy*vz;
    float ry = vx*vz;
    glPushMatrix();
    //draw the cylinder body
    glTranslatef(x1, y1, z1);
    if (fabs(vz) < zero)
    {
        glRotated(90.0, 0, 1, 0.0); // Rotate & align with x axis
        glRotated(ax, -1.0, 0.0, 0.0); // Rotate to point 2 in x-y plane
    }
    else
    {
        glRotated(ax, rx, ry, 0.0); // Rotate about rotation vector
    }
    glColor3f(0.5f, 1.0f, 0.0f);
    gluQuadricOrientation(quadric, GLU_OUTSIDE);
    gluCylinder(quadric, radius, radius, v, subdivisions, 1);
    
    //draw the first cap
    glColor3f(0.5f, 0.5f, 0.5f);
    gluQuadricOrientation(quadric, GLU_INSIDE);
    gluDisk(quadric, 0.0, radius, subdivisions, 1);
    glTranslatef(0, 0, v);
    
    //draw the second cap
    glColor3f(5.0f, 8.0f, 4.0f);
    gluQuadricOrientation(quadric, GLU_OUTSIDE);
    gluDisk(quadric, 0.0, radius, subdivisions, 1);
    glPopMatrix();
}
void renderCylinder_convenient(float x1, float y1, float z1, float x2, float y2, float z2, float radius, int subdivisions)
{
    /*	if (x1 == x2 && y1 == y2 && z1 == z2){
     return;
     }*/
    //the same quadric can be re-used for drawing many cylinders
    GLUquadricObj *quadric = gluNewQuadric();
    gluQuadricNormals(quadric, GLU_SMOOTH);
    renderCylinder(x1, y1, z1, x2, y2, z2, radius, subdivisions, quadric);
    gluDeleteQuadric(quadric);
}


//main display function
void renderScene()
{
    /*  clear all pixels  */
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    float pos[] = { 0.0, 0.0, 800.0, 1.0 };
    glLightfv(GL_LIGHT0, GL_POSITION, pos);
    float dif[] = { 1.0, 1.0, 1.0, 1.0 };
    glLightfv(GL_LIGHT0, GL_DIFFUSE, dif);
    float amb[] = { 0.5, 0.5, 0.5, 1.0 };
    glLightfv(GL_LIGHT0, GL_AMBIENT, amb);
    
    
    glColor3f(1.0, 1.0, 1.0);
    glBegin(GL_LINE_STRIP);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, difamb);
    
    node* curr = START;
    double x = 0.0f, y = 0.0f, z = 0.0f;
    double prev_x = 0.0f, prev_y = 0.0f, prev_z = 0.0f;
    //iterate all points and draw points
    while (curr != NULL)
    {
        x += curr->x;
        y += curr->y;
        z += curr->z;
        renderCylinder_convenient(prev_x, prev_y, prev_z, x, y, z, R , 500);
        prev_x = x;
        prev_y = y;
        prev_z = z;
        curr = curr->next;
    }
    // Done drawing points
    //	glEnd();
    
    /*testing*/
   /* glBegin(GL_POLYGON);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, difamb);
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(1000.0f, 0.0f, 0.0f);
    glVertex3f(1000.0f, 1000.0f, 0.0f);
    glVertex3f(0.0f, 1000.0f, 0.0f);
    glEnd();*/
    
    //print coordinates.
    //x-axis(red)
    glBegin(GL_LINES);
    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, difamb);
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
    
    //draw the cylinder body new way
    /*glPushMatrix();
    glTranslatef(0.0, 0.0, 0.0);
    glRotatef(90.0, 0.0, 0.0, 1.0);
    glBegin(GL_QUAD_STRIP);
    for (int j = 0; j < 360; j += 1){
        glColor3f(0.0, 1.0, 1.0);
        glVertex3f(cos(j)*500.0, +100.0, sin(j)*500.0);
        glColor3f(0.0, 1.0, 0.0);
        glVertex3f(cos(j)*500.0, -100.0, sin(j)*500.0);
    }
    glEnd();
    glPopMatrix();
    
    renderCylinder_convenient(0.0, 0.0, 50.0, 250.0, 250.0, 50.0, 25.0, 500);
    renderCylinder_convenient(250.0, 250.0, 50.0, 250.0, 250.0, 500.0, 25.0, 500);
    renderCylinder_convenient(250.0, 250.0, 500.0, 400.0, 500.0, 600.0, 25.0, 500);
    renderCylinder_convenient(400.0, 500.0, 600.0, 100.0, 500.0, 100.0, 25.0, 500);*/
    
    //read usb content.
    char temp_buff[4096];
    int n = read(fd, temp_buff, sizeof temp_buff);
    //flush again. just in case.
    if (!flag_start)
    {
        n = read(fd, temp_buff, sizeof temp_buff);
        flag_start = true;
    }
    //use string to simplify process.
    string buff(temp_buff);
    
    //parse data and calcuate position.
    parse_data(buff, n);
    /*  don't wait!
     *  start processing buffered OpenGL routines
     */
    glutSwapBuffers();
}

// main display function
void displayW1(void)
{
    double eyeX, eyeY, eyeZ;
    glutSetWindow(subWindow1);
    glLoadIdentity();
    glEnable(GL_DEPTH_TEST);
    glOrtho(-1000.0, 1000.0, -1000.0, 1000.0, -1000.0, 1000.0);
    polortoxyz(r, theta, alpha, &eyeX, &eyeY, &eyeZ);
    gluLookAt(eyeX, eyeY, eyeZ,
              0.0f, 0.0f, 0.0f,
              0.0f, -1.0f*eyeX, 0.0f);
    renderScene();
}

void displayW2(void)
{
    double eyeX, eyeY, eyeZ;
    glutSetWindow(subWindow2);
    glLoadIdentity();
    glEnable(GL_DEPTH_TEST);
    glOrtho(-1000.0, 1000.0, -1000.0, 1000.0, -1000.0, 1000.0);
    polortoxyz(r, theta, alpha+90.0, &eyeX, &eyeY, &eyeZ);
    gluLookAt(eyeX, eyeY, eyeZ,
              0.0f, 0.0f, 0.0f,
              0.0f, 0.0f, -1.0f);
    renderScene();
}

void displayW3(void)
{
    double eyeX, eyeY, eyeZ;
    glutSetWindow(subWindow3);
    glLoadIdentity();
    glEnable(GL_DEPTH_TEST);
    glOrtho(-1000.0, 1000.0, -1000.0, 1000.0, -1000.0, 1000.0);
    polortoxyz(r, theta, alpha + 180.0, &eyeX, &eyeY, &eyeZ);
    gluLookAt(eyeX, eyeY, eyeZ,
              0.0f, 0.0f, 0.0f,
              0.0f, -1.0f*eyeX, 0.0f);
    renderScene();
}

void displayW4(void)
{
    double eyeX, eyeY, eyeZ;
    glutSetWindow(subWindow4);
    glLoadIdentity();
    glEnable(GL_DEPTH_TEST);
    glOrtho(-1000.0, 1000.0, -1000.0, 1000.0, -1000.0, 1000.0);
    polortoxyz(r, theta, alpha + 270.0, &eyeX, &eyeY, &eyeZ);
    gluLookAt(eyeX, eyeY, eyeZ,
              0.0f, 0.0f, 0.0f,
              0.0f, 0.0f, 1.0f);
    renderScene();
}

//key interrupt handler
void processspace(unsigned char key, int xx, int yy)
{
    double fraction = 0.1f;
    
    //random generate lines, merely for test purpose.
    if (key == ' ')
    {
        node* temp = new node;
        temp->x = rand() % 500 - 250;
        temp->y = rand() % 500 - 250;
        temp->z = rand() % 500 - 250;
        temp->next = NULL;
        tail->next = temp;
        tail = temp;
        cout << "build new node." << endl;
        cout << "coordinate for new node are:" << temp->x << " " << temp->y << " " << temp->z << endl;
        for (int i = 0; i < 4; i++)
            data_buff[i] = data_buff[i + 1];
        data_buff[4] = to_string(temp->x) + "," + to_string(temp->y) + "," + to_string(temp->z) + "\n";
    }
    
    //change view
    switch (key) {
        case 'a':
            alpha = fmod((alpha - 2.0f), 360.0f);
            break;
        case 'd':
            alpha = fmod((alpha + 2.0f) , 360.0f);
            break;
        case 'w':
            theta = fmod((theta + 2.0f) , 360.0f);
            break;
        case 's':
            theta = fmod((theta - 2.0f) , 360.0f);
            break;
        case 'r'://reset position to init
            alpha = 45.0;
            theta = 45.0;
            r = 500.0;
            break;
        case 'f'://set position for 3D display
            alpha = 45.0;
            theta = 0.0;
            r = 500.0;
            break;
    }
}

void renderAll()
{
    printInfo();
    displayW1();
    displayW2();
    displayW3();
    displayW4();
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
	gluLookAt(0.0f, 100.0f, 0.0f,
		0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_COLOR_MATERIAL);
}


//function for sync clock
void runMainLoop(int val)
{
	//Frame logic
    renderAll();
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
    cout << "Welcome to 3D fingertip drawing system!\n";
    //init OpenGL window
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA);
    glutInitWindowSize(main_x, main_y);
    glutInitWindowPosition(0, 0);
    mainWindow = glutCreateWindow("ece445-3D drawing");
    glutDisplayFunc(printInfo);
    //glutKeyboardFunc(processspace);
    init();
    //glutFullScreen();
    int winD=300;
    int sub1_x = 200;
    int sub1_y = winD;
    //-2-
    //1-3
    //-4-
    subWindow1 = glutCreateSubWindow(mainWindow, sub1_x, sub1_y, winD, winD);
    glutDisplayFunc(displayW1);
    glutKeyboardFunc(processspace);
    init();
    subWindow2 = glutCreateSubWindow(mainWindow, sub1_x+winD, 0, winD, winD);
    glutDisplayFunc(displayW2);
    glutKeyboardFunc(processspace);
    init();
    subWindow3 = glutCreateSubWindow(mainWindow, sub1_x + 2 * winD, sub1_y, winD, winD);
    glutDisplayFunc(displayW3);
    glutKeyboardFunc(processspace);
    init();
    subWindow4 = glutCreateSubWindow(mainWindow, sub1_x + winD, 2 * winD, winD, winD);
    glutDisplayFunc(displayW4);
    glutKeyboardFunc(processspace);
    init();
    //glutDisplayFunc(display);
    glutIdleFunc(renderAll);
    glutTimerFunc(1000 / 30, runMainLoop, 0);
    glEnable(GL_DEPTH_TEST);
    glutMainLoop();
    return 0;  /* ISO C requires main to return int. */
}
