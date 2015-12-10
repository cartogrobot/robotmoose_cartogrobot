// hw1.cpp
// Daniel A. Bross
// 17 Sep 2013
//Some code is borrowed from the anim.cpp program written in class
//simple demonstration of primitives and transformations in glut

//OpenGL and GLUT includes
#include <cstdlib>
using std::exit;
#ifndef __APPLE__
# include <GL/glut.h>
#else
# include <GLUT/glut.h>
#endif

//Extra includes
#include "lib381/bitmapprinter.h"

#include <iostream>
#include <vector>
using std::cerr;
using std::endl;


//globals
//keyboard
const int ESCKEY = 27;         // ASCII value of Escape

//window settings
const int startwinsize = 600;  // Start window width & height (pixels)

//objects
double savetime;               // Time of previous movement (sec)

//draws a filled square
void drawGrid(const OccupancyGrid & map)
{
	glPushMatrix();
    glBegin(GL_QUADS);
   	for(std::size_t y = map.size(); y > 0; y--)
	{
		for(std::size_t x = 0; x < map.size(); x++)
		{
			glColor3f(0.0, map(x,y), 1.0-map(x,y));
			glVertex2d(-1.0 + x, -1.0 + y);
		    glVertex2d( 1.0 + x, -1.0 + y);
		    glVertex2d( 1.0 + x,  1.0 + y);
		    glVertex2d(-1.0 + x,  1.0 + y);
		}
	}
    glEnd();
    glPopMatrix();
}


// myDisplay
// The GLUT display function
void myDisplay()
{
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT);

    // Draw objects
    // Initial transformation
    glLoadIdentity();

    //hollow square
    glPushMatrix();
    glTranslated(-1.0, -1.0, 0.);
    glScaled(1.0/(double)map.size(), 1.0/(double)map.size(), 0.0);
    drawGrid(map);
    glPopMatrix();

    // Draw documentation
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);  // Set up simple ortho projection
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(-1., 1., -1., 1.);
    glColor3d(1., 1., 1.);        // Black text
    BitmapPrinter p(-0.9, 0.9, 0.1);
    p.print("Esc      Quit");
    glPopMatrix();                // Restore prev projection
    glMatrixMode(GL_MODELVIEW);

    glutSwapBuffers();
}


//idle function
void myIdle()
{
    //timing
    double currtime = glutGet(GLUT_ELAPSED_TIME) / 1000.;
    double elapsedtime = currtime - savetime;
    savetime = currtime;
    if (elapsedtime > 0.1)
        elapsedtime = 0.1;

	//glutPostRedisplay();
}


//GLUT keyboard function
void myKeyboard(unsigned char key, int x, int y)
{
    switch (key)
    {
    case ESCKEY:  //esc: quit
        exit(0);
        break;
    }
}


//initialize GL states and global data
void init()
{
	int * i;
	char ** c;
	//init opengl
    glutInit(i, c);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);

    //generate window
    glutInitWindowSize(startwinsize, startwinsize);
    glutInitWindowPosition(50, 50);
    glutCreateWindow("Robot Mapping");	
	
    //timing
    savetime = glutGet(GLUT_ELAPSED_TIME) / 1000.;

    //transformation stuff
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluOrtho2D(-1., 1., -1., 1.);

    //return to model/view mode
    glMatrixMode(GL_MODELVIEW);
    
    glutDisplayFunc(myDisplay);
    glutIdleFunc(myIdle);
    glutKeyboardFunc(myKeyboard);

    //the loop
    glutMainLoop();
}
