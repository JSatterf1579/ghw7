#include <stdio.h>
#include <stdlib.h>
#include <gl\glut.h>
#include <math.h>

#include "frame_buffer.h"
#include "primitives.h"
#include "color.h"
#include <vector>

#include <iostream>
#include <fstream>

#define ON 1
#define OFF 0
#define EPSILON 0.000001
#define M_PI 3.14159265358979323846

using namespace std;


// Global variables
int window_width, window_height;    // Window dimensions

const int INITIAL_RES = 400;

float * modelWorldMatrix;
FrameBuffer* fb;



class point
{
public:
	double x, y, z, w;

	point() { x = 0; y = 0; z = 0; w = 1; }
	point(double xa, double ya, double za)
	{
		x = xa; y = ya; z = za; w = 1.0;
	}
	point(double xa, double ya, double za, double wa)
	{
		x = xa; y = ya; z = za; w = wa;
	}
};

typedef struct _faceStruct {
	int v1, v2, v3;
	int n1, n2, n3;
} faceStruct;

typedef struct Mesh {
	int verts, faces, norms;    // Number of vertices, faces and normals in the system
	point *vertList, *normList; // Vertex and Normal Lists
	faceStruct *faceList;	    // Face List
  float ar, ag, ab, dr, dg, db, sr, sg, sb, k_a, k_d, s_exp, ref_index, k_relect, k_refract;  
};

typedef struct Light {
  int light_type;
  float x,y,z,r,g,b;
};

typedef struct Sphere {
  float radius;
  float x,y,z;
  float ar, ag, ab, dr, dg, db, sr, sg, sb, k_a, k_d, s_exp, ref_index, k_relect, k_refract;
};

typedef struct Ray {
	point origin, direction;
};


point* raySphereIntercept(Ray *r, Sphere *s);
void translateModelMatrix(float x, float y, float z);
void rotateModelMatrixX(float angle);
void rotateModelMatrixY(float angle);
void rotateModelMatrixZ(float angle);
void scaleModelMatrix(float scaleFactor);
point Transform(float* matrix, point p);
point* rayTriIntersection(Ray *r, point *v1, point *v2, point  *v3);


// The mesh reader itself
// It can read *very* simple obj files
void meshReader(char *filename, int sign, Mesh *mesh, float transformX, float transformY, float transformZ, float rotX, float rotY, float rotZ, float scale)
{
	//Build

	float x, y, z, len;
	int i;
	char letter;
	point v1, v2, crossP;
	int ix, iy, iz;
	int *normCount;
	FILE *fp;

	modelWorldMatrix = (float *)malloc(16 * sizeof(float));
	for (int i = 0; i < 16; i++) {
		if (i % 5 == 0) {
			modelWorldMatrix[i] = (float)1;
		}
		else {
			modelWorldMatrix[i] = (float)0;
		}
	}

	scaleModelMatrix(scale);
	rotateModelMatrixX(rotX);
	rotateModelMatrixY(rotY);
	rotateModelMatrixZ(rotZ);
	translateModelMatrix(transformX, transformY, transformZ);
	fp = fopen(filename, "r");
	if (fp == NULL) {
		printf("Cannot open %s\n!", filename);
		exit(0);
	}

	// Count the number of vertices and faces
	while (!feof(fp))
	{
		fscanf(fp, "%c %f %f %f\n", &letter, &x, &y, &z);
		if (letter == 'v')
		{
			mesh->verts++;
		}
		else
		{
			mesh->faces++;
		}
	}

	fclose(fp);

	printf("verts : %d\n", mesh->verts);
	printf("faces : %d\n", mesh->faces);

	// Dynamic allocation of vertex and face lists
	mesh->faceList = (faceStruct *)malloc(sizeof(faceStruct)*mesh->faces);
	mesh->vertList = (point *)malloc(sizeof(point)*mesh->verts);
	mesh->normList = (point *)malloc(sizeof(point)*mesh->verts);

	fp = fopen(filename, "r");

	// Read the veritces
	for (i = 0;i < mesh->verts;i++)
	{
		fscanf(fp, "%c %f %f %f\n", &letter, &x, &y, &z);
		mesh->vertList[i].x = x;
		mesh->vertList[i].y = y;
		mesh->vertList[i].z = z;
		mesh->vertList[i].w = 1.0;
		mesh->vertList[i] = Transform(modelWorldMatrix,mesh->vertList[i]);
	}

	// Read the faces
	for (i = 0;i < mesh->faces;i++)
	{
		fscanf(fp, "%c %d %d %d\n", &letter, &ix, &iy, &iz);
		mesh->faceList[i].v1 = ix - 1;
		mesh->faceList[i].v2 = iy - 1;
		mesh->faceList[i].v3 = iz - 1;
	}
	fclose(fp);


	// The part below calculates the normals of each vertex
	normCount = (int *)malloc(sizeof(int)*mesh->verts);
	for (i = 0;i < mesh->verts;i++)
	{
		mesh->normList[i].x = mesh->normList[i].y = mesh->normList[i].z = 0.0;
		normCount[i] = 0;
	}

	for (i = 0;i < mesh->faces;i++)
	{
		v1.x = mesh->vertList[mesh->faceList[i].v2].x - mesh->vertList[mesh->faceList[i].v1].x;
		v1.y = mesh->vertList[mesh->faceList[i].v2].y - mesh->vertList[mesh->faceList[i].v1].y;
		v1.z = mesh->vertList[mesh->faceList[i].v2].z - mesh->vertList[mesh->faceList[i].v1].z;
		v2.x = mesh->vertList[mesh->faceList[i].v3].x - mesh->vertList[mesh->faceList[i].v2].x;
		v2.y = mesh->vertList[mesh->faceList[i].v3].y - mesh->vertList[mesh->faceList[i].v2].y;
		v2.z = mesh->vertList[mesh->faceList[i].v3].z - mesh->vertList[mesh->faceList[i].v2].z;

		crossP.x = v1.y*v2.z - v1.z*v2.y;
		crossP.y = v1.z*v2.x - v1.x*v2.z;
		crossP.z = v1.x*v2.y - v1.y*v2.x;

		len = sqrt(crossP.x*crossP.x + crossP.y*crossP.y + crossP.z*crossP.z);

		crossP.x = -crossP.x / len;
		crossP.y = -crossP.y / len;
		crossP.z = -crossP.z / len;

		mesh->normList[mesh->faceList[i].v1].x = mesh->normList[mesh->faceList[i].v1].x + crossP.x;
		mesh->normList[mesh->faceList[i].v1].y = mesh->normList[mesh->faceList[i].v1].y + crossP.y;
		mesh->normList[mesh->faceList[i].v1].z = mesh->normList[mesh->faceList[i].v1].z + crossP.z;
		mesh->normList[mesh->faceList[i].v2].x = mesh->normList[mesh->faceList[i].v2].x + crossP.x;
		mesh->normList[mesh->faceList[i].v2].y = mesh->normList[mesh->faceList[i].v2].y + crossP.y;
		mesh->normList[mesh->faceList[i].v2].z = mesh->normList[mesh->faceList[i].v2].z + crossP.z;
		mesh->normList[mesh->faceList[i].v3].x = mesh->normList[mesh->faceList[i].v3].x + crossP.x;
		mesh->normList[mesh->faceList[i].v3].y = mesh->normList[mesh->faceList[i].v3].y + crossP.y;
		mesh->normList[mesh->faceList[i].v3].z = mesh->normList[mesh->faceList[i].v3].z + crossP.z;
		normCount[mesh->faceList[i].v1]++;
		normCount[mesh->faceList[i].v2]++;
		normCount[mesh->faceList[i].v3]++;
	}
	for (i = 0;i < mesh->verts;i++)
	{
		mesh->normList[i].x = (float)sign*mesh->normList[i].x / (float)normCount[i];
		mesh->normList[i].y = (float)sign*mesh->normList[i].y / (float)normCount[i];
		mesh->normList[i].z = (float)sign*mesh->normList[i].z / (float)normCount[i];
	}

	free(modelWorldMatrix);
}

void sceneReader(char * filename)
{
	FILE* pObjectFile = fopen(filename, "r");
	if(!pObjectFile)
		cout << "Failed to load " << filename << "." << endl;
	else
		cout << "Successfully loaded " << filename << "." << endl;

	struct matFile * mf = (struct matFile *)malloc(sizeof(struct matFile));
	while(!feof(pObjectFile))
	{
		char data [1000];
		fgets(data, "%c", pObjectFile);
		if(data[0] == 'L'){
			//We are reading in the definition of a light

			sscanf(data, "\n", );
			mf->par = par;
			mf->pag = pag;
			mf->pab = pab;
			mf->pdr = pdr;
			mf->pdg = pdg;
			mf->pdb = pdb;
			mf->psr = psr;
			mf->psg = psg;
			mf->psb = psb;
			mf->p_spec = n;
		}
		else if (data[0] == 'M')
		{
			//We are reading in a mesh definition
            float ar, ag, ab, dr, dg, db, sr, sg, sb, k_a, k_d, s_exp, ref_index, k_relect, k_refract;
            float ctar, ctag, ctab, ctd_val, ctrdr, ctrdg, ctrdb, cts_val, ctf0r, ctf0g, ctf0b, ctm1, ctw1, ctm2, ctw2;
			fscanf(pObjectFile,"%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",&ctar, &ctag, &ctab, &ctd_val, &ctrdr, &ctrdg, &ctrdb, &cts_val, &ctf0r, &ctf0g, &ctf0b, &ctm1, &ctw1, &ctm2, &ctw2);
			mf->ctar = ctar;
			mf->ctag = ctag;
			mf->ctab = ctab;
			mf->ctd_val = ctd_val;
			mf->ctrdr = ctrdr;
			mf->ctrdg = ctrdg;
			mf->ctrdb = ctrdb;
			mf->cts_val = cts_val;
			mf->ctf0r = ctf0r;
			mf->ctf0g = ctf0g;
			mf->ctf0b = ctf0b;
			mf->ctm1 = ctm1;
			mf->ctw1 = ctw1;
			mf->ctm2 = ctm2;
			mf->ctw2 = ctw2;
		} else if (data[0] == 'S'){
			//We are reading in a sphere definition.
            float ar, ag, ab, dr, dg, db, sr, sg, sb, k_a, k_d, s_exp, ref_index, k_relect, k_refract;
        }
	}
	return *mf;
}

void drawRect(double x, double y, double w, double h)
{
	glVertex2f(x, y);
	glVertex2f(x + w, y);
	glVertex2f(x + w, y + h);
	glVertex2f(x, y + h);
}

void translateModelMatrix(float x, float y, float z)
{
	float matrix[16] = { 1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,x,y,z,1.0 };
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(matrix);
	glMultMatrixf(modelWorldMatrix);
	glGetFloatv(GL_MODELVIEW_MATRIX, modelWorldMatrix);
}

void rotateModelMatrixX(float angle)
{
	float radians = angle * M_PI / 180.0;
	float matrix[16] = { 1,0,0,0,0,cos(radians),sin(radians),0,0,-sin(radians),cos(radians),0,0,0,0,1 };
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(modelWorldMatrix);
	glMultMatrixf(matrix);

	glGetFloatv(GL_MODELVIEW_MATRIX, modelWorldMatrix);
}

void rotateModelMatrixY(float angle)
{
	float radians = angle * M_PI / 180.0;
	float matrix[16] = { cos(radians),0,-sin(radians),0,0,1,0,0,sin(radians),0,cos(radians),0,0,0,0,1 };
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(modelWorldMatrix);
	glMultMatrixf(matrix);
	glGetFloatv(GL_MODELVIEW_MATRIX, modelWorldMatrix);
}

void rotateModelMatrixZ(float angle)
{
	float radians = angle * M_PI / 180.0;
	float matrix[16] = { cos(radians),sin(radians),0,0,-sin(radians),cos(radians),0,0,0,0,1,0,0,0,0,1 };
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(modelWorldMatrix);
	glMultMatrixf(matrix);
	glGetFloatv(GL_MODELVIEW_MATRIX, modelWorldMatrix);
}

void scaleModelMatrix(float scaleFactor)
{
	float matrix[16] = { scaleFactor,0,0,0,0,scaleFactor,0,0,0,0,scaleFactor,0,0,0,0,1 };
	glMatrixMode(GL_MODELVIEW);
	glLoadMatrixf(modelWorldMatrix);
	glMultMatrixf(matrix);
	glGetFloatv(GL_MODELVIEW_MATRIX, modelWorldMatrix);
}


// The display function. It is called whenever the window needs
// redrawing (ie: overlapping window moves, resize, maximize)
// You should redraw your polygons here
void	display(void)
{
	// Clear the background
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

	double w = 10 / double(fb->GetWidth());
	double h = 10 / double(fb->GetHeight());

	Color cl;
	glColor3f(0, 0, 1);

	glBegin(GL_QUADS);

	printf("width %d, height %d\n", fb->GetWidth(), fb->GetHeight());

	for (int y = 0; y < fb->GetHeight(); y++)
	{
		for (int x = 0; x < fb->GetHeight(); x++)
		{
			cl = fb->buffer[x][y].color;
			glColor3f(cl.r, cl.g, cl.b);

			drawRect(w*x, h*y, w, h);
		}
	}

	glEnd();
	glutSwapBuffers();
}


// This function is called whenever the window is resized. 
// Parameters are the new dimentions of the window
void	resize(int x, int y)
{
	glViewport(0, 0, x, y);
	window_width = x;
	window_height = y;

	printf("Resized to %d %d\n", x, y);
}


// This function is called whenever the mouse is pressed or released
// button is a number 0 to 2 designating the button
// state is 1 for release 0 for press event
// x and y are the location of the mouse (in window-relative coordinates)
void	mouseButton(int button, int state, int x, int y)
{
	;
}


//This function is called whenever the mouse is moved with a mouse button held down.
// x and y are the location of the mouse (in window-relative coordinates)
void	mouseMotion(int x, int y)
{
	;
}


// This function is called whenever there is a keyboard input
// key is the ASCII value of the key pressed
// x and y are the location of the mouse
void	keyboard(unsigned char key, int x, int y)
{
	switch (key) {
	case 'q':                           /* Quit */
		exit(1);
		break;
	case '-':
		fb->Resize(fb->GetHeight() / 2, fb->GetWidth() / 2);
		BresenhamLine(fb, fb->GetWidth()*0.1, fb->GetHeight()*0.1, fb->GetWidth()*0.9, fb->GetHeight()*0.9, Color(1, 0, 0));
		break;
	case '=':
		fb->Resize(fb->GetHeight() * 2, fb->GetWidth() * 2);
		BresenhamLine(fb, fb->GetWidth()*0.1, fb->GetHeight()*0.1, fb->GetWidth()*0.9, fb->GetHeight()*0.9, Color(1, 0, 0));
		break;
	default:
		break;
	}

	// Schedule a new display event
	glutPostRedisplay();
}


int main(int argc, char* argv[])
{

	fb = new FrameBuffer(INITIAL_RES, INITIAL_RES);

	BresenhamLine(fb, fb->GetWidth()*0.1, fb->GetHeight()*0.1, fb->GetWidth()*0.9, fb->GetHeight()*0.9, Color(1, 0, 0));



	// Initialize GLUT
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutCreateWindow("Raytracer");
	glutDisplayFunc(display);
	glutReshapeFunc(resize);
	glutMouseFunc(mouseButton);
	glutMotionFunc(mouseMotion);
	glutKeyboardFunc(keyboard);

	// Initialize GL
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, 10, 0, 10, -10000, 10000);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_POLYGON_SMOOTH);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_LINE_SMOOTH);

	// Switch to main loop
	glutMainLoop();
	return 0;
}


// Transform a point with an arbitrary matrix
point Transform(float* matrix, point p)
{
	point temp;
	temp.x = matrix[0] * p.x + matrix[4] * p.y + matrix[8] * p.z + matrix[12] * p.w;
	temp.y = matrix[1] * p.x + matrix[5] * p.y + matrix[9] * p.z + matrix[13] * p.w;
	temp.z = matrix[2] * p.x + matrix[6] * p.y + matrix[10] * p.z + matrix[14] * p.w;
	temp.w = matrix[3] * p.x + matrix[7] * p.y + matrix[11] * p.z + matrix[15] * p.w;
	return temp;

}


float Dot(point& v, point& vx)
{
	return (v.x * vx.x) + (v.y * vx.y) + (v.z * vx.z);
}

float magnitude(point p) {
	return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}

point multiply(point p, float m)
{
	point temp;
	temp.x = p.x * m;
	temp.y = p.y * m;
	temp.z = p.z * m;
	return temp;
}

point subtract(point p, point p2)
{
	point temp;
	temp.x = p.x - p2.x;
	temp.y = p.y - p2.y;
	temp.z = p.z - p2.z;
	temp.w = 0;
	return temp;
}

point cross(point p, point p2)
{
	point temp;
	temp.x = p.y * p2.z - p.z * p2.y;
	temp.y = p.z * p2.x - p.x * p2.z;
	temp.z = p.x * p2.y - p.y * p2.x;
	temp.w = 0;
	return temp;

}

point* raySphereIntercept(Ray *r, Sphere *s)
{
	point *ret = nullptr;
	point *dP = (point *)malloc(sizeof(point));
	dP->x = s->x - r->origin.x;
	dP->y = s->y - r->origin.y;
	dP->z = s->z - r->origin.z;
	dP->w = 0;

	float dotUdP = Dot(r->direction, *dP);
	point directionProject = multiply(r->direction, dotUdP);
	point diffdPDirectionProjection = subtract(*dP, directionProject);
	float mag = magnitude(diffdPDirectionProjection);
	float magsq = mag * mag;

	float discriminant = (s->radius * s->radius) - magsq;

	if (discriminant >= 0)
	{
		float sPos = Dot(r->direction, *dP) + sqrt(discriminant);
		float sNeg = Dot(r->direction, *dP) - sqrt(discriminant);
		float s;
		if (sPos < sNeg)
		{
			s = sPos;
		}
		else
		{
			s = sNeg;
		}
		ret = (point *)malloc(sizeof(point));
		ret->x = r->origin.x + s * r->direction.x;
		ret->y = r->origin.y + s * r->direction.y;
		ret->z = r->origin.z + s * r->direction.z;
		ret->w = 1;

	}

	return ret;
}


point* rayTriIntersection(Ray *r, point *v1, point *v2, point  *v3)
{
	point e1, e2;
	point P, Q, T;
	float det, invDet, u, v;
	float t;
	point *ret = nullptr;

	e1 = subtract(*v2, *v3);
	e2 = subtract(*v3, *v1);
	P = cross(r->direction, e2);
	det = Dot(e1, P);
	if (det > -EPSILON && det < EPSILON)
	{
		return ret;
	}
	invDet = 1.f / det;
	T = subtract(r->origin, *v1);
	u = Dot(T, P) * invDet;
	
	if (u < 0.f || u > 1.f)
	{
		return ret;
	}

	Q = cross(T, e1);
	v = Dot(r->direction, Q) * invDet;

	if (v < 0.f || v > 1.f)
	{
		return ret;
	}

	t = Dot(e2, Q) * invDet;

	if (t > EPSILON)
	{
		ret = (point *)malloc(sizeof(point));
		ret->x = r->origin.x + r->direction.x * t;
		ret->y = r->origin.y + r->direction.y * t;
		ret->z = r->origin.z + r->direction.z * t;
		ret->w = 0;
	}

	return ret;

}