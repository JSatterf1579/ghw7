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
#include <sstream>

#define ON 1
#define OFF 0
#define EPSILON 0.000001
#define M_PI 3.14159265358979323846

using namespace std;


// Global variables
int window_width, window_height;    // Window dimensions
int numSpheres, numMeshes, numLights;
float imagePlaneDistance = 8.f;
float imageplaneHalfSide = 5.f;

const int INITIAL_RES = 256;

float *modelWorldMatrix;
FrameBuffer *fb;

class point {
public:
    double x, y, z, w;

    point() {
        x = 0;
        y = 0;
        z = 0;
        w = 1;
    }

    point(double xa, double ya, double za) {
        x = xa;
        y = ya;
        z = za;
        w = 1.0;
    }

    point(double xa, double ya, double za, double wa) {
        x = xa;
        y = ya;
        z = za;
        w = wa;
    }

    point *copy() {
        return new point(x, y, z, w);
    }

    point operator*(double num)
    {
        return point(x * num, y * num, z * num);
    }
    point operator+(point c)
    {
        return point(c.x + x, c.y + y, c.z + z);
    }
    point operator-(point c)
    {
        return point(x - c.x, y - c.y, z - c.z);
    }
};

typedef struct _faceStruct {
    int v1, v2, v3;
    int n1, n2, n3;
} faceStruct;

typedef struct _Material {
    float ar, ag, ab, dr, dg, db, sr, sg, sb, k_a, k_d, k_s, s_exp, ref_index, k_relect, k_refract;
} Material;

typedef struct _Mesh {
    int verts, faces, norms;    // Number of vertices, faces and normals in the system
    point *vertList, *normList; // Vertex and Normal Lists
    faceStruct *faceList;        // Face List
    Material *material;
} Mesh;

typedef struct _Light {
    int light_type;
    float x, y, z, r, g, b;
} Light;

typedef struct _Sphere {
    float radius;
    float x, y, z;
    Material *material;
} Sphere;

Sphere *spheres;
Mesh *meshes;
Light *lights;

typedef struct _Ray {
    point origin, direction;
	bool isExternal;
} Ray;


point *raySphereIntercept(Ray *r, Sphere *s);

void translateModelMatrix(float x, float y, float z);

void rotateModelMatrixX(float angle);

void rotateModelMatrixY(float angle);

void rotateModelMatrixZ(float angle);

void scaleModelMatrix(float scaleFactor);

point Transform(float *matrix, point p);

point *rayTriIntersection(Ray *r, point *v1, point *v2, point *v3);

Color *localIllumination(point p, point norm, point view, Material mat);

float triArea(point v1, point v2, point v3);

point getTriNormal(point *p, Mesh *m, int i);

point getSphereNormal(point *p, Sphere *s);

point normalize(point pt);

Color rayTrace(Ray *r, int depth);

void raycast(Ray* r, Material*& mat, point*& norm, point*& closestPoint);

point subtract(point p, point p2);


// The mesh reader itself
// It can read *very* simple obj files
void meshReader(char *filename, int sign, Mesh *mesh, float transformX, float transformY, float transformZ, float rotX,
                float rotY, float rotZ, float scale) {
    //Build

    float x, y, z, len;
    char letter;
    point v1, v2, crossP;
    int ix, iy, iz;
    int *normCount;
    FILE *fp;

    modelWorldMatrix = (float *) malloc(16 * sizeof(float));
    for (int i = 0; i < 16; i++) {
        if (i % 5 == 0) {
            modelWorldMatrix[i] = (float) 1;
        }
        else {
            modelWorldMatrix[i] = (float) 0;
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
    mesh->verts = 0;
    mesh->faces = 0;
    // Count the number of vertices and faces
    while (!feof(fp)) {
        fscanf(fp, "%c %f %f %f\n", &letter, &x, &y, &z);
        if (letter == 'v') {
            mesh->verts++;
        }
        else {
            mesh->faces++;
        }
    }

    fclose(fp);

    printf("verts : %d\n", mesh->verts);
    printf("faces : %d\n", mesh->faces);

    // Dynamic allocation of vertex and face lists
    mesh->faceList = (faceStruct *) malloc(sizeof(faceStruct) * mesh->faces);
    mesh->vertList = (point *) malloc(sizeof(point) * mesh->verts);
    mesh->normList = (point *) malloc(sizeof(point) * mesh->verts);

    fp = fopen(filename, "r");

    // Read the veritces
    for (int i = 0; i < mesh->verts; i++) {
        fscanf(fp, "%c %f %f %f\n", &letter, &x, &y, &z);
        mesh->vertList[i].x = x;
        mesh->vertList[i].y = y;
        mesh->vertList[i].z = z;
        mesh->vertList[i].w = 1.0;
        mesh->vertList[i] = Transform(modelWorldMatrix, mesh->vertList[i]);
    }

    // Read the faces
    for (int i = 0; i < mesh->faces; i++) {
        fscanf(fp, "%c %d %d %d\n", &letter, &ix, &iy, &iz);
        mesh->faceList[i].v1 = ix - 1;
        mesh->faceList[i].v2 = iy - 1;
        mesh->faceList[i].v3 = iz - 1;
		mesh->faceList[i].n1 = ix - 1;
		mesh->faceList[i].n2 = iy - 1;
		mesh->faceList[i].n3 = iz - 1;
    }
    fclose(fp);


    // The part below calculates the normals of each vertex
    normCount = (int *) malloc(sizeof(int) * mesh->verts);
    for (int i = 0; i < mesh->verts; i++) {
        mesh->normList[i].x = mesh->normList[i].y = mesh->normList[i].z = 0.0;
        normCount[i] = 0;
    }

    for (int i = 0; i < mesh->faces; i++) {
        v1.x = mesh->vertList[mesh->faceList[i].v2].x - mesh->vertList[mesh->faceList[i].v1].x;
        v1.y = mesh->vertList[mesh->faceList[i].v2].y - mesh->vertList[mesh->faceList[i].v1].y;
        v1.z = mesh->vertList[mesh->faceList[i].v2].z - mesh->vertList[mesh->faceList[i].v1].z;
        v2.x = mesh->vertList[mesh->faceList[i].v3].x - mesh->vertList[mesh->faceList[i].v2].x;
        v2.y = mesh->vertList[mesh->faceList[i].v3].y - mesh->vertList[mesh->faceList[i].v2].y;
        v2.z = mesh->vertList[mesh->faceList[i].v3].z - mesh->vertList[mesh->faceList[i].v2].z;

        crossP.x = v1.y * v2.z - v1.z * v2.y;
        crossP.y = v1.z * v2.x - v1.x * v2.z;
        crossP.z = v1.x * v2.y - v1.y * v2.x;

        len = sqrt(crossP.x * crossP.x + crossP.y * crossP.y + crossP.z * crossP.z);

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
    for (int i = 0; i < mesh->verts; i++) {
        mesh->normList[i].x = (float) sign * mesh->normList[i].x / (float) normCount[i];
        mesh->normList[i].y = (float) sign * mesh->normList[i].y / (float) normCount[i];
        mesh->normList[i].z = (float) sign * mesh->normList[i].z / (float) normCount[i];
    }

    free(modelWorldMatrix);
}

void sceneReader(char *filename) {
    std::ifstream file(filename);
    std::string str;
    std::getline(file, str);
    stringstream ss(str);
    ss >> numLights;
    ss >> numSpheres;
    ss >> numMeshes;

    spheres = new Sphere[numSpheres];
    meshes = new Mesh[numMeshes];
    lights = new Light[numLights];

    int seenLights = 0, seenSpheres = 0, seenMeshes = 0;

    while (std::getline(file, str)) {
        stringstream ss(str);
        char lineType;
        ss >> lineType;

        if (lineType == 'L' && seenLights < numLights) {

            //Process the rest of the line as if it is a light definition
            Light *l = (Light *) malloc(sizeof(Light));
            ss >> l->light_type;
            ss >> l->x;
            ss >> l->y;
            ss >> l->z;
            ss >> l->r;
            ss >> l->g;
            ss >> l->b;
            lights[seenLights] = *l;
            seenLights++;
            std::string type = (l->light_type == 0) ? "Directional" : "Point";
            cout << "We found a " << type << " Light at position (" << l->x << ", " << l->y << ", " << l->z <<
            ") with color (" << l->r << ", " << l->g << ", " << l->b << ").\n";
        } else if (lineType == 'S' && seenSpheres < numSpheres) {
            //Process the rest of the line as if it is a sphere definition
            Sphere *s = (Sphere *) malloc(sizeof(Sphere));
            Material *mat = (Material *) malloc(sizeof(Material));
            s->material = mat;

            ss >> s->x;
            ss >> s->y;
            ss >> s->z;
            ss >> s->radius;

            ss >> mat->ar;
            ss >> mat->ag;
            ss >> mat->ab;

            ss >> mat->dr;
            ss >> mat->dg;
            ss >> mat->db;

            ss >> mat->sr;
            ss >> mat->sg;
            ss >> mat->sb;

            ss >> mat->k_a;
            ss >> mat->k_d;
            ss >> mat->k_s;

            ss >> mat->s_exp;
            ss >> mat->ref_index;
            ss >> mat->k_relect;
            ss >> mat->k_refract;

            spheres[seenSpheres] = *s;
            seenSpheres++;

            cout << "We found a sphere located at (" << s->x << ", " << s->y << ", " << s->z << ") with radius " <<
            s->radius << "\n";
            cout << "The material it uses has properties: RGB Ambient " << mat->ar << "," << mat->ag << "," <<
            mat->ab <<
            " RGB Diffuse " << mat->dr << "," << mat->dg << "," << mat->db << " RGB Specular " << mat->sr << "," <<
            mat->sg << "," << mat->sb << "\n\t Contants (Ambient, Diffuse, Specular): " << mat->k_a << " " <<
            mat->k_d << " " << mat->k_s << "\n\t Specular Exponent: " << mat->s_exp << " Index Refraction " <<
            mat->ref_index << " Constant Reflection: " << mat->k_relect << " Constant Refraction " << mat->k_refract;
            //Process the rest of the line as if it is the light definition
        } else if (lineType == 'M' && seenMeshes < numMeshes) {
            //Process the rest of the line as if it is a mesh definition
            string meshName;
            ss >> meshName;
            float scale, rotX, rotY, rotZ, x, y, z;

            Mesh *mesh = (Mesh *) malloc(sizeof(Mesh));
            Material *mat = (Material *) malloc(sizeof(Material));
            mesh->material = mat;

            ss >> scale;
            ss >> rotX;
            ss >> rotY;
            ss >> rotZ;
            ss >> x;
            ss >> y;
            ss >> z;

            cout << "Attempting to read in mesh: " << meshName << "\n";
            char *temp = strdup(meshName.c_str());
            meshReader(temp, 1, mesh, x, y, z, rotX, rotY, rotZ, scale);
            ss >> mat->ar;
            ss >> mat->ag;
            ss >> mat->ab;

            ss >> mat->dr;
            ss >> mat->dg;
            ss >> mat->db;

            ss >> mat->sr;
            ss >> mat->sg;
            ss >> mat->sb;

            ss >> mat->k_a;
            ss >> mat->k_d;
            ss >> mat->k_s;

            ss >> mat->s_exp;
            ss >> mat->ref_index;
            ss >> mat->k_relect;
            ss >> mat->k_refract;
            meshes[seenMeshes] = *mesh;
            seenMeshes++;
        }
    }
    return;
}

void drawRect(double x, double y, double w, double h) {
    glVertex2f(x, y);
    glVertex2f(x + w, y);
    glVertex2f(x + w, y + h);
    glVertex2f(x, y + h);
}

void translateModelMatrix(float x, float y, float z) {
    float matrix[16] = {1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, x, y, z, 1.0};
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(matrix);
    glMultMatrixf(modelWorldMatrix);
    glGetFloatv(GL_MODELVIEW_MATRIX, modelWorldMatrix);
}

void rotateModelMatrixX(float angle) {
    float radians = angle * M_PI / 180.0;
    float matrix[16] = {1, 0, 0, 0, 0, cos(radians), sin(radians), 0, 0, -sin(radians), cos(radians), 0, 0, 0, 0, 1};
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(modelWorldMatrix);
    glMultMatrixf(matrix);

    glGetFloatv(GL_MODELVIEW_MATRIX, modelWorldMatrix);
}

void rotateModelMatrixY(float angle) {
    float radians = angle * M_PI / 180.0;
    float matrix[16] = {cos(radians), 0, -sin(radians), 0, 0, 1, 0, 0, sin(radians), 0, cos(radians), 0, 0, 0, 0, 1};
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(modelWorldMatrix);
    glMultMatrixf(matrix);
    glGetFloatv(GL_MODELVIEW_MATRIX, modelWorldMatrix);
}

void rotateModelMatrixZ(float angle) {
    float radians = angle * M_PI / 180.0;
    float matrix[16] = {cos(radians), sin(radians), 0, 0, -sin(radians), cos(radians), 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(modelWorldMatrix);
    glMultMatrixf(matrix);
    glGetFloatv(GL_MODELVIEW_MATRIX, modelWorldMatrix);
}

void scaleModelMatrix(float scaleFactor) {
    float matrix[16] = {scaleFactor, 0, 0, 0, 0, scaleFactor, 0, 0, 0, 0, scaleFactor, 0, 0, 0, 0, 1};
    glMatrixMode(GL_MODELVIEW);
    glLoadMatrixf(modelWorldMatrix);
    glMultMatrixf(matrix);
    glGetFloatv(GL_MODELVIEW_MATRIX, modelWorldMatrix);
}


// The display function. It is called whenever the window needs
// redrawing (ie: overlapping window moves, resize, maximize)
// You should redraw your polygons here
void display(void) {
    // Clear the background
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    double w = 10 / double(fb->GetWidth());
    double h = 10 / double(fb->GetHeight());

    Color cl;
    glColor3f(0, 0, 1);

    glBegin(GL_QUADS);

    printf("width %d, height %d\n", fb->GetWidth(), fb->GetHeight());

    for (int y = 0; y < fb->GetHeight(); y++) {
        for (int x = 0; x < fb->GetHeight(); x++) {
            cl = fb->buffer[x][y].color;
            glColor3f(cl.r, cl.g, cl.b);

            drawRect(w * x, h * y, w, h);
        }
    }

    glEnd();
    glutSwapBuffers();
}

void render()
{
	point origin = point(0, 0, 0);
	//printf("Frambuffer size (%d, %d)\n", fb->GetHeight(), fb->GetWidth());
	for (int y = 0; y < fb->GetHeight(); y ++)
	{
		for (int x = 0; x < fb->GetWidth(); x++)
		{
			//printf("Rendering pixel (%d, %d)\n", x, y);
			float ypx = (y / (fb->GetHeight() / (2 * imageplaneHalfSide))) - imageplaneHalfSide;
			float xpx = (x / (fb->GetWidth() / (2 * imageplaneHalfSide))) - imageplaneHalfSide;

			Ray *r = (Ray *)malloc(sizeof(Ray));
			
			r->origin = origin;
			point pxPoint = normalize(subtract(point(xpx, ypx, -imagePlaneDistance), origin));
			r->direction = pxPoint;
			r->isExternal = true;

			Color c = rayTrace(r, 3);
			fb->SetPixel(x, y, c);
		}
	}
}


// This function is called whenever the window is resized. 
// Parameters are the new dimentions of the window
void resize(int x, int y) {
    glViewport(0, 0, x, y);
    window_width = x;
    window_height = y;

    printf("Resized to %d %d\n", x, y);
}


// This function is called whenever the mouse is pressed or released
// button is a number 0 to 2 designating the button
// state is 1 for release 0 for press event
// x and y are the location of the mouse (in window-relative coordinates)
void mouseButton(int button, int state, int x, int y) {
    ;
}


//This function is called whenever the mouse is moved with a mouse button held down.
// x and y are the location of the mouse (in window-relative coordinates)
void mouseMotion(int x, int y) {
    ;
}


// This function is called whenever there is a keyboard input
// key is the ASCII value of the key pressed
// x and y are the location of the mouse
void keyboard(unsigned char key, int x, int y) {
    switch (key) {
        case 'q':                           /* Quit */
            exit(1);
            break;
        case '-':
            fb->Resize(fb->GetHeight() / 2, fb->GetWidth() / 2);
			render();
//		BresenhamLine(fb, fb->GetWidth()*0.1, fb->GetHeight()*0.1, fb->GetWidth()*0.9, fb->GetHeight()*0.9, Color(1, 0, 0));
            break;
        case '=':
            fb->Resize(fb->GetHeight() * 2, fb->GetWidth() * 2);
			render();
//		BresenhamLine(fb, fb->GetWidth()*0.1, fb->GetHeight()*0.1, fb->GetWidth()*0.9, fb->GetHeight()*0.9, Color(1, 0, 0));
            break;
        default:
            break;
    }

    // Schedule a new display event
    glutPostRedisplay();
}


int main(int argc, char *argv[]) {

    fb = new FrameBuffer(INITIAL_RES, INITIAL_RES);

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

    //sceneReader("./spheres.rtl");
	sceneReader("./red_sphere_and_teapot.rtl");
	render();

    // Switch to main loop
	glutMainLoop();
    return 0;
}


// Transform a point with an arbitrary matrix
point Transform(float *matrix, point p) {
    point temp;
    temp.x = matrix[0] * p.x + matrix[4] * p.y + matrix[8] * p.z + matrix[12] * p.w;
    temp.y = matrix[1] * p.x + matrix[5] * p.y + matrix[9] * p.z + matrix[13] * p.w;
    temp.z = matrix[2] * p.x + matrix[6] * p.y + matrix[10] * p.z + matrix[14] * p.w;
    temp.w = matrix[3] * p.x + matrix[7] * p.y + matrix[11] * p.z + matrix[15] * p.w;
    temp.x = temp.x / temp.w;
    temp.y = temp.y / temp.w;
    temp.z = temp.z / temp.w;
    temp.w = 1.0;
    return temp;

}


float Dot(point &v, point &vx) {
    return (v.x * vx.x) + (v.y * vx.y) + (v.z * vx.z);
}

float magnitude(point p) {
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

point multiply(point p, float m) {
    point temp;
    temp.x = p.x * m;
    temp.y = p.y * m;
    temp.z = p.z * m;
    return temp;
}

point subtract(point p, point p2) {
    point temp;
    temp.x = p.x - p2.x;
    temp.y = p.y - p2.y;
    temp.z = p.z - p2.z;
    temp.w = 0;
    return temp;
}

point cross(point p, point p2) {
    point temp;
    temp.x = p.y * p2.z - p.z * p2.y;
    temp.y = p.z * p2.x - p.x * p2.z;
    temp.z = p.x * p2.y - p.y * p2.x;
    temp.w = 0;
    return temp;

}

point *raySphereIntercept(Ray *r, Sphere *s) {
    point *ret = nullptr;
    point *dP = (point *) malloc(sizeof(point));
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

    if (discriminant >= 0) {
        float sPos = Dot(r->direction, *dP) + sqrt(discriminant);
        float sNeg = Dot(r->direction, *dP) - sqrt(discriminant);
        float s;
        if (sPos < sNeg) {
            s = sPos;
        }
        else {
            s = sNeg;
        }
        ret = (point *) malloc(sizeof(point));
        ret->x = r->origin.x + s * r->direction.x;
        ret->y = r->origin.y + s * r->direction.y;
        ret->z = r->origin.z + s * r->direction.z;
        ret->w = 1;

    }

    return ret;
}


point *rayTriIntersection(Ray *r, point v1, point v2, point v3) {
    point e1, e2;
    point P, Q, T;
    float det, invDet, u, v;
    float t;
    point *ret = nullptr;

    e1 = subtract(v2, v3);
    e2 = subtract(v3, v1);
    P = cross(r->direction, e2);
    det = Dot(e1, P);
    if (det > -EPSILON && det < EPSILON) {
        return ret;
    }
    invDet = 1.f / det;
    T = subtract(r->origin, v1);
    u = Dot(T, P) * invDet;

    if (u < 0.f || u > 1.f) {
        return ret;
    }

    Q = cross(T, e1);
    v = Dot(r->direction, Q) * invDet;

    if (v < 0.f || v > 1.f) {
        return ret;
    }

    t = Dot(e2, Q) * invDet;

    if (t > EPSILON) {
        ret = (point *) malloc(sizeof(point));
        ret->x = r->origin.x + r->direction.x * t;
        ret->y = r->origin.y + r->direction.y * t;
        ret->z = r->origin.z + r->direction.z * t;
        ret->w = 0;
    }

    return ret;

}

point *sub(point *p1,  point *p2) {
    p1->x -= p2->x;
    p1->y -= p2->y;
    p1->z -= p2->z;
    return p1;
}

point normalize(point pt)
{
	float length = sqrt(pow(pt.x, 2) + pow(pt.y, 2) + pow(pt.z, 2));
	point norm;
	norm.x = pt.x / length;
	norm.y = pt.y / length;
	norm.z = pt.z / length;
	norm.w = 0;
	return norm;
}


point add(point p, point p2)
{
	point temp;
	temp.x = p.x + p2.x;
	temp.y = p.y + p2.y;
	temp.z = p.z + p2.z;
	temp.w = 0;
	return temp;
}

point reflect(point incident, point normal) {
	point norm = normalize(normal);
	//incident = incident * -1;
    return incident - norm * Dot(norm, incident) * 2.f;
}



point refract(point incident, point normal, float indexI, float indexR, bool isInternal) {
	point norm;
	if(isInternal)
	{
		norm = normal;
	}
	else
	{
		norm = normal * -1;
	}
	norm = normalize(norm);
	
	float indexRatio = indexI / indexR;
	float k = 1.0f - pow(indexRatio, 2) * (1.0f - pow(Dot(incident, normal), 2));
	if(k < 0.0f)
	{
		printf("K < 0");
	}
	return (incident * indexRatio) - normal * (indexRatio * Dot(norm, incident) + sqrt(k));

	/*float c1 = -Dot(norm, incident);
	float c2 = sqrt(1 - pow(indexRatio, 2) * (1 - pow(c1, 2)));

	return (incident * indexRatio) + norm * (indexRatio * c1 - c2);*/
}


float distance(point p, point p2)
{
	return magnitude(p - p2);
}

point *scale(point *p, point *scaleVals) {
    p->x *= scaleVals->x;
    p->y *= scaleVals->y;
    p->z *= scaleVals->z;
    return p;
}

float max(float input, float max) {
    if (input > max) {
        return input;
    }
    return max;
}


point* rayMeshIntersection(Ray *r, Mesh *m, int *triOut)
{
	point *res = nullptr;
	int i;
	for (i = 0; i < m->faces; i++)
	{
		faceStruct face = m->faceList[i];
		point *faceIntersect = rayTriIntersection(r, m->vertList[face.v1], m->vertList[face.v2], m->vertList[face.v3]);
		if (faceIntersect != nullptr && (res == nullptr || distance(r->origin, *faceIntersect) < distance(r->origin, *res)))
		{
			if (res != nullptr)
			{
				free(res);
			}
			res = faceIntersect;
			*triOut = i;
		}
		else
		{
			if (faceIntersect != nullptr)
			{
				free(faceIntersect);
			}
		}
	}
	return res;
}


void raycast(Ray* r, Material*& mat, point*& norm, point*& closestPoint)
{
	Sphere *closestSphere = nullptr;
	Mesh *closestMesh = nullptr;
	int *closestMeshTri = (int *)malloc(sizeof(int));
	bool intersect = false;
	bool sphereCloser = false;

	//Iterate through all meshes
	int i;
	for (i = 0; i < numMeshes; i++)
	{
		point *meshIntersect = rayMeshIntersection(r, &meshes[i], closestMeshTri);
		if (meshIntersect != nullptr && (closestPoint == nullptr || distance(r->origin, *meshIntersect) < distance(r->origin, *closestPoint)))
		{
			if (closestPoint != nullptr)
			{
				free(closestPoint);
			}
			closestPoint = meshIntersect;
			closestMesh = &meshes[i];
			intersect = true;
		}
		else
		{
			if (meshIntersect != nullptr)
			{
				free(meshIntersect);
			}
		}
	}

	//Iterate through all spheres
	for (i = 0; i < numSpheres; i++)
	{
		point *sphereIntersect = raySphereIntercept(r, &spheres[i]);
		if (sphereIntersect != nullptr && (closestPoint == nullptr || distance(r->origin, *sphereIntersect) < distance(r->origin, *closestPoint)))
		{
			if (closestPoint != nullptr)
			{
				free(closestPoint);
			}
			closestPoint = sphereIntersect;
			closestSphere = &spheres[i];
			sphereCloser = true;
			intersect = true;
		}
		else
		{
			if (sphereIntersect != nullptr)
			{
				free(sphereIntersect);
			}
		}
	}

	if (!intersect)
	{
		return;
	}

	if (sphereCloser)
	{
		mat = closestSphere->material;
        point sphereNorm = getSphereNormal(closestPoint, closestSphere);
		norm = new point(sphereNorm.x, sphereNorm.y, sphereNorm.z);
		//norm = &sphereNorm;
	}
	else
	{
		mat = closestMesh->material;
		point meshNorm = getTriNormal(closestPoint, closestMesh, *closestMeshTri);
		norm = new point(meshNorm.x, meshNorm.y, meshNorm.z);
		//norm = &meshNorm;
	}
}

Color rayTrace(Ray *r, int depth)
{

	Material *mat;
	point *norm;
	point *closestPoint = nullptr;

	raycast(r, mat, norm, closestPoint);

	//Completely missed everything, return ambient lighting of scene
	if (closestPoint == nullptr)
	{
		Color ret = Color(0.f, 0.f, 0.f);
		return ret;
	}
    point viewingVector = r->direction * -1;
	Color c = *localIllumination(*closestPoint, *norm, viewingVector, *mat);


	depth--;
	//If we haven't bottomed out
	if (depth > 0)
	{
		//Hit a surface, raycast to next surface (reflect + refract)
		point *reflectResult = nullptr;
		if (mat->k_relect > 0.f && r->isExternal)
		{
			Ray *reflectRay = (Ray *)malloc(sizeof(Ray));
			reflectRay->origin = *closestPoint;
			reflectRay->direction = reflect(r->direction, *norm);
			reflectRay->isExternal = true;
			Color reflected = rayTrace(reflectRay, depth);
			c = c + (reflected * mat->k_relect);
		}

		if (mat->k_refract > 0.f)
		{
			Ray *refractRay = (Ray *)malloc(sizeof(Ray));
			refractRay->origin = *closestPoint;
			if (r->isExternal)
			{
				refractRay->direction = refract(r->direction, *norm, 1, mat->ref_index, true);
				
				refractRay->isExternal = false;
			}
			else
			{
				refractRay->direction = refract(r->direction, *norm, mat->ref_index, 1, false);
				refractRay->isExternal = true;
			}
			Color refracted = rayTrace(refractRay, depth);
			c = c + (refracted * mat->k_refract);
		}
	}

	delete norm;

	return c;
}

point getTriNormal(point *p, Mesh *m, int i)
{
	//Getting the triangle verts
	point v1 = m->vertList[m->faceList[i].v1];
	point v2 = m->vertList[m->faceList[i].v2];
	point v3 = m->vertList[m->faceList[i].v3];

	//Getting the triangle normals
	point n1 = m->normList[m->faceList[i].n1];
	point n2 = m->normList[m->faceList[i].n2];
	point n3 = m->normList[m->faceList[i].n3];

	float a1 = triArea(v2, v3, *p);
	float a2 = triArea(v1, v3, *p);
	float a3 = triArea(v1, v2, *p);
	float A = triArea(v1, v2, v3);

	point comp1 = n1 * (a1 / A);
	point comp2 = n2 * (a2 / A);
	point comp3 = n3 * (a3 / A);

	return add(comp3, add(comp1, comp2));
}


point getSphereNormal(point *p, Sphere *s)
{
	point center;
	center.x = s->x;
	center.y = s->y;
	center.z = s->z;
	center.w = 0;

	return subtract(*p, center);
}

float triArea(point v1, point v2, point v3)
{
	point e1 = v2 - v3;
	point e2 = v1 - v3;
	point c = cross(e1, e2);
	return magnitude(c) / 2;
}


Color *localIllumination(point p, point norm, point view, Material mat) {
    point normal = normalize(norm);

    point V = normalize(view);

    Color *totalIllumination = new Color();
    Color materialDiffuse = Color(mat.dr, mat.dg, mat.db);
    Color materialSpecular = Color(mat.sr, mat.sg, mat.sb);
    Color materialAmbient = Color(mat.ar, mat.ag, mat.ab);
    Color ambientLight = Color(0.3, 0.3, 0.3) * mat.k_a * materialAmbient;
    totalIllumination->r += ambientLight.r;
    totalIllumination->g += ambientLight.g;
    totalIllumination->b += ambientLight.b;
    for (int i = 0; i < numLights; i++) {
        Light l = lights[i];

        //The light's RGB color values and position in space
        Color lightColor = Color(l.r, l.g, l.b);
        point lightPos = point(l.x, l.y, l.z);
        //Compute the amount of light generated by this light

        //Compute light vector
        point L;
        if(l.light_type == 0){
            //Directional
            L = normalize(lightPos * -1);
        } else if (l.light_type == 1){
            //Point lights
            L = normalize(lightPos - p);
        }

        //Compute halfway vector
        point H = normalize(V+L);


		float nLDot = Dot(normal, L);
		float nHDot = Dot(normal, H);
        //compute diffuse lighting
        Color diffuseLight = materialDiffuse * max(Dot(normal, L), 0.0) * lightColor * mat.k_d;
        diffuseLight = diffuseLight.Clamp(0.0, 1.0);

        //Compute specular light
        Color specularLight = lightColor * materialSpecular * mat.k_s * pow(max(Dot(normal, H), 0.0), mat.s_exp);
        specularLight = specularLight.Clamp(0.0, 1.0);
        totalIllumination->r += specularLight.r + diffuseLight.r;
        totalIllumination->g += specularLight.g + diffuseLight.g;
        totalIllumination->b += specularLight.b + diffuseLight.b;
    }
    //cout << "Total Illumination Color: " << totalIllumination->r << ", " << totalIllumination->g << ", " << totalIllumination->b << "\n";
    return totalIllumination;
}