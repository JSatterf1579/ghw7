#include <stdio.h>
#include <stdlib.h>

#include <OpenGL/gl.h>
#include <GLUT/glut.h>
#include <math.h>

#include "frame_buffer.h"
#include <fstream>
#include <string>
#include "color.h"
#include <vector>

#include <iostream>
#include <sstream>

#define ON 1
#define OFF 0
#define M_PI 3.14159265358979323846

using namespace std;


// Global variables
int window_width, window_height;    // Window dimensions
int numSpheres, numMeshes, numLights;

const int INITIAL_RES = 400;

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
};

typedef struct _faceStruct {
    int v1, v2, v3;
    int n1, n2, n3;
} faceStruct;

typedef struct Material {
    float ar, ag, ab, dr, dg, db, sr, sg, sb, k_a, k_d, k_s, s_exp, ref_index, k_relect, k_refract;
};

typedef struct Mesh {
    int verts, faces, norms;    // Number of vertices, faces and normals in the system
    point *vertList, *normList; // Vertex and Normal Lists
    faceStruct *faceList;        // Face List
    Material *material;
};

typedef struct Light {
    int light_type;
    float x, y, z, r, g, b;
};

typedef struct Sphere {
    float radius;
    float x, y, z;
    Material *material;
};

Sphere *spheres;
Mesh *meshes;
Light *lights;

void scaleModelMatrix(float scaleFactor);

void translateModelMatrix(float x, float y, float z);

void rotateModelMatrixX(float angle);

void rotateModelMatrixY(float angle);

void rotateModelMatrixZ(float angle);

point Transform(float *matrix, point p);

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

        cout << "Line Type: " << lineType << '\n';

        if (lineType == 'L' && seenLights < numLights) {

            //Process the rest of the line as if it is a light definition
            Light *l = (Light *) malloc(sizeof(struct Light));
            ss >> l->light_type;
            ss >> l->x;
            ss >> l->y;
            ss >> l->z;
            ss >> l->r;
            ss >> l->g;
            ss >> l->b;
            lights[seenLights] = *l;
            seenLights++;
            std::string type = (l->light_type == 0) ? "Point" : "Directional";
            cout << "We found a " << type << " Light at position (" << l->x << ", " << l->y << ", " << l->z <<
            ") with color (" << l->r << ", " << l->g << ", " << l->b << ").\n";
        } else if (lineType == 'S' && seenSpheres < numSpheres) {
            //Process the rest of the line as if it is a sphere definition
            Sphere *s = (Sphere *) malloc(sizeof(struct Sphere));
            Material *mat = (Material *) malloc(sizeof(struct Material));
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
            cout << "The material it uses has properties: RGB Ambient " << mat->ar << "," << mat->ag << "," << mat->ab <<
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

            Mesh * mesh = (Mesh *)malloc(sizeof(struct Mesh));
            Material *mat = (Material *) malloc(sizeof(struct Material));
            mesh->material = mat;

            ss >> scale;
            ss >> rotX;
            ss >> rotY;
            ss >> rotZ;
            ss >> x;
            ss >> y;
            ss >> z;

            cout << "Attempting to read in mesh: " << meshName << "\n";
            char * temp = strdup(meshName.c_str());
            meshReader(temp,1,mesh,x,y,z,rotX,rotY,rotZ,scale);
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
//		BresenhamLine(fb, fb->GetWidth()*0.1, fb->GetHeight()*0.1, fb->GetWidth()*0.9, fb->GetHeight()*0.9, Color(1, 0, 0));
            break;
        case '=':
            fb->Resize(fb->GetHeight() * 2, fb->GetWidth() * 2);
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

//	BresenhamLine(fb, fb->GetWidth()*0.1, fb->GetHeight()*0.1, fb->GetWidth()*0.9, fb->GetHeight()*0.9, Color(1, 0, 0));



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

    sceneReader("./red_sphere_and_teapot.rtl");

    // Switch to main loop
//	glutMainLoop();
    return 0;
}


// Transform a point with an arbitrary matrix
point Transform(float *matrix, point p) {
    point temp;
    temp.x = matrix[0] * p.x + matrix[4] * p.y + matrix[8] * p.z + matrix[12] * p.w;
    temp.y = matrix[1] * p.x + matrix[5] * p.y + matrix[9] * p.z + matrix[13] * p.w;
    temp.z = matrix[2] * p.x + matrix[6] * p.y + matrix[10] * p.z + matrix[14] * p.w;
    temp.w = matrix[3] * p.x + matrix[7] * p.y + matrix[11] * p.z + matrix[15] * p.w;
    return temp;

}
