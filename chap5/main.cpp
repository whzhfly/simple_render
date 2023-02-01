#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include "tgaimage.h"
#include "geometry.h"
#include "model.h"
#include <string>
#include <fstream>
#include <sstream>
#include "Matrix.hpp"


const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
const TGAColor green = TGAColor(0,   255, 0,   255);
const TGAColor blue  = TGAColor(0,   0,   255, 255);
const int width  = 800;
const int height = 800;
const int depth  = 255;

int test_count = 20;

Model *model = NULL;
int *zbuffer = NULL;
// Vec3f light_dir(0,0,-1);
Vec3f light_dir = Vec3f(1,-1,1).normalize();
Vec3f camera(0,0,3);
Vec3f eye(1,1,3);
Vec3f center(0,0,0);

void line(Vec2i p0, Vec2i p1, TGAImage &image, TGAColor color) {
    bool steep = false;
    if (std::abs(p0.x-p1.x)<std::abs(p0.y-p1.y)) {
        std::swap(p0.x, p0.y);
        std::swap(p1.x, p1.y);
        steep = true;
    }
    if (p0.x>p1.x) {
        std::swap(p0, p1);
    }

    for (int x=p0.x; x<=p1.x; x++) {
        float t = (x-p0.x)/(float)(p1.x-p0.x);
        int y = p0.y*(1.-t) + p1.y*t + .5;
        if (steep) {
            image.set(y, x, color);
        } else {
            image.set(x, y, color);
        }
    }
}

void rasterize(Vec2i p0, Vec2i p1, TGAImage &image, TGAColor color, int ybuffer[]) {
    if (p0.x>p1.x) {
        std::swap(p0, p1);
    }
    for (int x=p0.x; x<=p1.x; x++) {
        float t = (x-p0.x)/(float)(p1.x-p0.x);
        int y = p0.y*(1.-t) + p1.y*t + .5;
        if (ybuffer[x]<y) {
            ybuffer[x] = y;
            image.set(x, 0, color);
        }
    }
}


////////////////////////
/*
    3D Begin
*/
///////////////////////
Vec3f barycentric(Vec2i* pts, Vec2i P){
    Vec3f u = Vec3f(pts[2].raw[0]-pts[0].raw[0], pts[1].raw[0]-pts[0].raw[0], pts[0].raw[0]-P.raw[0])^Vec3f(pts[2].raw[1]-pts[0].raw[1], pts[1].raw[1]-pts[0].raw[1], pts[0].raw[1]-P.raw[1]);
    /* `pts` and `P` has integer value as coordinates
    so `abs(u[2])` < 1 means `u[2]` is 0, that means
       triangle is degenerate, in this case return something with negative coordinates */
    if (std::abs(u.z)<1) return Vec3f(-1,1,1);
    return Vec3f(1.f-(u.x+u.y)/u.z, u.y/u.z, u.x/u.z); 
}

///////////////////////
Vec3f f3_barycentric(Vec3f* pts, Vec3f P){
    Vec3f u = Vec3f(pts[2].raw[0]-pts[0].raw[0], pts[1].raw[0]-pts[0].raw[0], pts[0].raw[0]-P.raw[0])^Vec3f(pts[2].raw[1]-pts[0].raw[1], pts[1].raw[1]-pts[0].raw[1], pts[0].raw[1]-P.raw[1]);
    /* `pts` and `P` has integer value as coordinates
    so `abs(u[2])` < 1 means `u[2]` is 0, that means
       triangle is degenerate, in this case return something with negative coordinates */
    if (std::abs(u.z)<1) return Vec3f(-1,1,1);
    return Vec3f(1.f-(u.x+u.y)/u.z, u.y/u.z, u.x/u.z); 
}

void new_fill_triangle(Vec2i *pts, TGAImage &image, TGAColor color) { 
    Vec2i bboxmin(image.get_width()-1,  image.get_height()-1); 
    Vec2i bboxmax(0, 0); 
    Vec2i clamp(image.get_width()-1, image.get_height()-1); 
    for (int i=0; i<3; i++) { 
        bboxmin.x = std::max(0, std::min(bboxmin.x, pts[i].x));
	bboxmin.y = std::max(0, std::min(bboxmin.y, pts[i].y));

	bboxmax.x = std::min(clamp.x, std::max(bboxmax.x, pts[i].x));
	bboxmax.y = std::min(clamp.y, std::max(bboxmax.y, pts[i].y));
    } 
    Vec2i P; 
    for (P.x=bboxmin.x; P.x<=bboxmax.x; P.x++) { 
        for (P.y=bboxmin.y; P.y<=bboxmax.y; P.y++) { 
            Vec3f bc_screen  = barycentric(pts, P); 
            if (bc_screen.x<0 || bc_screen.y<0 || bc_screen.z<0) continue;
            // add Hidden in there
            image.set(P.x, P.y, color); 
        } 
    } 
} 

void new_fill_triangle_with_hidden(Vec3f *pts, float *zbuffer, TGAImage &image, TGAColor color) {
    Vec2f bboxmin( std::numeric_limits<float>::max(),  std::numeric_limits<float>::max());
    Vec2f bboxmax(-std::numeric_limits<float>::max(), -std::numeric_limits<float>::max());
    Vec2f clamp(image.get_width()-1, image.get_height()-1);
    for (int i=0; i<3; i++) {
        for (int j=0; j<2; j++) {
            bboxmin.raw[j] = std::max(0.f,      std::min(bboxmin.raw[j], pts[i].raw[j]));
            bboxmax.raw[j] = std::min(clamp.raw[j], std::max(bboxmax.raw[j], pts[i].raw[j]));
        }
    }
    Vec3f P;
    for (P.x=bboxmin.x; P.x<=bboxmax.x; P.x++) {
        for (P.y=bboxmin.y; P.y<=bboxmax.y; P.y++) {
            Vec3f bc_screen  = f3_barycentric(pts, P);
            if (bc_screen.x<0 || bc_screen.y<0 || bc_screen.z<0) continue;
            P.z = 0;
            for (int i=0; i<3; i++) P.z += pts[i].raw[2]*bc_screen.raw[i];
            if (zbuffer[int(P.x+P.y*width)]<P.z) {
                zbuffer[int(P.x+P.y*width)] = P.z;
                image.set(P.x, P.y, color);
            }
        }
    }
}

void id_triangle(Vec3i t0, Vec3i t1, Vec3i t2, Vec2i uv0, Vec2i uv1, Vec2i uv2, TGAImage &image, float intensity, int *zbuffer) {
    if (t0.y==t1.y && t0.y==t2.y) return; // i dont care about degenerate triangles
    if (t0.y>t1.y) { std::swap(t0, t1); std::swap(uv0, uv1); }
    if (t0.y>t2.y) { std::swap(t0, t2); std::swap(uv0, uv2); }
    if (t1.y>t2.y) { std::swap(t1, t2); std::swap(uv1, uv2); }

    int total_height = t2.y-t0.y;
    for (int i=0; i<total_height; i++) {
        bool second_half = i>t1.y-t0.y || t1.y==t0.y;
        int segment_height = second_half ? t2.y-t1.y : t1.y-t0.y;
        float alpha = (float)i/total_height;
        float beta  = (float)(i-(second_half ? t1.y-t0.y : 0))/segment_height; // be careful: with above conditions no division by zero here
        Vec3i A   =               t0  + Vec3f(t2-t0  )*alpha;
        Vec3i B   = second_half ? t1  + Vec3f(t2-t1  )*beta : t0  + Vec3f(t1-t0  )*beta;
        Vec2i uvA =               uv0 +      (uv2-uv0)*alpha;
        Vec2i uvB = second_half ? uv1 +      (uv2-uv1)*beta : uv0 +      (uv1-uv0)*beta;
        if (A.x>B.x) { std::swap(A, B); std::swap(uvA, uvB); }
        for (int j=A.x; j<=B.x; j++) {
            float phi = B.x==A.x ? 1. : (float)(j-A.x)/(float)(B.x-A.x);
            Vec3i   P = Vec3f(A) + Vec3f(B-A)*phi;
            Vec2i uvP =     uvA +   (uvB-uvA)*phi;
            int idx = P.x+P.y*width;
            if (zbuffer[idx]<P.z) {
                zbuffer[idx] = P.z;
                TGAColor color = model->diffuse(uvP);
                image.set(P.x, P.y, TGAColor(color.r*intensity, color.g*intensity, color.b*intensity));
            }
        }
    }
}



void id_triangle_light(Vec3i t0, Vec3i t1, Vec3i t2, float ity0, float ity1, float ity2, TGAImage &image, int *zbuffer) {
    if (t0.y==t1.y && t0.y==t2.y) return; // i dont care about degenerate triangles
    if (t0.y>t1.y) { std::swap(t0, t1); std::swap(ity0, ity1); }
    if (t0.y>t2.y) { std::swap(t0, t2); std::swap(ity0, ity2); }
    if (t1.y>t2.y) { std::swap(t1, t2); std::swap(ity1, ity2); }

    int total_height = t2.y-t0.y;
    for (int i=0; i<total_height; i++) {
        bool second_half = i>t1.y-t0.y || t1.y==t0.y;
        int segment_height = second_half ? t2.y-t1.y : t1.y-t0.y;
        float alpha = (float)i/total_height;
        float beta  = (float)(i-(second_half ? t1.y-t0.y : 0))/segment_height; // be careful: with above conditions no division by zero here
        Vec3i A   =               t0  + Vec3f(t2-t0  )*alpha;
        Vec3i B   = second_half ? t1  + Vec3f(t2-t1  )*beta : t0  + Vec3f(t1-t0  )*beta;
        float ityA =               ity0 +   (ity2-ity0)*alpha;
        float ityB = second_half ? ity1 +   (ity2-ity1)*beta : ity0 +   (ity1-ity0)*beta;
        if (A.x>B.x) { std::swap(A, B); std::swap(ityA, ityB); }
        for (int j=A.x; j<=B.x; j++) {
            float phi = B.x==A.x ? 1. : (float)(j-A.x)/(float)(B.x-A.x);
            Vec3i   P = Vec3f(A) + Vec3f(B-A)*phi;
            float ityP =    ityA  + (ityB-ityA)*phi;
            int idx = P.x+P.y*width;
            if (P.x>=width||P.y>=height||P.x<0||P.y<0) continue;
            if (zbuffer[idx]<P.z) {
                zbuffer[idx] = P.z;
                image.set(P.x, P.y, TGAColor(255, 255, 255)*ityP);
            }
        }
    }
}


Vec3i i3_barycentric(Vec3i* pts, Vec3i P){
    Vec3f u = Vec3f(pts[2].raw[0]-pts[0].raw[0], pts[1].raw[0]-pts[0].raw[0], pts[0].raw[0]-P.raw[0])^Vec3f(pts[2].raw[1]-pts[0].raw[1], pts[1].raw[1]-pts[0].raw[1], pts[0].raw[1]-P.raw[1]);
    /* `pts` and `P` has integer value as coordinates
    so `abs(u[2])` < 1 means `u[2]` is 0, that means
       triangle is degenerate, in this case return something with negative coordinates */
    if (std::abs(u.z)<1) return Vec3f(-1,1,1);
    return Vec3f(1.f-(u.x+u.y)/u.z, u.y/u.z, u.x/u.z); 
}


Vec3f world2screen(Vec3f v) {
    /*
        It means that i have a point Vec2f v, it belongs to the square [-1,1]*[-1,1]. 
        I want to draw it in the image of (width, height) dimensions. 
        Value (v.x+1) is varying between 0 and 2, (v.x+1)/2 between 0 and 1, and (v.x+1)*width/2 sweeps all the image.
        Thus we effectively mapped the bi-unit square onto the image.
    */
    return Vec3f(int((v.x+1.)*width/2.+.5), int((v.y+1.)*height/2.+.5), v.z);
}


Vec3f world2screen_project_camera(Matrix Projection, Matrix ViewPort, Vec3f v) {
    return m2v(ViewPort*Projection*v2m(v));
}

Vec3f world2screen_project_camera_model(Matrix model_transform, Vec3f v) {
    return m2v(model_transform*v2m(v));
}
////////////////////////
/*
    3D End
*/
///////////////////////


int main(int argc, char** argv) {
    // 3d show
    TGAImage texture_scene(width, height, TGAImage::RGB);
    auto path = "obj/african_head_diffuse.tga";
    texture_scene.read_tga_file(path);
    int text_width = texture_scene.get_width();
    int text_height = texture_scene.get_height();
    auto t_s = sizeof(texture_scene.buffer());
    std::cout<<text_width<<text_height<<std::endl;

    if (2==argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("obj/african_head.obj");
    }
    TGAImage image(width, height, TGAImage::RGB);
    zbuffer = new int[width*height];
    for (int i=0; i<width*height; i++) {
        zbuffer[i] = std::numeric_limits<int>::min();
    }
    // add matrix
    Matrix ModelView = n_lookat(eye, center, Vec3f(0,1,0));
    // Matrix ModelView = n_lookat(eye, camera, Vec3f(0,1,0));
    Matrix Projection = Matrix::identity(4);
    // Projection[3][2] = -1.f/camera.z;
    Projection[3][2] = -1.f/(eye-center).norm();
    Matrix ViewPort   = viewport(width/8, height/8, width*3/4, height*3/4);


    Matrix model_transform = (ViewPort*Projection*ModelView);

    {
        for (int i=0; i<model->nfaces(); i++) {
            std::vector<int> face = model->face(i);

            Vec3f pts[3];
            Vec3f world_coords[3];
            float intensity[3];
            for (int j=0; j<3; j++) {
                Vec3f pos = model->vert(face[j]);
                pts[j] = world2screen_project_camera_model(model_transform, pos);
                world_coords[j]= pos;
                intensity[j] = model->norm(i, j)*light_dir;
            }
            // get texture begin
            Vec2f f_pt[2];
            id_triangle_light(pts[0], pts[1], pts[2], intensity[0], intensity[1], intensity[2], image, zbuffer);
        }
        image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
        image.write_tga_file("output.tga");
    }


    { // dump z-buffer (debugging purposes only)
        TGAImage zbimage(width, height, TGAImage::GRAYSCALE);
        for (int i=0; i<width; i++) {
            for (int j=0; j<height; j++) {
                zbimage.set(i, j, TGAColor(zbuffer[i+j*width], 1));
            }
        }
        zbimage.flip_vertically(); // i want to have the origin at the left bottom corner of the image
        zbimage.write_tga_file("zbuffer.tga");
    }

    delete model;
    return 0;
}

