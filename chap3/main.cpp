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

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
const TGAColor green = TGAColor(0,   255, 0,   255);
const TGAColor blue  = TGAColor(0,   0,   255, 255);
const int width  = 800;
const int height = 500;

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

Vec3f world2screen(Vec3f v) {
    return Vec3f(int((v.x+1.)*width/2.+.5), int((v.y+1.)*height/2.+.5), v.z);
}
////////////////////////
/*
    3D End
*/
///////////////////////


int main(int argc, char** argv) {

    // 2d show
    { // just dumping the 2d scene (yay we have enough dimensions!)
        TGAImage scene(width, height, TGAImage::RGB);

        // scene "2d mesh"
        line(Vec2i(20, 34),   Vec2i(744, 400), scene, red);
        line(Vec2i(120, 434), Vec2i(444, 400), scene, green);
        line(Vec2i(330, 463), Vec2i(594, 200), scene, blue);

        // screen line
        line(Vec2i(10, 10), Vec2i(790, 10), scene, white);

        scene.flip_vertically(); // i want to have the origin at the left bottom corner of the image
        scene.write_tga_file("scene.tga");
    }

    {
        TGAImage render(width, 16, TGAImage::RGB);
        int ybuffer[width];
        for (int i=0; i<width; i++) {
            ybuffer[i] = std::numeric_limits<int>::min();
        }
        rasterize(Vec2i(20, 34),   Vec2i(744, 400), render, red,   ybuffer);
        rasterize(Vec2i(120, 434), Vec2i(444, 400), render, green, ybuffer);
        rasterize(Vec2i(330, 463), Vec2i(594, 200), render, blue,  ybuffer);

        // 1-pixel wide image is bad for eyes, lets widen it
        for (int i=0; i<width; i++) {
            for (int j=1; j<16; j++) {
                render.set(i, j, render.get(i, 0));
            }
        }
        render.flip_vertically(); // i want to have the origin at the left bottom corner of the image
        render.write_tga_file("render.tga");
    }


    // 3d show
    {
        TGAImage texture_scene(width, height, TGAImage::RGB);
            // texture_scene.read_tga_file("obj/african_head_diffuse.tga");
        auto path = "obj/african_head_diffuse.tga";
        // std::ifstream in;
        // in.open(path, std::ifstream::in);
        texture_scene.read_tga_file(path);
        int text_width = texture_scene.get_width();
        int text_height = texture_scene.get_height();
        auto t_s = sizeof(texture_scene.buffer());
        std::cout<<text_width<<text_height<<std::endl;

        float *zbuffer = new float[width*height];

        Model* model = new Model("obj/african_head.obj");
        // Vec3f light_dir(0,0,-1); // define light_dir
        Vec3f light_dir(0,1,1); // define light_dir

        TGAImage image(width, height, TGAImage::RGB);

        for (int i=0; i<model->nfaces(); i++) {
            std::vector<int> face = model->face(i);

            Vec3f pts[3];
            Vec3f world_coords[3];
            for (int i=0; i<3; i++) {
                pts[i] = world2screen(model->vert(face[i]));
                world_coords[i]= model->vert(face[i]);
            }
            Vec3f n = (world_coords[2]-world_coords[0])^(world_coords[1]-world_coords[0]);
            n.normalize();
            float intensity = n*light_dir;



            // get texture begin
            std::vector<int> texture_face = model->face_texture(i);
            Vec2f f_pt[2];
            for (int i=0; i<2; i++) {
                f_pt[i] = model->texture(texture_face[i]);
            }
            // std::string s = (char*)(texture_scene.buffer());
            // TGAColor c = texture_scene.get(int(f_pt->u*text_width*2-.5), int(f_pt->v*text_height*2-.5));
            // TGAColor c = texture_scene.get(int(f_pt->u*text_width/2-.5), int(f_pt->v*text_height/2-.5));
            TGAColor c = texture_scene.get(int(f_pt->u*text_width), int(f_pt->v*text_height));
            // new_fill_triangle_with_hidden(pts, zbuffer, image, TGAColor(rand()%255, rand()%255, rand()%255, 255));
            // c.r *= intensity;
            // c.g *= intensity;
            // c.b *= intensity;
            // get texture end

            new_fill_triangle_with_hidden(pts, zbuffer, image, c);
        }

        // for (int i=0; i<model->nfaces(); i++) {
        //     std::vector<int> face = model->face(i);
        //     Vec2i screen_coords[3];

        //     Vec3f world_coords[3]; // use for light

        //     for (int j=0; j<3; j++) {
        //         Vec3f v = model->vert(face[j]); 
        //         screen_coords[j] = Vec2i((v.x+1.)*width/2., (v.y+1.)*height/2.);
        //         world_coords[j]  = v; 
        //     }
        //     Vec3f n = (world_coords[2]-world_coords[0])^(world_coords[1]-world_coords[0]);
        //     n.normalize();
        //     float intensity = n*light_dir; 
        //     // new_fill_triangle_with_hidden(screen_coords[0], screen_coords[1], screen_coords[2], image, TGAColor(rand()%255, rand()%255, rand()%255, 255)); 
        //     new_fill_triangle_with_hidden(screen_coords, zbuffer, image, TGAColor(rand()%255, rand()%255, rand()%255, 255)); 
        //     // if (intensity>0) {
        //     //     new_fill_triangle_with_hidden(screen_coords, image, TGAColor(intensity*255, intensity*255, intensity*255, 255));
        //     // }
        // }

        image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
        image.write_tga_file("output.tga");
        delete model;
    }
    return 0;
}

