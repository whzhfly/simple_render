#include <vector>
#include <cmath>
#include "tgaimage.h"
#include "model.h"
#include "geometry.h"

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red   = TGAColor(255, 0,   0,   255);
const TGAColor green   = TGAColor(0, 255,   0,   255);
Model *model = NULL;
const int width  = 800;
const int height = 800;

void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color) {
    bool steep = false;
    if (std::abs(x0-x1)<std::abs(y0-y1)) {
        std::swap(x0, y0);
        std::swap(x1, y1);
        steep = true;
    }
    if (x0>x1) {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    for (int x=x0; x<=x1; x++) {
        float t = (x-x0)/(float)(x1-x0);
        int y = y0*(1.-t) + y1*t;
        if (steep) {
            image.set(y, x, color);
        } else {
            image.set(x, y, color);
        }
    }
}

void sort_y_triangle(Vec2i& t0, Vec2i& t1, Vec2i& t2){
    /*
        sort vertex by y coordinates
    */
    if (t0.y > t1.y) std::swap(t0, t1);
    if (t0.y > t2.y) std::swap(t0, t2);
    if (t1.y > t2.y) std::swap(t1, t2);
};


void triangle(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color){
    sort_y_triangle(t0, t1, t2);
    line(t0.x, t0.y, t1.x, t1.y, image, white);
    line(t2.x, t2.y, t1.x, t1.y, image, green);
    line(t0.x, t0.y, t2.x, t2.y, image, red);
}

void triangle_cutting_horizon(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color){
    /*
        draw the bottom half of the triangle by cutting it horizontally
        after sort the bottom vertex is t0 and t1
        so the point which y coordinates is bigger than t1.y should not paint
        so line t1-t2 should not paint and line t0-t2 cutting horizon
    */
    sort_y_triangle(t0, t1, t2);
    int total_height = t2.y-t0.y; 
    for (int y=t0.y; y<=t1.y; y++) { 
        int segment_height = t1.y-t0.y+1; 
        float alpha = (float)(y-t0.y)/total_height; 
        float beta  = (float)(y-t0.y)/segment_height; // be careful with divisions by zero 
        Vec2i A = t0 + (t2-t0)*alpha; 
        Vec2i B = t0 + (t1-t0)*beta; 
        image.set(A.x, y, red); 
        image.set(B.x, y, green); 
    } 
}

void triangle_fill(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color){
    /*
        基于 triangle_cutting_horizon 的基础
        一个三角形分层上下两部分 用t1.y 分割
        每次循环 同一个y triangle_cutting_horizon是分别回执了两个线上的点 mage.set(A.x, y, red)|image.set(B.x, y, green)
        这里我们可以加个内循环 通过这两个点 绘制线
    */
    sort_y_triangle(t0, t1, t2);
    int total_height = t2.y-t0.y; 
    for (int y=t0.y; y<=t1.y; y++) { 
        int segment_height = t1.y-t0.y+1; 
        float alpha = (float)(y-t0.y)/total_height; 
        float beta  = (float)(y-t0.y)/segment_height; // be careful with divisions by zero 
        Vec2i A = t0 + (t2-t0)*alpha; 
        Vec2i B = t0 + (t1-t0)*beta; 
        if (A.x>B.x) std::swap(A, B); 
        for (int j=A.x; j<=B.x; j++) { 
            image.set(j, y, red); // attention, due to int casts t0.y+i != A.y 
        } 
    } 
    for (int y=t1.y; y<=t2.y; y++) { 
        int segment_height =  t2.y-t1.y+1; 
        float alpha = (float)(y-t0.y)/total_height; 
        float beta  = (float)(y-t1.y)/segment_height; // be careful with divisions by zero 
        Vec2i A = t0 + (t2-t0)*alpha; 
        Vec2i B = t1 + (t2-t1)*beta; 
        if (A.x>B.x) std::swap(A, B); 
        for (int j=A.x; j<=B.x; j++) { 
            image.set(j, y, green); // attention, due to int casts t0.y+i != A.y 
        } 
    } 
}

void triangle_fill_op(Vec2i t0, Vec2i t1, Vec2i t2, TGAImage &image, TGAColor color){
    /*
        triangle_fill 基础上把两段上下代码合并
        对于一个 y 找出对应的 x1 x2 然后 x1 x2 绘制线
        加个second_half 表示上一段还是下一段
    */
    sort_y_triangle(t0, t1, t2);
    int total_height = t2.y-t0.y; 
    for (int i=0; i<total_height; i++) { 
        bool second_half = i>t1.y-t0.y || t1.y==t0.y; 
        int segment_height = second_half ? t2.y-t1.y : t1.y-t0.y; 
        float alpha = (float)i/total_height; 
        float beta  = (float)(i-(second_half ? t1.y-t0.y : 0))/segment_height; // be careful: with above conditions no division by zero here 
        Vec2i A =               t0 + (t2-t0)*alpha;
        Vec2i B = second_half ? t1 + (t2-t1)*beta : t0 + (t1-t0)*beta;
        if (A.x>B.x) std::swap(A, B); 
        for (int j=A.x; j<=B.x; j++) { 
            image.set(j, t0.y+i, color); // attention, due to int casts t0.y+i != A.y 
        } 
    } 
}

////////////////////////
/*
    Begin
    new method in triangle
    find barycentric 重心坐标
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
            image.set(P.x, P.y, color);
        } 
    } 
} 

////////////////////////
/*
    END
*/
///////////////////////


int main(int argc, char** argv) {
    if (2==argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("obj/african_head.obj");
    }

    // Vec3f light_dir(0,0,-1); // define light_dir
    Vec3f light_dir(0,1,1); // define light_dir

    TGAImage image(width, height, TGAImage::RGB);
    for (int i=0; i<model->nfaces(); i++) {
        std::vector<int> face = model->face(i);
        Vec2i screen_coords[3];

        Vec3f world_coords[3]; // use for light

        for (int j=0; j<3; j++) {
            Vec3f v = model->vert(face[j]); 
            screen_coords[j] = Vec2i((v.x+1.)*width/2., (v.y+1.)*height/2.);
            world_coords[j]  = v; 
        }
        Vec3f n = (world_coords[2]-world_coords[0])^(world_coords[1]-world_coords[0]);
        n.normalize();
        float intensity = n*light_dir; 
        // triangle_fill_op(screen_coords[0], screen_coords[1], screen_coords[2], image, TGAColor(rand()%255, rand()%255, rand()%255, 255)); 
        if (intensity>0) {
            new_fill_triangle(screen_coords, image, TGAColor(intensity*255, intensity*255, intensity*255, 255));
        }
    }
    
    // Vec2i t0[3] = {Vec2i(10, 70),   Vec2i(50, 160),  Vec2i(70, 80)}; 
    // Vec2i t1[3] = {Vec2i(180, 50),  Vec2i(150, 1),   Vec2i(70, 180)}; 
    // Vec2i t2[3] = {Vec2i(180, 150), Vec2i(120, 160), Vec2i(130, 180)}; 
    // triangle(t0[0], t0[1], t0[2], image, red); 
    // triangle(t1[0], t1[1], t1[2], image, white); 
    // triangle(t2[0], t2[1], t2[2], image, green);

    // triangle_fill(t0[0], t0[1], t0[2], image, red); 
    // triangle_fill(t1[0], t1[1], t1[2], image, white); 
    // triangle_fill(t2[0], t2[1], t2[2], image, green);


    // new_fill_triangle(t0, image, red); 
    // new_fill_triangle(t1, image, white); 
    // new_fill_triangle(t2, image, green);

    image.flip_vertically(); // i want to have the origin at the left bottom corner of the image
    image.write_tga_file("output.tga");
    delete model;
    return 0;
}

