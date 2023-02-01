#include <vector>
#include <iostream>

#include "tgaimage.h"
#include "model.h"
#include "geometry.h"
#include "our_gl.h"

Model *model     = NULL;
const int width  = 800;
const int height = 800;

Vec3f light_dir(1,-1,1);
// Vec3f       eye(0,-1,3);
Vec3f eye(1,1,3);
Vec3f    center(0,0,0);
Vec3f        up(0,1,0);

struct GouraudShader : public IShader {
    Vec3f varying_intensity; // written by vertex shader, read by fragment shader
    mat<2,3,float> varying_uv;

    virtual Vec4f vertex(int iface, int nthvert) {
        varying_uv.set_col(nthvert, model->uv(iface, nthvert));
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
        gl_Vertex = Viewport*Projection*ModelView*gl_Vertex;     // transform it to screen coordinates
        varying_intensity[nthvert] = std::max(0.f, model->normal(iface, nthvert)*light_dir); // get diffuse lighting intensity
        return gl_Vertex;
    }

    virtual bool fragment(Vec3f bar, TGAColor &color) {
        // bar 表示重心差值坐标
        // 差值强度
        // varying_intensity[0]*bar[0]+varying_intensity[1]*bar[1]+varying_intensity[2]*bar[2]
        float intensity = varying_intensity*bar;   // interpolate intensity for the current pixel
        // color = TGAColor(255, 255, 255)*intensity; // well duh
        Vec2f uv = varying_uv*bar;
        color = model->diffuse(uv)*intensity;
        // color = model->diffuse(uv);
        return false;                              // no, we do not discard this pixel
        // float intensity = varying_intensity*bar;
        // if (intensity>.85) intensity = 1;
        // else if (intensity>.60) intensity = .80;
        // else if (intensity>.45) intensity = .60;
        // else if (intensity>.30) intensity = .45;
        // else if (intensity>.15) intensity = .30;
        // else intensity = 0;
        // color = TGAColor(255, 155, 0)*intensity;
        // return false;
    }
};

struct GouraudShader_2 : public IShader {
    // GouraudShader 用的是顶点与光照之间的强度
    // GouraudShader_2 直接用index中纹理的强度 model->normal(uv) 纹理哪个地方的强度
    Vec3f varying_intensity; // written by vertex shader, read by fragment shader
    mat<2,3,float> varying_uv;
    mat<4,4,float> uniform_M;   //  Projection*ModelView
    mat<4,4,float> uniform_MIT; // (Projection*ModelView).invert_transpose()

    virtual Vec4f vertex(int iface, int nthvert) {
        varying_uv.set_col(nthvert, model->uv(iface, nthvert));
        Vec4f gl_Vertex = embed<4>(model->vert(iface, nthvert)); // read the vertex from .obj file
        gl_Vertex = Viewport*Projection*ModelView*gl_Vertex;     // transform it to screen coordinates
        return gl_Vertex;
    }

    virtual bool fragment(Vec3f bar, TGAColor &color) {
        // bar 表示重心差值坐标
        // 差值强度
        // varying_intensity[0]*bar[0]+varying_intensity[1]*bar[1]+varying_intensity[2]*bar[2]
        // color = TGAColor(255, 255, 255)*intensity; // well duh
        Vec2f uv = varying_uv*bar;
        Vec3f n = proj<3>(uniform_MIT*embed<4>(model->normal(uv))).normalize();
        Vec3f l = proj<3>(uniform_M  *embed<4>(light_dir        )).normalize();
        float intensity = std::max(0.f, n*l);
        color = model->diffuse(uv)*intensity;
        return false;                              // no, we do not discard this pixel
    }
};

int main(int argc, char** argv) {
    if (2==argc) {
        model = new Model(argv[1]);
    } else {
        model = new Model("obj/african_head.obj");
    }

    lookat(eye, center, up);
    viewport(width/8, height/8, width*3/4, height*3/4);
    projection(-1.f/(eye-center).norm());
    light_dir.normalize();

    TGAImage image  (width, height, TGAImage::RGB);
    TGAImage zbuffer(width, height, TGAImage::GRAYSCALE);

    // t1
    // GouraudShader shader;

    // t2
    GouraudShader_2 shader;
    shader.uniform_M   =  Projection*ModelView;
    shader.uniform_MIT = (Projection*ModelView).invert_transpose();

    // t3
    for (int i=0; i<model->nfaces(); i++) {
        Vec4f screen_coords[3];
        for (int j=0; j<3; j++) {
            screen_coords[j] = shader.vertex(i, j);
        }
        triangle(screen_coords, shader, image, zbuffer);
    }

    image.  flip_vertically(); // to place the origin in the bottom left corner of the image
    zbuffer.flip_vertically();
    image.  write_tga_file("output.tga");
    zbuffer.write_tga_file("zbuffer.tga");

    delete model;
    return 0;
}
