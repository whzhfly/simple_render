#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <vector>
#include "model.h"

Model::Model(const char *filename) : verts_(), faces_() {
    std::ifstream in;
    in.open (filename, std::ifstream::in);
    if (in.fail()) return;
    std::string line;
    while (!in.eof()) {
        std::getline(in, line);
        std::istringstream iss(line.c_str());
        char trash;
        if (!line.compare(0, 2, "v ")) {
            iss >> trash;
            Vec3f v;
            for (int i=0;i<3;i++) iss >> v.raw[i];
            // std::cout<<v.x<<v.y<<std::endl;
            verts_.push_back(v);
        } else if (!line.compare(0, 2, "f ")) {
            std::vector<int> f;
            std::vector<int> f_t;
            int itrash, idx, text_index;
            iss >> trash;
            while (iss >> idx >> trash >> text_index >> trash >> itrash) {
                idx--; // in wavefront obj all indices start at 1, not zero
                f.push_back(idx);
                text_index--;
                f_t.push_back(text_index);
            }
            t_faces_.push_back(f_t);
            faces_.push_back(f);
        }else if (!line.compare(0, 2, "vt")) {
            iss >> trash >>trash;
            Vec2f v;
            // std::cout<<line.c_str()<<std::endl;
            // float x,y;
            // iss>>x>>y;
            for (int i=0;i<2;i++) iss >> v.raw[i];
            // std::cout<<v.x<<v.y<<std::endl;
            texture_.push_back(v);
        }
    }
    // std::cout<<texture_.size()<<std::endl;
    // std::cout<<t_faces_.size()<<std::endl;
    // for (auto t : texture_){
    //     std::cout<<t.x<<t.y;
    // }
    std::cerr << "# v# " << verts_.size() << " f# "  << faces_.size() << std::endl;
}

Model::~Model() {
}

int Model::nverts() {
    return (int)verts_.size();
}

int Model::nfaces() {
    return (int)faces_.size();
}

std::vector<int> Model::face(int idx) {
    return faces_[idx];
}

std::vector<int> Model::face_texture(int idx) {
    return t_faces_[idx];
}

Vec2f Model::texture(int i) {
    return texture_[i];
}

Vec3f Model::vert(int i) {
    return verts_[i];
}

