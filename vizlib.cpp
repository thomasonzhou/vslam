#include "vizlib.h"
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pangolin/pangolin.h>
#include <unistd.h>

using namespace Eigen;

void pangolin_draw(const std::vector<Isometry3d, aligned_allocator<Isometry3d>>& poses);
void draw_axes(const Isometry3d& pose);
void connect_axes(const Isometry3d& pose1, const Isometry3d& pose2);
void draw_line(const Vector3d& v1, const Vector3d& v2);

void visualize_trajectory(){
    std::string trajectory_file = "../trajectory.txt";

    std::vector<Isometry3d, aligned_allocator<Isometry3d>> poses;

    std::ifstream fin(trajectory_file);
    if (!fin){
        std::cout << "file not found: " << trajectory_file << std::endl;
        return;
    }

    double time, tx, ty, tz, qx, qy, qz, qw;
    while(fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw){
        Isometry3d Twr(Quaterniond(qw, qx, qy, qz));
        Twr.pretranslate(Vector3d(tx, ty, tz));
        poses.push_back(Twr);
    }
    std::cout << poses.size() << " poses" << std::endl;

    pangolin_draw(poses);
}

void pangolin_draw(const std::vector<Isometry3d, aligned_allocator<Isometry3d>>& poses){

    constexpr float view_w = 1024.0f;
    constexpr float view_h = 768.0f;
    pangolin::CreateWindowAndBind("Trajectory", view_w, view_h);
    glEnable(GL_DEPTH_TEST); // enable 3D depth buffer for occlusion
    glEnable(GL_BLEND); // enable translucent objects
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // weighted blend for translucent objects

    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(view_w, view_h, 500, 500, 512, 389, 0.1, 1000), // intrinsics
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0) // extrinsics
    );

    pangolin::View &d_cam = pangolin::CreateDisplay() 
        .SetBounds(0.0, 1.0, 0.0, 1.0, -view_w/view_h)
        .SetHandler(new pangolin::Handler3D(s_cam));
    
    constexpr float white = 1.0f;
    while(!pangolin::ShouldQuit()){
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam); // use s_cam for display
        glClearColor(white, white, white, white);
        glLineWidth(2);

        for (size_t i = 0; i < poses.size(); ++i){
            draw_axes(poses[i]);
            if (i > 0){
                connect_axes(poses[i - 1], poses[i]);
            }
        }

        pangolin::FinishFrame();
        usleep(5000);
    }
}

void draw_axes(const Isometry3d& pose){
    constexpr double axisLength = 0.1;
    const Vector3d Ow = pose.translation();
    const Vector3d Ox = pose * (axisLength * Vector3d(1.0, 0, 0));
    const Vector3d Oy = pose * (axisLength * Vector3d(0, 1.0, 0));
    const Vector3d Oz = pose * (axisLength * Vector3d(0, 0, 1.0));
    glBegin(GL_LINES);
    glColor3f(1.0, 0.0, 0.0);
    draw_line(Ow, Ox);
    glColor3f(0.0, 1.0, 0.0);
    draw_line(Ow, Oy);
    glColor3f(0.0, 0.0, 1.0);
    draw_line(Ow, Oz);
    glEnd();
}

void connect_axes(const Isometry3d& pose1, const Isometry3d& pose2){
    const Vector3d t1 = pose1.translation();
    const Vector3d t2 = pose2.translation();
    glBegin(GL_LINES);
    glColor3f(0.0f, 0.0f, 0.0f);
    draw_line(t1, t2);
    glEnd();
}

void draw_line(const Vector3d& v1, const Vector3d& v2){
    glVertex3d(v1[0], v1[1], v1[2]);
    glVertex3d(v2[0], v2[1], v2[2]);
}