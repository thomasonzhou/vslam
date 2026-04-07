#pragma once
#include "viz/vizlib.h"
#include "util/bal.h"

namespace viz {

void pangolin_draw(const util::BALProblem &bal_problem){
    pangolin_run([&](){
        glPointSize(2.0f);
        glColor3f(0.0f, 0.0f, 0.0f);
        glBegin(GL_POINTS);
        for(size_t i = 0; i < bal_problem.num_points; ++i){
            Eigen::Map<const Eigen::Vector3d> p(bal_problem.points(i));
            draw_point(p);
        }
        glEnd();
    });
}

}  // namespace viz