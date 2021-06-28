#include "body.hpp"
#include <Eigen/core>
#include <Eigen/dense>


void Body::solveIK( int target, const glm::vec3& targetP ) {
	std::vector<int> ancestors = getAncestors(target);
    int num = ancestors.size() + 1;
    using namespace Eigen;
    using namespace glm;
    MatrixXf J = MatrixXf(3, num * 3);
    MatrixXf b = MatrixXf(3, 1);
    const vec3 axes[] = { {1,0,0},{0,1,0},{0,0,1} };
    
    for (auto iter = 0; iter < 100; iter++) {
        for (int i = 0; i < num; i++) { //joint index

            vec3 p = links[target].getPos() - links[i].parentGlobalP;

            for (int j = 0; j < 3; j++) { //axis index

                vec3 v = cross(axes[j], p);
                /*J(0, i * 3 + j) = v.x;
                J(1, i * 3 + j) = v.y;
                J(2, i * 3 + j) = v.z;*/ // æ∆∑°∂˚ ∞∞¿Ω
                J.col(i * 3 + j) << v.x, v.y, v.z;
                
            }
        }
        vec3 d = targetP - links[target].getPos();
        b << d.x, d.y, d.z;
        //solve
        auto solver = J.bdcSvd(ComputeThinU|ComputeThinV);
        solver.setThreshold(0.01); // singular πÊ¡ˆ
        auto x = solver.solve(b);

        for (int i = 0; i < num; i++) { //joint index
            for (int j = 0; j < 3; j++) { //axis index
                links[i].rotate(exp(quat(0, 0.01f*axes[j] * x(i * 3 + j))));
            }
        }
        updatePos();
    }

}
