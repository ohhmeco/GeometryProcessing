#include "MongePatch.h"
#include <Eigen/Dense>
#include <iostream>

static glm::vec3 _P, _normal;
static vector<glm::vec3> _closest;

void MongePatch::init(const glm::vec3 &P, const glm::vec3 &normal, const vector<glm::vec3> &closest)
{
    _P = P;
    _normal = normal;
    _closest = closest;
}

void MongePatch::principalCurvatures(float &kmin, float &kmax) const
{
    if (_closest.empty()) {
        kmin = kmax = 0.0f;
        return;
    }

    // (u, v, w), w is normal
    glm::vec3 w = -glm::normalize(_normal); // w = -n
    
    glm::vec3 OX(1.0f, 0.0f, 0.0f);
    if (std::abs(glm::dot(OX, w)) > 0.9f) {
        OX = glm::vec3(0.0f, 1.0f, 0.0f);
    }

    glm::vec3 u = glm::normalize(glm::cross(OX, w));
    glm::vec3 v = glm::cross(w, u);

    // fitting 
    Eigen::Matrix<float, 6, 6> A = Eigen::Matrix<float, 6, 6>::Zero();
    Eigen::Matrix<float, 6, 1> B = Eigen::Matrix<float, 6, 1>::Zero();

    for (const auto& pi : _closest) {
        glm::vec3 p_rel = pi - _P;
        float ui = glm::dot(u, p_rel);
        float vi = glm::dot(v, p_rel);
        float wi = glm::dot(w, p_rel);

        // Vettore delle basi: q = [u^2, uv, v^2, u, v, 1] 
        Eigen::Matrix<float, 6, 1> q;
        q << ui*ui, ui*vi, vi*vi, ui, vi, 1.0f;

        A += q * q.transpose();
        B += wi * q;
    }

    Eigen::VectorXf s = A.ldlt().solve(B);
    float a = s(0);
    float b = s(1);
    float c = s(2);

   // H = [2a  b]
    //      [b  2c]
    Eigen::Matrix2f Hessian;
    Hessian << 2.0f * a, b,
                b, 2.0f * c;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> solver(Hessian);
    
    if (solver.info() == Eigen::Success) {
        kmin = solver.eigenvalues()(0);
        kmax = solver.eigenvalues()(1);
    } else {
        kmin = kmax = 0.0f;
    }
}
