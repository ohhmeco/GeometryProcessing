#define GLM_ENABLE_EXPERIMENTAL 
#include "RBFFunction.h"
#include "PointCloud.h"
#include <iostream>
#include <vector>
#include <cmath> 
#include <glm/gtx/norm.hpp>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>
#include "NearestNeighbors.h"

namespace RBFInternal {
    const PointCloud *inputCloud = nullptr;
    float sigma = 0.0f; 
    float supportR = 0.0f; 
    float offset_d = 0.0f; 
    std::vector<glm::vec3> artificialPoints; // p_i^+ e p_i^-
    Eigen::VectorXd coefficientsC; // c
    NearestNeighbors artKnn;  

float RBF_phi(float r, float sigma, float supportR) 
{
    if (r >= supportR) {
        return 0.0f;
    }
    // phi(r) = exp(-r^2 / 2c^2)
    return std::exp(- (r * r) / (2.0f * sigma * sigma));
}


/* Initialize everything to be able to compute the implicit distance to the reconstructed
 * point cloud at arbitrary points that are close enough to the point cloud.
 */
void RBFFunction::init(const PointCloud *pointCloud, float standardDeviation, float supportRadius)
{
    RBFInternal::inputCloud = pointCloud;
    RBFInternal::sigma = standardDeviation;
    RBFInternal::supportR = supportRadius; 
    
    RBFInternal::offset_d = supportRadius / 3.0f; 
    
    const std::vector<glm::vec3>& P = RBFInternal::inputCloud->getPoints();
    const std::vector<glm::vec3>& N = RBFInternal::inputCloud->getNormals();
    size_t n = P.size();
    
    if (n == 0 || n != N.size()) {
        std::cerr << "ERR Nuvola di punti non valida o normali mancanti." << std::endl;
        return;
    }

    size_t m = 2 * n; 
    RBFInternal::artificialPoints.clear();
    RBFInternal::coefficientsC.resize(m); 
    Eigen::VectorXd v(m); // Vettore target

    for (size_t i = 0; i < n; ++i) {
        const glm::vec3& pi = P[i];
        const glm::vec3& ni = N[i];
        
        // p_i^+ = p_i + d * n_i, f(p_i^+) = d
        glm::vec3 pi_plus = pi + RBFInternal::offset_d * ni;
        
        // p_i^- = p_i - d * n_i, f(p_i^-) = -d
        glm::vec3 pi_minus = pi - RBFInternal::offset_d * ni;
        
        RBFInternal::artificialPoints.push_back(pi_plus);
        RBFInternal::artificialPoints.push_back(pi_minus);
        
        // f(p_i) = v_i, dove v_i = +/-d
        v[2*i]      = (double)RBFInternal::offset_d;  
        v[2*i + 1] = (double)-RBFInternal::offset_d; 
    }

    Eigen::SparseMatrix<double> A(m, m);
    A.reserve(Eigen::VectorXi::Constant(m, 20)); 
    
    for (size_t i = 0; i < m; ++i) {
        for (size_t j = i; j < m; ++j) { 
            float r = glm::distance(RBFInternal::artificialPoints[i], RBFInternal::artificialPoints[j]);
            float phi_val = RBF_phi(r, RBFInternal::sigma, RBFInternal::supportR);
            
            if (std::abs(phi_val) > 1e-12f) {  //j
                A.insert(i, j) = (double)phi_val;
                if (i != j) {
                    A.insert(j, i) = (double)phi_val; 
                }
            }
        }
    }
    A.makeCompressed();

    //  A * c = v
    
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>> solver;
    solver.setTolerance(1e-8); 
    
    solver.compute(A);
    if(solver.info() != Eigen::Success) {
        std::cerr << "ERRORE: La decomposizione della matrice A è fallita." << std::endl;
        return;
    }
    
    RBFInternal::coefficientsC = solver.solve(v);
    if(solver.info() != Eigen::Success) {
        std::cerr << "ERRORE: La risoluzione del sistema A*c=v è fallita." << std::endl;
        return;
    }

    RBFInternal::artKnn.setPoints(&RBFInternal::artificialPoints);
    
    std::cout << "RBF: Sistema risolto per " << m << " punti artificiali." << std::endl;
}


bool RBFFunction::operator()(const glm::vec3 &P, float &value) const
{
    if (RBFInternal::coefficientsC.size() == 0) {
        return false;
    }
    
    std::vector<std::pair<size_t, float>> neighborPairs;
    
    RBFInternal::artKnn.getNeighborsInRadius(P, RBFInternal::supportR, neighborPairs);

    if (neighborPairs.empty()) {
        value = 0.0f; 
        return true; //f is 0 fuori dal supporto
    }
    
    double sum_f = 0.0;
    
    for (const auto& pair : neighborPairs) {
        size_t i = pair.first; 
        float dist_squared = pair.second; 
        
        float r = std::sqrt(dist_squared); 
        
        float phi_val = RBF_phi(r, RBFInternal::sigma, RBFInternal::supportR);
        
        // + contributo: phi * c_i
        sum_f += (double)phi_val * RBFInternal::coefficientsC[i];
    }

    value = (float)sum_f;
    return true;
}
