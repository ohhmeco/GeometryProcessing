#include "NormalEstimator.h"
#include "NearestNeighbors.h"
#include <Eigen/Dense>
#include <vector>
#include <glm/glm.hpp>

void NormalEstimator::computePointCloudNormals(const std::vector<glm::vec3> &points, std::vector<glm::vec3> &normals)
{
    NearestNeighbors nn; 
    nn.setPoints(&points); 
    const unsigned int K = 15; 
    for (size_t i = 0; i < points.size(); ++i)
    {
        std::vector<size_t> neighborIndices_size_t; 
        std::vector<float> dists_squared;
        //  knn
        unsigned int num_results = nn.getKNearestNeighbors(points[i], K, neighborIndices_size_t, dists_squared);

        Eigen::MatrixXd neighbors(num_results, 3);
        Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
        
        for (size_t j = 0; j < num_results; ++j)
        {
            const glm::vec3& p = points[neighborIndices_size_t[j]];
            neighbors.row(j) << (double)p.x, (double)p.y, (double)p.z;
            centroid += neighbors.row(j);
        }
        centroid /= (double)num_results;

        // centramento
        for (int row = 0; row < neighbors.rows(); ++row)
        {
            neighbors.row(row) -= centroid.transpose();
        }

        Eigen::Matrix3d covarianceMatrix = neighbors.transpose() * neighbors;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigenSolver(covarianceMatrix);
        Eigen::Vector3d normal_eigen = eigenSolver.eigenvectors().col(0);
        
        glm::vec3 normal_glm((float)normal_eigen.x(), (float)normal_eigen.y(), (float)normal_eigen.z());
        normals[i] = glm::normalize(normal_glm);
        
        // coerenza orientamento
        if (glm::dot(normals[i], points[i]) > 0.0f)
        {
             normals[i] = -normals[i];
        }
    }
}
