#include <iostream>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>
#include "LaplacianSmoothing.h"

void LaplacianSmoothing::setMesh(TriangleMesh *newMesh)
{
    mesh = newMesh;
}

/* This method should apply nIterations iterations of the laplacian vector multiplied by lambda 
   to each of the vertices. */

void LaplacianSmoothing::iterativeLaplacian(int nIterations, float lambda)
{
    int n = mesh->getVertices().size();
    for (int it = 0; it < nIterations; ++it) {
        vector<glm::vec3> delta(n, glm::vec3(0.0f));
        vector<glm::vec3>& P = mesh->getVertices();

        for (int i = 0; i < n; ++i) {
            vector<unsigned int> neighbors;
            mesh->getNeighbors(i, neighbors);
            if (neighbors.empty()) continue;

            glm::vec3 sum(0.0f);
            for (unsigned int neighbor : neighbors) sum += P[neighbor];
            delta[i] = (sum / (float)neighbors.size()) - P[i];
        }

        for (int i = 0; i < n; ++i) {
            P[i] += lambda * delta[i];
        }
    }
}

/* This method should apply nIterations iterations of the bilaplacian operator using lambda 
   as a scaling factor. */

void LaplacianSmoothing::iterativeBilaplacian(int nIterations, float lambda)
{
    int n = mesh->getVertices().size();
    for (int it = 0; it < nIterations; ++it) {
        vector<glm::vec3>& P = mesh->getVertices();
        
        // 1
        vector<glm::vec3> delta1(n, glm::vec3(0.0f));
        for (int i = 0; i < n; ++i) {
            vector<unsigned int> neighbors;
            mesh->getNeighbors(i, neighbors);
            if (neighbors.empty()) continue;
            glm::vec3 sum(0.0f);
            for (unsigned int neighbor : neighbors) sum += P[neighbor];
            delta1[i] = (sum / (float)neighbors.size()) - P[i];
        }
        for (int i = 0; i < n; ++i) P[i] += lambda * delta1[i];

        //  2
        vector<glm::vec3> delta2(n, glm::vec3(0.0f));
        for (int i = 0; i < n; ++i) {
            vector<unsigned int> neighbors;
            mesh->getNeighbors(i, neighbors);
            if (neighbors.empty()) continue;
            glm::vec3 sum(0.0f);
            for (unsigned int neighbor : neighbors) sum += P[neighbor];
            delta2[i] = (sum / (float)neighbors.size()) - P[i];
        }
        for (int i = 0; i < n; ++i) P[i] -= lambda * delta2[i];
    }
}

/* This method should apply nIterations iterations of Taubin's operator using lambda 
   as a scaling factor, and computing the corresponding nu value. */

void LaplacianSmoothing::iterativeLambdaNu(int nIterations, float lambda)
{
    float mu = lambda / (0.1f * lambda - 1.0f);
    int n = mesh->getVertices().size();

    for (int it = 0; it < nIterations; ++it) {
        vector<glm::vec3>& P = mesh->getVertices();

        // Lambda
        vector<glm::vec3> deltaL(n, glm::vec3(0.0f));
        for (int i = 0; i < n; ++i) {
            vector<unsigned int> neighbors;
            mesh->getNeighbors(i, neighbors);
            if (neighbors.empty()) continue;
            glm::vec3 sum(0.0f);
            for (unsigned int neighbor : neighbors) sum += P[neighbor];
            deltaL[i] = (sum / (float)neighbors.size()) - P[i];
        }
        for (int i = 0; i < n; ++i) P[i] += lambda * deltaL[i];

        // Mu
        vector<glm::vec3> deltaM(n, glm::vec3(0.0f));
        for (int i = 0; i < n; ++i) {
            vector<unsigned int> neighbors;
            mesh->getNeighbors(i, neighbors);
            if (neighbors.empty()) continue;
            glm::vec3 sum(0.0f);
            for (unsigned int neighbor : neighbors) sum += P[neighbor];
            deltaM[i] = (sum / (float)neighbors.size()) - P[i];
        }
        for (int i = 0; i < n; ++i) P[i] += mu * deltaM[i];
    }
}

/* This method should compute new vertices positions by making the laplacian zero, while
   maintaing the vertices marked as constraints fixed. */

void LaplacianSmoothing::globalLaplacian(const vector<bool> &constraints)
{
    int n = mesh->getVertices().size();
    Eigen::SparseMatrix<float> A(n, n);
    vector<Eigen::Triplet<float>> triplets;
    Eigen::MatrixXf b = Eigen::MatrixXf::Zero(n, 3);

    for (int i = 0; i < n; ++i) {
        if (constraints[i]) {
            triplets.push_back(Eigen::Triplet<float>(i, i, 1.0f));
            b(i, 0) = mesh->getVertices()[i].x;
            b(i, 1) = mesh->getVertices()[i].y;
            b(i, 2) = mesh->getVertices()[i].z;
        } else {
            vector<unsigned int> neighbors;
            mesh->getNeighbors(i, neighbors);
            float invDeg = 1.0f / (float)neighbors.size();
            triplets.push_back(Eigen::Triplet<float>(i, i, -1.0f));
            for (unsigned int neighbor : neighbors) {
                triplets.push_back(Eigen::Triplet<float>(i, neighbor, invDeg));
            }
        }
    }

    A.setFromTriplets(triplets.begin(), triplets.end());
    Eigen::SparseLU<Eigen::SparseMatrix<float>> solver;
    solver.compute(A);
    Eigen::MatrixXf X = solver.solve(b);

    for (int i = 0; i < n; ++i) {
        mesh->getVertices()[i] = glm::vec3(X(i, 0), X(i, 1), X(i, 2));
    }
}

/* This method has to optimize the vertices' positions in the least squares sense, 
   so that the laplacian is close to zero and the vertices remain close to their 
   original locations. The constraintWeight parameter is used to control how close 
   the vertices have to be to their original positions. */

void LaplacianSmoothing::globalBilaplacian(const vector<bool> &constraints, float constraintWeight)
{
    int n = mesh->getVertices().size();
    Eigen::SparseMatrix<float> L(n, n);
    vector<Eigen::Triplet<float>> tripletsL;

    for (int i = 0; i < n; ++i) {
        vector<unsigned int> neighbors;
        mesh->getNeighbors(i, neighbors);
        tripletsL.push_back(Eigen::Triplet<float>(i, i, -1.0f));
        float invDeg = 1.0f / (float)neighbors.size();
        for (unsigned int j : neighbors) tripletsL.push_back(Eigen::Triplet<float>(i, j, invDeg));
    }
    L.setFromTriplets(tripletsL.begin(), tripletsL.end());

    Eigen::SparseMatrix<float> A(n + n, n); 
    Eigen::MatrixXf B = Eigen::MatrixXf::Zero(n + n, 3);
    vector<Eigen::Triplet<float>> tripletsA;

    for (int k=0; k < L.outerSize(); ++k)
        for (Eigen::SparseMatrix<float>::InnerIterator it(L, k); it; ++it)
            tripletsA.push_back(Eigen::Triplet<float>(it.row(), it.col(), it.value()));

    for (int i = 0; i < n; ++i) {
        float weight = constraints[i] ? constraintWeight : 0.0f;
        tripletsA.push_back(Eigen::Triplet<float>(i + n, i, weight));
        B(i + n, 0) = mesh->getVertices()[i].x * weight;
        B(i + n, 1) = mesh->getVertices()[i].y * weight;
        B(i + n, 2) = mesh->getVertices()[i].z * weight;
    }

    A.setFromTriplets(tripletsA.begin(), tripletsA.end());
    Eigen::SparseMatrix<float> AtA = A.transpose() * A;
    Eigen::MatrixXf AtB = A.transpose() * B;
    
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<float>> solver;
    solver.compute(AtA);
    Eigen::MatrixXf X = solver.solve(AtB);

    for (int i = 0; i < n; ++i) {
        mesh->getVertices()[i] = glm::vec3(X(i, 0), X(i, 1), X(i, 2));
    }
}
