#include <iostream>
#include <cstdlib>
#include <set>
#include <map>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>
#include "Parameterizer.h"
#include "timing.h"

using namespace std;

// This method should compute new texture coordinates for the input mesh
// using the harmonic coordinates approach with uniform weights.
// The 'TriangleMesh' class has a method 'getTexCoords' that may be used 
// to access and update its texture coordinates per vertex.
void Parameterizer::harmonicCoordinates(TriangleMesh *mesh)
{
    // border edges of the input mesh
    int n = mesh->getVertices().size();
    vector<unsigned int> tris = mesh->getTriangles();
    map<pair<int, int>, int> bordi; // conteggio

    for (int i = 0; i < tris.size(); i += 3) {
        for (int j = 0; j < 3; j++) {
            int v1 = tris[i + j];
            int v2 = tris[i + (j + 1) % 3];
            bordi[make_pair(v1, v2)]++;
        }
    }

    map<int, int> succ; // prossimo
    for (map<pair<int, int>, int>::iterator it = bordi.begin(); it != bordi.end(); it++) {
        int a = it->first.first;
        int b = it->first.second;
        if (bordi.find(make_pair(b, a)) == bordi.end()) {
            succ[a] = b;
        }
    }

    if (succ.empty()) return;

    // edges into a single cycle
    vector<int> c; //util 
    int inizio = succ.begin()->first;
    int corr = inizio;
    while (true) {
        c.push_back(corr);
        corr = succ[corr];
        if (corr == inizio) break; 
    }

    // Each of the vertices on the border polyline will receive a texture coordinate
    // on the border of the parameter space ([0,0] -> [1,0] -> [1,1] -> [0,1]), using 
    // the chord length approach
    float t = 0; // totale
    vector<float> d_v; // distanze
    for (int i = 0; i < c.size(); i++) {
        int v1 = c[i];
        int v2 = c[(i + 1) % c.size()];
        float d = glm::distance(mesh->getVertices()[v1], mesh->getVertices()[v2]);
        d_v.push_back(d);
        t += d;
    }

    vector<glm::vec2>& uv = mesh->getTexCoords();
    uv.assign(n, glm::vec2(0.5, 0.5)); 
    
    vector<bool> b_v(n, false); 
    float acc = 0; //
    for (int i = 0; i < c.size(); i++) {
        int v = c[i];
        b_v[v] = true;
        float l = acc / t;
       
        if (l < 0.25)      uv[v] = glm::vec2(4.0 * l, 0.0);
        else if (l < 0.5)  uv[v] = glm::vec2(1.0, 4.0 * (l - 0.25));
        else if (l < 0.75) uv[v] = glm::vec2(1.0 - 4.0 * (l - 0.5), 1.0);
        else               uv[v] = glm::vec2(0.0, 1.0 - 4.0 * (l - 0.75));
        
        acc += d_v[i];
    }

    // We build an equation system to compute the harmonic coordinates of the 
    // interior vertices
    Eigen::SparseMatrix<float> mat(n, n); 
    vector<Eigen::Triplet<float> > list; 
    Eigen::MatrixXf b = Eigen::MatrixXf::Zero(n, 2); 

    for (int i = 0; i < n; i++) {
        if (b_v[i]) {
            list.push_back(Eigen::Triplet<float>(i, i, 1.0));
            b(i, 0) = uv[i].x;
            b(i, 1) = uv[i].y;
        } else {
            vector<unsigned int> nbs; // vicini
            mesh->getNeighbors(i, nbs);
            if (nbs.size() > 0) {
                float w = 1.0 / (float)nbs.size(); 
                list.push_back(Eigen::Triplet<float>(i, i, -1.0));
                for (int j = 0; j < nbs.size(); j++) {
                    list.push_back(Eigen::Triplet<float>(i, nbs[j], w));
                }
            } else {
                list.push_back(Eigen::Triplet<float>(i, i, 1.0));
            }
        }
    }

    // Finally, we solve the system and assign the computed texture coordinates
    // to their corresponding vertices on the input mesh
    mat.setFromTriplets(list.begin(), list.end());
    Eigen::SparseLU<Eigen::SparseMatrix<float> > s; 
    s.compute(mat);
    Eigen::MatrixXf res = s.solve(b); //

    for (int i = 0; i < n; i++) {
        uv[i].x = res(i, 0);
        uv[i].y = res(i, 1);
    }
}
