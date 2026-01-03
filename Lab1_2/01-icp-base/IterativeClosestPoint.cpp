#define GLM_ENABLE_EXPERIMENTAL 

#include <iostream>
#include <algorithm>
#include <cmath> 
#include <numeric> 
#include "IterativeClosestPoint.h"
#include "NearestNeighbors.h" 
#include "PointCloud.h" 
#include <Eigen/Dense>
#include <Eigen/LU>
#include <Eigen/SVD> 
#include <Eigen/Eigenvalues> 
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/norm.hpp> 

using namespace std; 

void IterativeClosestPoint::setClouds(PointCloud *pc1, PointCloud *pc2)
{
    cout << "inizializzo le nuvole..." << endl;
    cloud1 = pc1;
    cloud2 = pc2;
    knn.setPoints(&(cloud1->getPoints()));
    
    isBorder.assign(cloud1->getPoints().size(), false); 
    T_total = glm::mat4(1.0f); 
    
    if (cloud1 != NULL && cloud2 != NULL) {
        cout << "nuvole caricate" << endl;
    } else {
        cout << "puntatori nulli!" << endl;
    }
}

void IterativeClosestPoint::markBorderPoints()
{
    const std::vector<glm::vec3>& P = cloud1->getPoints();
    float pi_greco = 3.14159265359f;
    float SOGLIA = pi_greco / 2.0f; 

    isBorder.assign(P.size(), false); 

    for (size_t i = 0; i < P.size(); ++i)
    {
        std::vector<size_t> indici;
        std::vector<float> distanze;
        knn.getKNearestNeighbors(P[i], 15, indici, distanze);
        
        int n_vicini = indici.size();
        
        if (n_vicini < 3) {
            // salto perche sono troppo pochi
            continue;
        } else {
            // pca
            Eigen::MatrixXd M(n_vicini, 3);
            Eigen::Vector3d baricentro(0,0,0);
            
            for (int j = 0; j < n_vicini; ++j) {
                glm::vec3 p_tmp = P[indici[j]];
                M(j, 0) = (double)p_tmp.x;
                M(j, 1) = (double)p_tmp.y;
                M(j, 2) = (double)p_tmp.z;
                baricentro(0) += M(j, 0);
                baricentro(1) += M(j, 1);
                baricentro(2) += M(j, 2);
            }
            baricentro /= (double)n_vicini;

            Eigen::MatrixXd M_centrale = M.rowwise() - baricentro.transpose();
            Eigen::Matrix3d MAT_COV = M_centrale.transpose() * M_centrale;
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> risolutore(MAT_COV);

            // axes 
            Eigen::Vector3d v_min = risolutore.eigenvectors().col(0); 
            Eigen::Vector3d v_mid = risolutore.eigenvectors().col(1); 
            Eigen::Vector3d v_max = risolutore.eigenvectors().col(2); 
            
            std::vector<double> angoli_polar;
            for (int j = 0; j < n_vicini; ++j) {
                glm::vec3 vettore_diff = P[indici[j]] - P[i]; 
                Eigen::Vector3d d_e((double)vettore_diff.x, (double)vettore_diff.y, (double)vettore_diff.z);

                double val_x = d_e.dot(v_max);
                double val_y = d_e.dot(v_mid);
                
                double res_atan = atan2(val_y, val_x);
                angoli_polar.push_back(res_atan);
            }

            sort(angoli_polar.begin(), angoli_polar.end());
            
            double max_diff = 0.0;
            for (size_t j = 0; j < angoli_polar.size() - 1; ++j) {
                double d = angoli_polar[j+1] - angoli_polar[j];
                if (d > max_diff) {
                    // aggiorno il massimo
                    max_diff = d;
                } else {
                    // nulla
                }
            }
            
            double d_fine = 2.0 * 3.14159265359 - (angoli_polar.back() - angoli_polar.front());
            if (d_fine > max_diff) {
                max_diff = d_fine;
            }

            if (max_diff > (double)SOGLIA) {
                // bordo
                isBorder[i] = true;
            } else {
                // non bordo
                isBorder[i] = false;
            }
        }
    }
}

std::vector<int> *IterativeClosestPoint::computeCorrespondence()
{
    const std::vector<glm::vec3>& P = cloud1->getPoints();
    const std::vector<glm::vec3>& Q = cloud2->getPoints();
    
    std::vector<int>* risultato_corr = new std::vector<int>(Q.size(), -1); 
    P_corr.clear();
    Q_corr.clear();

    for (size_t i = 0; i < Q.size(); ++i) {
        std::vector<size_t> i_vicini; 
        std::vector<float> d_vicini;
        knn.getKNearestNeighbors(Q[i], 1, i_vicini, d_vicini);
        
        if (i_vicini.size() > 0) {
            int bersaglio = (int)i_vicini[0];
            (*risultato_corr)[i] = bersaglio;
            P_corr.push_back(P[bersaglio]);
            Q_corr.push_back(Q[i]);
        } else {
            // non dovrebbe succedere
            (*risultato_corr)[i] = -1;
        }
    }

    if (P_corr.size() == 0) {
        // debug per errore
        cout << "nessuna coppia trovata!" << endl;
        delete risultato_corr;
        return NULL;
    } else {
        return risultato_corr;
    }
}

glm::mat4 IterativeClosestPoint::computeICPStep()
{
    int tot_punti = P_corr.size();
    if (tot_punti < 3) {
        // pochi punti
        cout << "errore: servono almeno 3 punti" << endl;
        return glm::mat4(1.0f); 
    } else {
        // calcolo centroids
        Eigen::Vector3d cP(0,0,0);
        Eigen::Vector3d cQ(0,0,0);
        Eigen::MatrixXd mP(tot_punti, 3);
        Eigen::MatrixXd mQ(tot_punti, 3);

        for (int i = 0; i < tot_punti; ++i) {
            mP(i, 0) = (double)P_corr[i].x;
            mP(i, 1) = (double)P_corr[i].y;
            mP(i, 2) = (double)P_corr[i].z;
            mQ(i, 0) = (double)Q_corr[i].x;
            mQ(i, 1) = (double)Q_corr[i].y;
            mQ(i, 2) = (double)Q_corr[i].z;
            
            cP(0) += mP(i, 0); cP(1) += mP(i, 1); cP(2) += mP(i, 2);
            cQ(0) += mQ(i, 0); cQ(1) += mQ(i, 1); cQ(2) += mQ(i, 2);
        }
        
        cP /= (double)tot_punti;
        cQ /= (double)tot_punti;

        Eigen::MatrixXd mP_c = mP.rowwise() - cP.transpose();
        Eigen::MatrixXd mQ_c = mQ.rowwise() - cQ.transpose();
        
        Eigen::Matrix3d MAT_S = mQ_c.transpose() * mP_c; 

        Eigen::JacobiSVD<Eigen::Matrix3d> svd_obj(MAT_S, Eigen::ComputeFullU | Eigen::ComputeFullV);
        
        Eigen::Matrix3d u_mat = svd_obj.matrixU();
        Eigen::Matrix3d v_mat = svd_obj.matrixV();

        Eigen::Matrix3d rot_eigen = v_mat * u_mat.transpose();
        
        if (rot_eigen.determinant() < 0.0) {
            // correzione per riflessione
            cout << "fix per riflessione..." << endl;
            v_mat(0, 2) *= -1.0; v_mat(1, 2) *= -1.0; v_mat(2, 2) *= -1.0; 
            rot_eigen = v_mat * u_mat.transpose();
        } else {
            // rotazione ok
        }
        
        Eigen::Vector3d t_eigen = cP - rot_eigen * cQ;

        glm::mat4 T_finale = glm::mat4(1.0f);
        
        for(int r=0; r<3; r++) {
            for(int c=0; c<3; c++) {
                T_finale[c][r] = (float)rot_eigen(r,c);
            }
        }
        
        T_finale[3][0] = (float)t_eigen.x();
        T_finale[3][1] = (float)t_eigen.y();
        T_finale[3][2] = (float)t_eigen.z();

        return T_finale;
    }
}

std::vector<int> *IterativeClosestPoint::computeFullICP(unsigned int max_iter)
{
    cout << "parto con icp totale..." << endl;
    std::vector<int> *c_vecchie = NULL; 

    for (unsigned int n = 0; n < max_iter; ++n) {
        cout << "step: " << n << endl;
        
        std::vector<int> *c_nuove = computeCorrespondence();
        if (c_nuove == NULL) {
            break; 
        } else {
            // controllo se le corrispondenze sono uguali a prima
            if (c_vecchie != NULL) {
                bool sono_uguali = true;
                for(size_t j=0; j<c_nuove->size(); ++j) {
                    if((*c_nuove)[j] != (*c_vecchie)[j]) {
                        sono_uguali = false;
                        break;
                    }
                }
                if (sono_uguali == true) {
                    cout << "finito per convergenza" << endl;
                    delete c_nuove;
                    break;
                }
            }
        }

        glm::mat4 step_t = computeICPStep();
        
        // muovo la nuvola 2
        cloud2->transform(step_t);
        T_total = step_t * T_total; 
        
        if (c_vecchie != NULL) {
            delete c_vecchie;
        }
        c_vecchie = c_nuove;
    }
    
    cout << "icp terminato" << endl;
    return c_vecchie;
}
