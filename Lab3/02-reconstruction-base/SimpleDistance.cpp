#include "SimpleDistance.h"
#include "PointCloud.h"
#include <glm/gtx/norm.hpp> // Per glm::distance2
#include <iostream>

// I membri privati 'knn', 'cloud', 'radius' sono dichiarati in SimpleDistance.h

/* Initialize everything to be able to compute the implicit distance of [Hoppe92] 
 * at arbitrary points that are close enough to the point cloud.
 */
void SimpleDistance::init(const PointCloud *pointCloud, float samplingRadius)
{
    cloud = pointCloud;
    knn.setPoints(&pointCloud->getPoints());
    radius = samplingRadius;
    
    if (cloud->getNormals().empty() || cloud->getPoints().size() != cloud->getNormals().size()) {
        std::cerr << "ERRORE: Le normali non sono disponibili o non corrispondono ai punti. Assicurati che siano state calcolate (es. con la PCA)." << std::endl;
    }
}


/* This operator returns a boolean that if true signals that the value parameter
 * has been modified to contain the value of the implicit function of [Hoppe92]
 * at point P.
 */
bool SimpleDistance::operator()(const glm::vec3 &P, float &value) const
{
    if (cloud->getPoints().empty() || cloud->getNormals().empty()) {
        return false;
    }
    
    // Trova il punto più vicino pi e la sua normale ni 
    size_t closestIndex;
    std::vector<size_t> indices;
    std::vector<float> dists_squared;
    
    knn.getKNearestNeighbors(P, 1, indices, dists_squared);
    
    if (indices.empty()) {
        return false;
    }
    
    closestIndex = indices[0];
    const glm::vec3& pi = cloud->getPoints()[closestIndex];
    const glm::vec3& ni = cloud->getNormals()[closestIndex];
    
    glm::vec3 diff = P - pi;
    
    //  dot product: (p - pi) · ni
    float dot_product = glm::dot(diff, ni);
    
    // z <- pi - ((p - pi) · ni) · n
    glm::vec3 z = pi - dot_product * ni; 
    
    // Filtering: distance(z, pi) < radius 
    // 'radius' come raggio di campionamento (p + epsilon nel PDF)
    float dist_z_pi_sq = glm::distance2(z, pi);
    float radius_sq = radius * radius;
    
    if (dist_z_pi_sq < radius_sq) 
    {
        // Funzione definita: f(p) <- (p - pi) · ni 
        value = dot_product;
        return true;
    } 
    else 
    {
        // 5. Funzione undefined 
        return false;
    }
}


