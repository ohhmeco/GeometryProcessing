#ifndef _RBF_FUNCTION_INCLUDE
#define _RBF_FUNCTION_INCLUDE

#include "ImplicitFunction.h"
#include "PointCloud.h"
#include "NearestNeighbors.h"
#include <glm/glm.hpp>


class RBFFunction : public ImplicitFunction
{

public:
    // La funzione init funge da costruttore logico
    void init(const PointCloud *pointCloud, float standardDeviation, float supportRadius);

    // L'operatore di valutazione
    bool operator()(const glm::vec3 &P, float &value) const;
    
};


#endif // _RBF_FUNCTION_INCLUDE
