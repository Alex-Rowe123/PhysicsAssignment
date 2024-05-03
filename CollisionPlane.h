
#ifndef _CollisionPlane_H_
#define _CollisionPlane_H_
#include "phy.h"
#include <glm/glm.hpp>
#include <vector>

class CollisionPlane : public Object
{
public:
    CollisionPlane();
    ~CollisionPlane();
    glm::vec3 getNormal(){return _plane_normal;}
    glm::vec3 getPoint(){return _plane_point;}
    void setNormal(glm::vec3 normal){_plane_normal = normal;}
    void setPoint(glm::vec3 point){_plane_point = point;}
private:
    glm::vec3 _plane_normal;
    glm::vec3 _plane_point;
};

struct PlaneContainer
{
    std::vector<CollisionPlane> _plane_vector;
};

#endif