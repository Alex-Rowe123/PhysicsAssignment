#ifndef _KinematicsObject_H_
#define _KinematicsObject_H_

#include "phy.h"
#include "glm/glm.hpp"

class KinematicsObject : public Object
{
public:

	KinematicsObject();
	~KinematicsObject();

	/** Update function to override the base class function
	*   for physics computation
	*   This function is typically organized that it calls each physics computation procedures/algorithms
	*   in a sequential order, so that each computational algorithm provides required information for the
	*   next procedure.
	*   @param float deltaTs simulation time step length
	*/
	void update(float deltaTs);
	
	void setPosition(float x, float y, float z)
	{
		_position = glm::vec3(x, y, z);
		Object::setPosition(x, y, z);
	}

	void setVelocity(glm::vec3 vel) { _velocity = vel; }
	
	void setAcceleration(glm::vec3 accel) { _acceleration = accel; }

	glm::vec3 getVelocity() { return _velocity; }

//private:

	/** Set up physics parameters for computation
	*  Specific parameters are determined by the physics simulation
	*/
	glm::vec3 _velocity;
	glm::vec3 _position;
	glm::vec3 _acceleration;


};

#endif //!_KinematicsObject_H_

