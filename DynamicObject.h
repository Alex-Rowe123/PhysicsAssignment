
#ifndef _DynamicObject_H_
#define _DynamicObject_H_

#include "phy.h"
#include "glm/glm.hpp"
#include <vector>
#include <algorithm>


/*! \brief Brief description.
*  This physics dynamic object class is derived from the GameObject class, as a one type/class of game objects
*  It sets up parameters for physics computation and calls certain physics algorithms.
*  It returns the position and orientation of the object for the visualisation engine to display.
*  It is important to not include any graphics drawings in this class. This is the principle of the separation
*  of physics computation from graphics
*
*/

class DynamicObject : public Object
{
public:

	DynamicObject();
	~DynamicObject();

	/** Update function to override the base class function
	*   for physics computation
	*   This function is typically organized that it calls each physics computation procedures/algorithms
	*   in a sequential order, so that each computational algorithm provides required information for the
	*   next procedure.
	*   @param float deltaTs simulation time step length
	*/
	void update(float deltaTs);
	void addForce(const glm::vec3 force) { _force += force; }
	void clearForces() { _force = glm::vec3(0.0f, 0.0f, 0.0f); }
	void computeCollisionRes(float deltaTs);
	void spheresCollisionResponse(DynamicObject* _other);
	void applyImpulseResponses(DynamicObject* objA, DynamicObject* objB);

	/** Numerical integration: Euler's method
	**/
	void euler(float deltaTs);
	void rk2(float deltaTs);
	void rk4(float deltaTs);
	void verlet(float deltaTs);

	// Setters and getters
	void setForce(const glm::vec3 force) { _force = force; }
	void setMass(float mass);
	void setBoundingRadius(float r) { _bRadius = r; }
	void setPosition(float x, float y, float z)
	{
		_position = glm::vec3(x, y, z);
		Object::setPosition(x, y, z);
	}
	
	void setVelocity(const glm::vec3 vel) { _velocity = vel; }
	glm::vec3 getVelocity() { return _velocity; }
	void setAcceleration(glm::vec3 accel) { _acceleration = accel; }
	const glm::vec3 getForce() const { return _force; }
	const glm::vec3 getPosition() const { return _position;}
    const float getMass() const { return _mass; }
	const float getBoundingRadius() const { return _bRadius; }
	
	void addTorque(const glm::vec3 torque) {_torque += torque;}
	void clearTorque() {_torque = glm::vec3(0.0f,0.0f,0.0f);}
	void setInertiaTensor();
	void computeInverseInertiaTensor();
	glm::vec3 computeTorque(glm::vec3 torque_arm, glm::vec3 contact_force);
	glm::vec3 frictionForce(glm::vec3 relative_velocity, glm::vec3 contact_normal, glm::vec3 force_normal, float mu);
	glm::vec3 getEulerAngles(glm::mat3 R);

	static std::vector<DynamicObject*> object_list;

private:

	/** Set up physics parameters for computation
	*  Specific parameters are determined by the physics simulation
	*/
	glm::vec3 _force;
	glm::vec3 _velocity;
	glm::vec3 _acceleration;
	glm::vec3 _position;
	glm::vec3 _previous_position;
	float _mass;
	/** The radius of a bounding sphere of the object
	*/
	float _bRadius;
	glm::vec3 _torque;
	glm::vec3 _angular_velocity;
	glm::vec3 _angular_momentum;
	glm::mat3 _inertia_tensor_inverse;
	glm::mat3 _body_inertia_tensor_inverse;
	glm::mat3 _R;
	float global_damping;

	
};

#endif //!_DynamicObject_H_


