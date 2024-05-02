#include "DynamicObject.h"
#include "Utility.h"
//#include "CollisionPlane.h"

/*! \brief Brief description.
*  Physics dynamic object class is derived from the GameObject class, as a one type/class of game objects
*  It sets up parameters for physics computation and calls certain physics algorithms.
*  It returns the position and orientation of the object for the visualisation engine to display.
*  It is important to not include any graphics drawings in this class. This is the principle of the separation
*  of physics computation from graphics
*
*/

std::vector<DynamicObject*> DynamicObject::object_list;

DynamicObject::DynamicObject()
{
	// Set initial values for physics parameters
	// No forces act on the object to start with
	_force = glm::vec3(0.0f, 0.0f, 0.0f);
	_velocity = glm::vec3(0.0f, 0.0f, 0.0f);
	_mass = 1.0f;
	_bRadius = 1.0f;
	_previous_position = glm::vec3(0.0f);

	// Angular parameters
	_torque = glm::vec3(0.0f,0.0f,0.0f);
	_angular_velocity = glm::vec3(0.0f,0.0f,0.0f);
	_angular_momentum = glm::vec3(0.0f,0.0f,0.0f);

	_R = glm::mat3(
		1.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 1.0f
	);

	global_damping = 12.0f;

	setMass(1.0f);
	object_list.push_back(this);
	//collision_list.clear();
}

DynamicObject::~DynamicObject()
{
	auto index = std::find(object_list.begin(), object_list.end(), this);
	if (index != object_list.end())
	{
		object_list.erase(index);
		object_list.shrink_to_fit();
	}
}

void DynamicObject::setMass(float m)
{
	_mass = m;

	setInertiaTensor();
}

void DynamicObject::setInertiaTensor()
{
	glm::mat3 body_inertia;

	body_inertia = glm::mat3{
		(2.0f / 5.0f) * _mass * std::pow(_bRadius, 2),0,0,
		0,(2.0f / 5.0f) * _mass * std::pow(_bRadius, 2),0,
		0,0,(2.0f / 5.0f) * _mass * std::pow(_bRadius, 2)
	};

	_body_inertia_tensor_inverse = glm::inverse(body_inertia);
	computeInverseInertiaTensor();
}

void DynamicObject::computeInverseInertiaTensor()
{
	_inertia_tensor_inverse = _R * _body_inertia_tensor_inverse * glm::transpose(_R);
}

glm::vec3 DynamicObject::computeTorque(glm::vec3 torque_arm, glm::vec3 contact_force)
{
	glm::vec3 torque = (glm::cross(contact_force, torque_arm));
	return torque;
}

glm::vec3 DynamicObject::frictionForce(glm::vec3 relative_velocity, glm::vec3 contact_normal, glm::vec3 force_normal, float mu)
{
	glm::vec3 friction_force;
	glm::vec3 forward_relative_velocity = relative_velocity - glm::dot(relative_velocity, contact_normal) * contact_normal;
	float tangent_length = glm::length(forward_relative_velocity);
	if (tangent_length > 1e-6f)
	{
		glm::vec3 forward_direction = glm::normalize(forward_relative_velocity);

		glm::vec3 friction_direction = -forward_direction;
		friction_force = friction_direction * mu * glm::length(force_normal);
		return friction_force;
	}
	else return glm::vec3(0);
}

void DynamicObject::update(float deltaTs)
{

	/** This function is the work horse of our physics simulation
	* it implements physics simulation algorithm:
	* the basic idea is for each update with a time step size- dt
	* we use F = ma to update the state of every rigid body.
	* Please refer to my lecture notes for more details.
	* The computation consists the following steps
	* Step 1: Clear all the forces act on the object in the previous time step
	* ClearForces();
	* Step 2: Compute all the forces act on the object at the current time step
	* ComputeForces();
	* Step 3: Compute collision response
	* CollisionResponses(dt);
	* Step 4: One step time integration for simulation update !
	* TimeIntegration(dt);
	*/
	
	// Step 1: Clear all the forces act on the object
	clearForces();
	clearTorque();
	// Step 2: Compute each of forces acts on the object
	// Only gravitational force at the moment
	glm::vec3 force = _mass * _acceleration;
	// Add the force to the total force
	addForce(force);

	// Step 3: Compute collisions and responses
	// note: this collision detection is a temp fix for now
	// We will show how to detect collisions in the following lectures

	computeCollisionRes(deltaTs);
	for (DynamicObject* obj : object_list) if (obj != this) spheresCollisionResponse(obj);

	// Step 4:  Integration
	euler(deltaTs); 
	//rk2(deltaTs);  
	//rk4(deltaTs);  
	//verlet(deltaTs);
	
	setPosition(_position.x, _position.y, _position.z);
	glm::vec3 euler_angles = getEulerAngles(_R);
	float degree = 180.0f / 3.141596f;
	setRotation(euler_angles.x* degree, euler_angles.y*degree, euler_angles.z* degree);
	
}


void DynamicObject::computeCollisionRes(float deltaTs)
{
	glm::vec3 n = glm::vec3(0.0f,1.0f,0.0f);
	glm::vec3 p = _position;
	glm::vec3 q = glm::vec3(0.0f,0.0f,0.0f);
	float r = getBoundingRadius();
	float elasticity = getElasticity() * 1.0f;

	float d = PFG::DistanceToPlane(n, p, q);
	if (d <= r)
	{
		float j_numerator = -(1+elasticity) * glm::dot(getVelocity(), n);
		float total_inverse_mass = (1.0f / _mass);
		float j = j_numerator / total_inverse_mass;
		glm::vec3 impulseVector = j * n;
		glm::vec3 velocity = getVelocity() + impulseVector * (1.0f/_mass);
		setVelocity(velocity);

		glm::vec3 contact_force = glm::vec3(0.0f,9.8f * _mass, 0.0f);
		addForce(contact_force);
	}


}

/*!\brief: Numerical integration function to compute the current velocity and the current position
* based on the velocity and the position of the previous time step
*/
void  DynamicObject::euler(float deltaTs)
{
	// This function is the numerical integration the dynamic physics computation
	float oneOverMass = 1 / _mass;
	// Compute the current velocity based on the previous velocity
	_velocity += (_force * oneOverMass) * deltaTs;
	// Compute the current position based on the previous position
	_position += _velocity * deltaTs;

	_angular_momentum += _torque * deltaTs;
	computeInverseInertiaTensor();

	_angular_velocity = _inertia_tensor_inverse * _angular_momentum;

	glm::mat3 omega_star = glm::mat3(0.0f, -_angular_velocity.z, _angular_velocity.y, _angular_velocity.z, 0.0f, -_angular_velocity.x, -_angular_velocity.y, _angular_velocity.x, 0.0f);
	
	_R += omega_star * _R * deltaTs;
}

void DynamicObject::rk2(float deltaTs)
{

	glm::vec3 force;
	glm::vec3 acceleration;
	glm::vec3 k0;
	glm::vec3 k1;

	// Evaluate once at t0
	force = _force;
	acceleration = force / _mass;
	k0 = acceleration * deltaTs;

	// Evaluate once at t0 + deltaT/2.0 using half of k0
	force = _force + k0 / 2.0f;
	acceleration = force / _mass;
	k1 = acceleration * deltaTs;

	// Evaluate once at t0 + deltaT using k1
	_velocity += k1;
	_position += _velocity * deltaTs;
}

void DynamicObject::rk4(float deltaTs)
{

	glm::vec3 force;
	glm::vec3 acceleration;
	glm::vec3 k0;
	glm::vec3 k1;
	glm::vec3 k2;
	glm::vec3 k3;

	// Evaluate once at t0 to find k0
	force = _force;
	acceleration = force / _mass;
	k0 = deltaTs * acceleration;

	// Evaluate twice at t0 + deltaT/2.0 using half of k0 and half of k1
	force = _force + k0 / 2.0f;
	acceleration = force / _mass;
	k1 = deltaTs * acceleration;

	force = _force + k1 / 2.0f;
	acceleration = force / _mass;
	k2 = deltaTs * acceleration;

	// Evaluate once at t0 + deltaT using k2
	force = _force + k2;
	acceleration = force / _mass;
	k3 = deltaTs * acceleration;

	// Evaluate at t0 + deltaT using weighted sum of k0, k1, k2, and k3
	_velocity += (k0 + 2.0f * k1 + 2.0f * k2 + k3) / 6.0f;
	// Update position
	_position += _velocity * deltaTs;
}

void DynamicObject::verlet(float deltaTs)
{   /* Equations for your reference 
	*https://www.physics.udel.edu/~bnikolic/teaching/phys660/numerical_ode/node5.html
	*/
	glm::vec3 acceleration = _force / _mass;
	_previous_position = _position - _velocity * deltaTs + 0.5f * acceleration * deltaTs * deltaTs;
	_position = 2.0f * _position - _previous_position + acceleration * deltaTs * deltaTs;
	_velocity = (_position - _previous_position) / (2.0f * deltaTs);
	_velocity += acceleration * deltaTs;

}

void DynamicObject::spheresCollisionResponse(DynamicObject* _other)
{
	if (!_other)
	{
		return;
	}

	float elasticity = 0.66f;
	glm::vec3 position_distance = _position - _other->getPosition();
	glm::vec3 n = glm::normalize(position_distance);
	float r1 = _bRadius;
	float r2 = _other->getBoundingRadius();
	glm::vec3 other_velocity = _other->getVelocity();
	glm::vec3 velA = _velocity;
	glm::vec3 velB = other_velocity;
	glm::vec3 relative_velocity = velA - velB;
	glm::vec3 cp;

	float distance = glm::length(position_distance);
	bool collision = PFG::SphereToSphereCollision(_position, _other->getPosition(), r1, r2, cp);
	if (collision)
	{
		applyImpulseResponses(this, _other);
		float penetration = abs((r1 + r2) - distance);
		float _mass2 = _other->getMass();
		float inverse_mass1 = 1.0f/_mass;
		float inverse_mass2 = 1.0f/_mass2;
		float total_inverse_mass = inverse_mass1 + inverse_mass2;
		float total_mass = _mass + _mass2;

		glm::vec3 mov = _position + penetration * n * inverse_mass1 / total_inverse_mass;
		setPosition(mov.x, mov.y, mov.z);
		mov = _other->getPosition() + penetration * n * inverse_mass2 / total_inverse_mass;
		_other->setPosition(mov.x, mov.y, mov.z);

	}
}

void DynamicObject::applyImpulseResponses(DynamicObject* objA, DynamicObject* objB)
{
	glm::vec3 position_distance = objA->getPosition() - objB->getPosition();
	glm::vec3 n = glm::normalize(position_distance);
	float rA = objA->getBoundingRadius();
	float rB = objB->getBoundingRadius();
	glm::vec3 velocity_A = objA->getVelocity();
	glm::vec3 velocity_B = objB->getVelocity();
	glm::vec3 relative_velocity = velocity_A - velocity_B;

	float elasticity = objA->getElasticity() * objB->getElasticity(); // should be non - hardcoded
	float one_over_massA = 1.0f / objA->getMass();
	float one_over_massB = 1.0f / objB->getMass();
	float J_numerator = -(1.0f + elasticity) * glm::dot(relative_velocity,n);
	float total_inverse_mass = one_over_massA + one_over_massB;
	float J = J_numerator / total_inverse_mass;

	glm::vec3 collision_impulse_vector = J * n;

	glm::vec3 velocity = objA->getVelocity() + collision_impulse_vector * one_over_massA;
	objA->setVelocity(velocity);

	velocity = objB->getVelocity() - collision_impulse_vector * one_over_massB;
	objB->setVelocity(velocity);

}

void DynamicObject::planeCollisionResponse(CollisionPlane _plane, float deltaTs)
{
	float elasticity = 0.5f;
	glm::vec3 n = _plane.getNormal();
	glm::vec3 c0 = _position;
	glm::vec3 q = _plane.getPoint();
	glm::vec3 c1 = _position + _velocity * deltaTs;
	glm::vec3 ci(0);
	float r = getBoundingRadius();
	float inverse_mass1 = 1.0f / _mass;
	float inverse_mass2 = 0.0f;
	glm::vec3 velA = _velocity + glm::cross(_angular_velocity, r * n);
	glm::vec3 velB = glm::vec3(0.0f,0.0f,0.0f);
	glm::vec3 relative_velocity = velA - velB;

	glm::vec3 cross1 = glm::cross(r * n, n);
	cross1 = glm::cross(_inertia_tensor_inverse * cross1, r * n);
	float total_inverse_mass = inverse_mass1 + inverse_mass2 + glm::dot(cross1,n);

	bool collision = PFG::MovingSphereToPlaneCollision(n, c0, c1, q, r, ci);

	if (collision)
	{
		glm::vec3 floor_velocity = glm::vec3(0.0f,0.0f,0.0f);
		float collision_impulse = -(1 + elasticity) * glm::dot(relative_velocity, n) / total_inverse_mass;
		glm::vec3 collision_impulse_vector = collision_impulse * n;
		_velocity += collision_impulse_vector / _mass;
		_angular_velocity += _inertia_tensor_inverse * glm::cross(r * n, collision_impulse_vector);
		glm::vec3 gravity_force = glm::vec3(0.0f, 9.8f * _mass, 0.0f);
		glm::vec3 normal_force = glm::dot(gravity_force,n) * n;
		addForce(normal_force);

		float d_mu = 0.66f;
		glm::vec3 forward_relative_velocity = relative_velocity - glm::dot(relative_velocity, n) * n;

		glm::vec3 friction_force = frictionForce(relative_velocity, n, normal_force, d_mu);
		glm::vec3 torque_arm = r * n;
		glm::vec3 torque = computeTorque(torque_arm, friction_force);

		torque -= _angular_momentum * global_damping;
		addForce(friction_force);

		if (glm::length(forward_relative_velocity) - glm::length(friction_force / _mass) * deltaTs > 0.0f)
		{
			addForce(-friction_force);
			addTorque(torque);
		}
	}
}

glm::vec3 DynamicObject::getEulerAngles(glm::mat3 R)
{
	glm::vec3 angles;
	
	float value = R[0][0] * R[0][0] + R[1][0] * R[1][0];
	float sy = sqrt(value);

	bool singular = sy < 1e-6;

	float x,y,z;
	if (!singular)
	{
		x = atan2(R[2][1], R[2][2]);
		y = atan2(-R[2][0], sy);
		z = atan2(R[1][0], R[0][0]);
	}
	else
	{
		std::cout << "&&&&&&&&&&" << std::endl;
		x = atan2(-R[1][2], R[1][1]);
		y = atan2(-R[2][0], sy);
		z = 0;
	}

	angles = glm::vec3(x,y,z);
	return angles;
}