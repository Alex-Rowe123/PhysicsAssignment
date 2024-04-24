#include "phy.h"
#include <glm/glm.hpp>

#include <cmath>
#include "KinematicsObject.h"
#include "DynamicObject.h"

// g++ -o main main.cpp KinematicsObject.cpp DynamicObject.cpp Utility.cpp -lGLEW -lSDL2 -lGL -g

int main()
{
	KinematicsObject a;
	DynamicObject b;
	DynamicObject c;
	KinematicsObject d;

	float dt = 1.0f / 30.0f;

	a.setType(PHY_SPHERE);
	a.setColor(1, 1, 1);
	a.setScale(5, 5, 5);
	a.setPosition(0, 0, 0);
	a.setVelocity(glm::vec3(0, 0, 0));
	a.setAcceleration(glm::vec3(0, -9.8f, 0));


	b.setType(PHY_SPHERE);
	b.setColor(0, 1, 0);
	b.setScale(6, 6, 6);
	b.setPosition(0, 15, 0);
	b.setVelocity(glm::vec3(0, 0, 0));
	b.setAcceleration(glm::vec3(0, -9.8f, 0));
	b.setBoundingRadius(3.0f);
	b.setMass(0.1f);

	c.setType(PHY_CUBE);
	c.setColor(1.0f, 0.0f, 0.0f);
	c.setPosition(1, 0.0f, 0.0f);
	c.setVelocity(glm::vec3(0.0f, 30.0f, 0.0f));
	c.setAcceleration(glm::vec3(0, -9.8f, 0));
	c.setScale(5, 5, 5);
	c.setBoundingRadius(2.5f);
	c.setMass(3.0f);

	d.setColor(0.0f, 1.0f, 0.3f);
	d.setPosition(10.f, 20.0f, 0.0f);
	d.setScale(5.0f, 5.0f, 5.0f);
	d.setType(PHY_CUBE);


    float rot = 0.0f;
	while (true)
	{
		a.update(dt);
		b.update(dt);
		c.update(dt);

		d.setPosition(10, 20.0f, 0.0f);
		d.setRotation(rot, 0.0f, 0.0f);

		rot++;
		wait(10);
	}

	return 0;
}
