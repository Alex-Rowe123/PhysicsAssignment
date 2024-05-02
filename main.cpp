#include "phy.h"
#include <glm/glm.hpp>
#include <fstream>
#include <cmath>
#include "KinematicsObject.h"
#include "DynamicObject.h"
#include "CollisionPlane.h"

// g++ -o main main.cpp KinematicsObject.cpp DynamicObject.cpp CollisionPlane.cpp Utility.cpp -lGLEW -lSDL2 -lGL -g

int stotype(std::string str);

int main()
{
	std::vector<DynamicObject*> Dynamic_objs;
	std::vector<KinematicsObject*> Kinematic_objs;
	std::vector<CollisionPlane*> Plane_objs;

	float dt = 1.0f / 120.0f;
	/*DynamicObject a;
	DynamicObject b;
	DynamicObject c;
	KinematicsObject d;

	
	a.setType(PHY_SPHERE);
	a.setColor(1, 1, 1);
	a.setScale(5, 5, 5);
	a.setPosition(20, 30, 7);
	a.setVelocity(glm::vec3(0, -9.8f, 0));
	a.setAcceleration(glm::vec3(0, -9.8f, 0));
	a.setBoundingRadius(2.5f);
	a.setMass(10.0f);
	a.setElasticity(0.8f);


	b.setType(PHY_SPHERE);
	b.setColor(0, 1, 0);
	b.setScale(5, 5, 5);
	b.setPosition(0.0f, 20.0f, 0.0f);
	b.setVelocity(glm::vec3(0, -9.8f, 0));
	b.setAcceleration(glm::vec3(0, -9.8f, 0));
	b.setBoundingRadius(2.5f);
	b.setMass(50.0f);
	b.setElasticity(0.5f);

	c.setType(PHY_SPHERE);
	c.setColor(1.0f, 0.0f, 0.0f);
	c.setPosition(4.0f, 5.0f, 0.0f);
	c.setVelocity(glm::vec3(0.0f, 20.0f, 0.0f));
	c.setAcceleration(glm::vec3(0, -9.8f, 0));
	c.setScale(5, 5, 5);
	c.setBoundingRadius(2.5f);
	c.setMass(50.0f);
	c.setElasticity(0.0f);

	d.setColor(0.0f, 1.0f, 0.3f);
	d.setPosition(10.f, 20.0f, 0.0f);
	d.setScale(5.0f, 5.0f, 5.0f);
	d.setType(PHY_CUBE);
*/
	std::vector<std::vector<std::string>> config;

	std::ifstream file("config.txt");

	if (!file.is_open())
	{
		std::cout << "File opening error" << std::endl;
		return 0;
	}

	std::string line;
	while (std::getline(file,line))
	{
		std::vector<std::string> arguments;
		std::string currentParameter;
		for(char character : line)
		{
			if (character == ',')
			{
				arguments.push_back(currentParameter);
				currentParameter.clear();
			}
			else currentParameter.push_back(character);
		}
		config.push_back(arguments);
	}

	// DynamicObject, Shape, Color, position, velocity, Acceleration, Scale, Radius, Mass, Elasticity
	// KinematicsObject, Shape, Color, Position, Scale
	// CollisionPlane, Position, Normal
	for (std::vector<std::string> args : config)
	{
		if (args[0] == "DynamicObject")
		{
			Dynamic_objs.push_back(new DynamicObject);
			Dynamic_objs.back()->setType(stotype(args[1]));
			Dynamic_objs.back()->setColor(stof(args[2]),stof(args[3]),stof(args[4]));
			Dynamic_objs.back()->setPosition(stof(args[5]),stof(args[6]),stof(args[7]));
			Dynamic_objs.back()->setVelocity(glm::vec3(stof(args[8]),stof(args[9]),stof(args[10])));
			Dynamic_objs.back()->setAcceleration(glm::vec3(stof(args[11]),stof(args[12]),stof(args[13])));
			Dynamic_objs.back()->setScale(stof(args[14]),stof(args[15]),stof(args[16]));
			Dynamic_objs.back()->setBoundingRadius(stof(args[17]));
			Dynamic_objs.back()->setMass(stof(args[18]));
			Dynamic_objs.back()->setElasticity(stof(args[19]));
		}
		else if ( args[0] == "KinematicsObject")
		{
			Kinematic_objs.push_back(new KinematicsObject);
			Kinematic_objs.back()->setType(stotype(args[1]));
			Kinematic_objs.back()->setColor(stof(args[2]),stof(args[3]),stof(args[4]));
			Kinematic_objs.back()->setPosition(stof(args[5]),stof(args[6]),stof(args[7]));
			Kinematic_objs.back()->setScale(stof(args[14]),stof(args[15]),stof(args[16]));
		}
		else if (args[0] == "CollisionPlane")
		{
			Plane_objs.push_back(new CollisionPlane);
			Plane_objs.back()->setPosition(stof(args[1]),stof(args[2]),stof(args[3]));
			Plane_objs.back()->setNormal(glm::vec3(stof(args[4]),stof(args[5]),stof(args[6])));
		}
	}




    float rot = 0.0f;
	while (true)
	{
		/*a.update(dt);
		b.update(dt);
		c.update(dt);

		d.setPosition(10, 20.0f, 0.0f);
		d.setRotation(rot, 0.0f, 0.0f);

		rot++;*/

		for (DynamicObject* obj : Dynamic_objs) obj->update(dt);
		wait(10);
	}

	return 0;
}

int stotype(std::string str)
{
		if (str == "sphere") return PHY_SPHERE;
		else if (str == "cube") return PHY_CUBE;
		else
		{
			std::cout << "INVALID TYPE PARAMETER" << std::endl;
			return 0;
		}
}