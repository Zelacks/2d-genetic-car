#pragma once
#include <Box2D\Box2D.h>
#include "genAlg.h"
#define M_PI   3.14159265358979323846264338327950288


//b2Body *m_chassis;

class GeneCar
{
public:
	GeneCar() {}
	GeneCar(int genomeindex, std::vector<float> cargenome, b2World *world);
	~GeneCar();
	void update();
	bool flagForDelete;
	int	 m_genomeIndex;
	float getPosx() {return m_chassis->GetPosition().x;}
	float getPosy() {return m_chassis->GetPosition().y;}
	float getMaxDist() {return (float)m_maxDist;}

private:
	void initCar();
	b2Body* createWheel(float x, float y, float radius);
	int determineWheelVert(float code, int numofvert);

	b2World *m_world;
	b2Body *m_chassis;
	b2Body *wheel1;
	b2Body *wheel2;
	b2Body *wheel3;
	b2Body *wheel4;

	std::vector<GeneCar*> m_carList;
	
	std::vector<float> m_carGenome;
	std::vector<b2Vec2> m_vertex;
	
	int m_maxDist;
	int m_health;

};