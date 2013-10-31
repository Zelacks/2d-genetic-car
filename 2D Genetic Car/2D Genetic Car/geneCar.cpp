#include "geneCar.h"
#include <map>
#include <iostream>
GeneCar::GeneCar(int genomeindex, std::vector<float> cargenome, b2World *world) {
	wheel1 = 0;
	wheel2 = 0;
	wheel3 = 0;
	wheel4 = 0;
	m_genomeIndex = genomeindex;
	m_carGenome = cargenome;
	m_world = world;
	m_maxDist = 0;
	m_health = 1000;
	initCar();
	flagForDelete = false;
}

void GeneCar::update()
{
	if (((int)m_chassis->GetPosition().x) > m_maxDist) {
		m_maxDist = m_chassis->GetPosition().x;
		m_health = 500;
	}
	else {
		m_health--;
	}

	if (m_health <= 0)
	{
		flagForDelete = true;
	}

}


void GeneCar::initCar() {
	std::map<float, float> carVertex;
	std::multimap<int, float> carWheel;
	
	//Put the angle and the magnitude into a map
	//The data structure automatically sorts the map
	for (int i = 0; i < 8; i++)
	{
		float tempAng, tempMag;
		tempAng = m_carGenome[i*2]*360;
		tempMag = (m_carGenome[(i*2)+1]*3)+1;  //min 1, max 4 for a car that can max 8 metres in length
		carVertex.insert(std::pair<float,float>(tempAng, tempMag));
	}

	//Using the vector data, create vertices for the car.
	std::map<float, float>::iterator it=carVertex.begin();
	for (int i = 0; i < carVertex.size(); i++)
	{
		m_vertex.push_back(b2Vec2((cos(it->first*M_PI/180.0)*it->second), (sin(it->first*M_PI/180.0)*it->second)));
		it++;
	}

	//Set up the car body, first
	b2BodyDef body_def;
	body_def.type = b2_dynamicBody;
	body_def.position.Set(10.0, 10.0);
	m_chassis = m_world->CreateBody(&body_def);

	//Create a fixture, containing the required shape, add it to the main chassis body
	for (int i = 0; i < m_vertex.size()-1; i++) {
		b2Vec2 bodyPart[3];
		bodyPart[0] = b2Vec2(0,0);
		bodyPart[1] = m_vertex[i];
		bodyPart[2] = m_vertex[i+1];
		b2PolygonShape shape;
		shape.Set(bodyPart, 3);
		b2FixtureDef fix_def;
		fix_def.shape = &shape;
		fix_def.density = 80;
		fix_def.friction = 10;
		fix_def.filter.groupIndex = -1;
		m_chassis->CreateFixture(&fix_def);
	}

	//Create an extra fixture, combining the last vertex with the first
	b2Vec2 bodyPart[3];
	bodyPart[0] = b2Vec2(0,0);
	bodyPart[1] = m_vertex[0];
	bodyPart[2] = m_vertex[m_vertex.size()-1];
	b2PolygonShape shape;
	shape.Set(bodyPart, 3);
	b2FixtureDef fix_def;
	fix_def.shape = &shape;
	fix_def.density = 80;
	fix_def.friction = 10;
	fix_def.filter.groupIndex = -1;
	m_chassis->CreateFixture(&fix_def);


	//Gather the data for the wheel
	for (int i = 8; i < 12; i++)
	{
		float tempRad;
		int tempVert;
		tempVert = determineWheelVert(m_carGenome[i*2], m_vertex.size());
		tempRad = (m_carGenome[(i*2)+1]*1.7)+0.3; // min 30cm radius, max 2m radius
		carWheel.insert(std::pair<int, float>(tempVert,tempRad));
	}
	
	std::multimap<int, float>::iterator itr=carWheel.begin();

	//first wheel
	if (itr->first != -1) {
		wheel1 = createWheel(10+m_vertex[itr->first].x, 10+m_vertex[itr->first].y, itr->second );
	}
	itr++;
	//second wheel
	if (itr->first != -1) {
		wheel2 = createWheel(10+m_vertex[itr->first].x, 10+m_vertex[itr->first].y, itr->second );
	}
	itr++;
	//third wheel
	if (itr->first != -1) {
		wheel3 = createWheel(10+m_vertex[itr->first].x, 10+m_vertex[itr->first].y, itr->second );
	}
	itr++;
	//fourth wheel
	if (itr->first != -1) {
		wheel4 = createWheel(10+m_vertex[itr->first].x, 10+m_vertex[itr->first].y, itr->second );
	}


	//create joints for each wheel, the variable poitns to a wheel
	itr=carWheel.begin();
	if (wheel1 != 0)
	{
		b2RevoluteJointDef jointDef;
		jointDef.bodyA=m_chassis;
		jointDef.bodyB=wheel1;
		jointDef.localAnchorA.Set(m_vertex[itr->first].x, m_vertex[itr->first].y);
		jointDef.localAnchorB.Set(0, 0);
		jointDef.enableMotor=true;
		jointDef.maxMotorTorque=800.0;
		jointDef.motorSpeed=-350;
		m_world->CreateJoint(&jointDef);
	}
	itr++;

	if (wheel2 != 0)
	{
		b2RevoluteJointDef jointDef;
		jointDef.bodyA=m_chassis;
		jointDef.bodyB=wheel2;
		jointDef.localAnchorA.Set(m_vertex[itr->first].x, m_vertex[itr->first].y);
		jointDef.localAnchorB.Set(0, 0);
		jointDef.enableMotor=true;
		jointDef.maxMotorTorque=800.0;
		jointDef.motorSpeed=-35;
		m_world->CreateJoint(&jointDef);
	}
	itr++;

	if (wheel3 != 0)
	{
		b2RevoluteJointDef jointDef;
		jointDef.bodyA=m_chassis;
		jointDef.bodyB=wheel3;
		jointDef.localAnchorA.Set(m_vertex[itr->first].x, m_vertex[itr->first].y);
		jointDef.localAnchorB.Set(0, 0);
		jointDef.enableMotor=true;
		jointDef.maxMotorTorque=800.0;
		jointDef.motorSpeed=-35;
		m_world->CreateJoint(&jointDef);
	}
	itr++;

	if (wheel4 != 0)
	{
		b2RevoluteJointDef jointDef;
		jointDef.bodyA=m_chassis;
		jointDef.bodyB=wheel4;
		jointDef.localAnchorA.Set(m_vertex[itr->first].x, m_vertex[itr->first].y);
		jointDef.localAnchorB.Set(0, 0);
		jointDef.enableMotor=true;
		jointDef.maxMotorTorque=800.0;
		jointDef.motorSpeed=-35;
		m_world->CreateJoint(&jointDef);
	}
}

int GeneCar::determineWheelVert(float code, int numofvert)
{
	if (code < 0.2)
	{
		return(-1);
	}
	else
	{
		int myTemp = (int)((code)*numofvert);
		if (myTemp == numofvert)
		{
			myTemp = numofvert-1;
		}
		return(myTemp);
	}

}

b2Body* GeneCar::createWheel(float x, float y, float radius) {
	b2BodyDef bodydef;
	bodydef.position.Set(x,y);
	bodydef.type= b2_dynamicBody;
	b2Body* body = m_world->CreateBody(&bodydef);

	b2CircleShape shape;
	shape.m_radius = radius;
	shape.m_p.Set(0, 0);

	b2FixtureDef fixturedef;
	fixturedef.shape=&shape;
	fixturedef.density=4.0;
	fixturedef.friction=10000;
	fixturedef.filter.groupIndex = -1;

	body->CreateFixture(&fixturedef);
	return(body);
}


GeneCar::~GeneCar() {
	if (m_chassis != 0)
	{
		m_world->DestroyBody(m_chassis);
		m_chassis = 0;
	}

	if (wheel1 != 0)
	{
		m_world->DestroyBody(wheel1);
		wheel1 = 0;
	}

	if (wheel2 != 0)
	{
		m_world->DestroyBody(wheel2);
		wheel2 = 0;
	}

	if (wheel3 != 0)
	{
		m_world->DestroyBody(wheel3);
		wheel3 = 0;
	}

	if (wheel4 != 0)
	{
		m_world->DestroyBody(wheel4);
		wheel4 = 0;
	}
}