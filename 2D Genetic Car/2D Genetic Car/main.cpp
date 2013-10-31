#include <SDL.h>
#include <SDL_opengl.h>
#include <gl\GLU.h>
#include <iostream>
#include <Box2D\Box2D.h>
#include <math.h>
#include <cstdlib>
#include <vector>
#include <random>
#include <string>
#include "genAlg.h"
#include "geneCar.h"
#include <fstream>
#include <iostream>

//Game State Variable
bool simulate = true;

//Camera control
int camx;
int camy;

int generation = 0;
int carmax = 0;
int carmin = MAXINT;
float carsum = 0;
int carcount = 0;

//Box2D + OpenGL/SDL ratios
#define M_PI   3.14159265358979323846264338327950288
const float M2P=20;
const float P2M=1/M2P;
const int WIDTH = 640;
const int HEIGHT = 480;

//std::random_device seeder;
std::uniform_int_distribution<int> dist(0, 3);
std::string worldseed;

//SDL screen
SDL_Surface *sdlWindow;

//Box2D world
b2World *world;

//GameObject lists
std::vector<b2Body*> floorList;
int maxFloorTiles = 800;
std::vector<GeneCar*> carList;

//Genetic Algorithm Class
GenAlg *gAManager;

std::ofstream myfile;


//Foward Function Declaration
void updateObjects();
void CalculateFrameRate();
void physicsStep();
void render();
void SDLEvent(SDL_Event *E);
void mainGameLoop();
void openglSetup();
void init();
void twoDBoxSetup();
b2Body* createRect(int x, int y, int w, int h, bool dynamic, bool test);
void drawSquare(b2Vec2* points, b2Vec2 center, float angle);
b2Body* createCircle(int x, int y, int radius, bool dynamic, bool test);
void drawCircle(b2Vec2 center, float r, float angle);
void drawTriangle(b2Vec2* points, b2Vec2 center, float angle);
void createFloor();


int main(int argc, char *argv[])  
{
	init();
	mainGameLoop();
	return(0);
}

void init() 
{

	myfile.open("GeneCar.txt");

	  if (myfile.is_open())
  {
    std::cout << "File Loaded successfully.\n";
  }

	SDL_Init( SDL_INIT_VIDEO );
	SDL_GL_SetAttribute(SDL_GL_RED_SIZE,            8);
    SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE,          8);
    SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE,           8);
    SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE,          8);

    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE,          16);
    SDL_GL_SetAttribute(SDL_GL_BUFFER_SIZE,         32);

    SDL_GL_SetAttribute(SDL_GL_ACCUM_RED_SIZE,      8);
    SDL_GL_SetAttribute(SDL_GL_ACCUM_GREEN_SIZE,    8);
    SDL_GL_SetAttribute(SDL_GL_ACCUM_BLUE_SIZE,     8);
    SDL_GL_SetAttribute(SDL_GL_ACCUM_ALPHA_SIZE,    8);

    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS,  1);
    SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES,  2);

	SDL_EnableKeyRepeat(SDL_DEFAULT_REPEAT_DELAY, SDL_DEFAULT_REPEAT_INTERVAL);

	sdlWindow = SDL_SetVideoMode(WIDTH, HEIGHT, 32, SDL_OPENGL);

	

	std::cout << "Enter world Seed:" << std::endl;
	std::getline(std::cin,worldseed);
	std::cout << "Enter population size:" << std::endl;
	int popsize;
	std::cin >> popsize;
	gAManager = new GenAlg(popsize, 24);
	gAManager->setSurvivalRate(0.3);
	gAManager->setMutationRate(0.1);
	gAManager->setCrossoverRate(0.1);

	std::uniform_int_distribution<int> mydist(0, 100000);
	std::seed_seq seed2 (worldseed.begin(),worldseed.end());
	std::mt19937 engine(seed2);
	srand(2);

	openglSetup();
	twoDBoxSetup();
}

void openglSetup()
{
	glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
	glOrtho(0, WIDTH, 0, HEIGHT, -1, 1);
	glMatrixMode(GL_MODELVIEW);
	camx = 0;
	camy = 0;
} 

void twoDBoxSetup()
{
	b2Vec2 gravity(0.0f,-10.0f);
	world = new b2World(gravity);
	createFloor();
	//createRect(320, 30, WIDTH, 30, false, false);
}

void mainGameLoop()
{
	while(1) {
		
		//Basic counting variables
		carmax = 0;
		carmin = MAXINT;
		carsum = 0;
		carcount = 0;

		//Generate new cars for each Genome that was created
		myfile << "=======================" << std::endl;
		myfile << "Car Generation: "<< generation << std::endl;
		for (int i = 0; i < gAManager->getPopulationSize(); i++)
		{
			carList.push_back(new GeneCar(i, gAManager->getGenome(i), world));
		}

		//Enter main game loop
		while (simulate) {
		//Handle any SDL events
			static SDL_Event Event;
			while (SDL_PollEvent(&Event))
			{
				SDLEvent(&Event);
			}
		//Update the physics
			physicsStep();
		//Update the status of objects
			updateObjects();
		//Render the scene
			render();
		}
		float avg = carsum / (float)carcount;
		myfile << "Car Max: "<< carmax << std::endl;
		myfile << "Car Min: "<< carmin << std::endl;
		myfile << "Car Avg: "<< avg << std::endl;
		myfile << "=======================" << std::endl;

		gAManager->newGeneration();
		generation++;
		simulate = true;
	}
}

void SDLEvent(SDL_Event *E)
{
	switch(E->type)
	{
	case SDL_QUIT:
		exit(0);
		break;
	case SDL_KEYDOWN:
		break;
	case SDL_MOUSEBUTTONDOWN:
		break;
	default:
		break;
	}
}

void physicsStep()
{
	int32 velocityIterations = 6;
	int32 positionIterations = 2;
	float32 timeStep = 1.0f / 60.0f;
	world->Step(timeStep, velocityIterations, positionIterations);

}

void updateObjects()
{
	if (carList.size() == 0)
	{
		simulate = false;
	}
	else
	{
		for (int i = 0; i < carList.size(); i++)
		{
			carList[i]->update();

			if(carList[i]->flagForDelete) {
				float carFitness = carList[i]->getMaxDist();
				gAManager->setFitness(carList[i]->m_genomeIndex, carFitness);
				if (carFitness > carmax)
				{
					carmax = carFitness;
				}
				if (carFitness < carmin)
				{
					carmin = carFitness;
				}

				carsum = carsum + carFitness;
				carcount++;
				myfile << "Car "<< carList[i]->m_genomeIndex << ": " << carFitness << std::endl;

				delete carList[i];
				carList.erase(carList.begin()+i);
				i--;
			}
		}
	}

}

void render()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();



	float tempx = 0;
	float tempy = 0;
	for(int i = 0; i < carList.size(); i++) {
		if (carList[i]->getPosx() > tempx)
		{
			tempx = carList[i]->getPosx();
			tempy = carList[i]->getPosy();
		}
	}
	glTranslatef((-tempx*M2P)+320.0, (-tempy*M2P)+240.0, 0);

	b2Body* currentBody=world->GetBodyList();

	//For each body, render the shape.
	//Point currentBody to the next body
	//If there are no more bodies, currentBody will point to null



	while(currentBody)
	{
		if (currentBody->GetFixtureList()->GetShape()->GetType()==2)
		{
			if (((b2PolygonShape*) currentBody->GetFixtureList()->GetShape())->GetVertexCount() == 4)
			{
				b2Vec2 points[4];
				for (int i = 0; i < 4; i++)
				{
					points[i] = ((b2PolygonShape*) currentBody->GetFixtureList()->GetShape()) ->GetVertex(i);
				}

				drawSquare(points, currentBody->GetWorldCenter(), currentBody->GetAngle());
			}

			//Worst idea ever, but this currently draws eeach car, as they are the only triangle available
			else if (((b2PolygonShape*) currentBody->GetFixtureList()->GetShape())->GetVertexCount() == 3)
			{
				b2Fixture *fl = currentBody->GetFixtureList();
				while (fl != 0)
				{
					b2Vec2 points[3];
					for (int i = 0; i < 3; i++)
					{
						points[i] = ((b2PolygonShape*) fl->GetShape())->GetVertex(i);
					}
					drawTriangle(points, currentBody->GetPosition(), currentBody->GetAngle());

					fl= fl->GetNext();
				}
			}

		}

		else if (currentBody->GetFixtureList()->GetShape()->GetType()==0)
		{
			b2CircleShape* c = ((b2CircleShape*) currentBody->GetFixtureList()->GetShape());
			drawCircle(currentBody->GetWorldCenter(), c->m_radius, currentBody->GetAngle());
		}


		currentBody=currentBody->GetNext();
	}

	CalculateFrameRate();
	//glFlush();
	SDL_GL_SwapBuffers();
}

void drawTriangle(b2Vec2* points, b2Vec2 center, float angle)
{
	glPushMatrix();
		glTranslatef(center.x*M2P, center.y*M2P, 0);
		glRotatef(angle*180.0/M_PI,0, 0, 1);

		glBegin(GL_TRIANGLES);
			for(int i=0;i<3;i++)
			{
				glColor3f(1.0f, 0.0f, 0.0f);
				glVertex2f(points[i].x*M2P, points[i].y*M2P);
				
			}

		glEnd();


	glPopMatrix();
}

void drawSquare(b2Vec2* points, b2Vec2 center, float angle)
{
	glPushMatrix();
		glTranslatef(center.x*M2P, center.y*M2P, 0);
		glRotatef(angle*180.0/M_PI,0, 0, 1);

		glBegin(GL_QUADS);
			for(int i=0;i<4;i++)
			{
				glColor3f(1.0f, 1.0f, 1.0f);
				glVertex2f(points[i].x*M2P, points[i].y*M2P);
				
			}

		glEnd();


	glPopMatrix();
}

void drawCircle(b2Vec2 center, float r, float angle) {

	
	glPushMatrix();
		glTranslatef(center.x*M2P, center.y*M2P, 0);
		glRotatef(angle*180.0/M_PI, 0, 0, 1);

		glBegin(GL_TRIANGLE_FAN);
			glColor3f(1.0, 1.0, 1.0);
			glVertex2f(0, 0);
			for (float i = 0.0; i <=360; i+= 360.0/20.0) 
			{
				glVertex2f((cos(i*M_PI/180.0)*r)*M2P,(sin(i*M_PI/180.0)*r)*M2P);
			}
		glEnd();

		glBegin(GL_LINES);
			glColor3f(0.0, 0.0, 0.0);
			
			glVertex2f(0, 0);
			glVertex2f(r*M2P, 0);
		glEnd();
		glLineWidth(2); 
	glPopMatrix();

}

void CalculateFrameRate()
{
	static float framesPerSecond    = 0.0f;       // This will store our fps
	static float lastTime			= 0.0f;       // This will hold the time from the last frame
	float currentTime = GetTickCount() * 0.001f;    
	++framesPerSecond;
	if( currentTime - lastTime > 1.0f )
	{
		lastTime = currentTime;
		std::cout << framesPerSecond << std::endl;
		framesPerSecond = 0;
	}
}

b2Body* createRect(int x, int y, int w, int h, bool dynamic, bool test) {
	b2BodyDef bodydef;
	bodydef.position.Set(x*P2M,y*P2M);
	if (dynamic)
		bodydef.type= b2_dynamicBody;
	b2Body* body = world->CreateBody(&bodydef);

	b2PolygonShape shape;
	shape.SetAsBox(P2M*w/2, P2M*h/2);

	b2FixtureDef fixturedef;
	fixturedef.shape=&shape;
	fixturedef.density=1.0;
	//fixturedef.friction=0.0;
	if (test)
		fixturedef.filter.groupIndex = -1;

	body->CreateFixture(&fixturedef);
	return(body);
}

b2Body* createCircle(int x, int y, int radius, bool dynamic, bool test) {
	b2BodyDef bodydef;
	bodydef.position.Set(x*P2M,y*P2M);
	if (dynamic)	
		bodydef.type= b2_dynamicBody;
	b2Body* body = world->CreateBody(&bodydef);

	b2CircleShape shape;
	shape.m_radius = radius*P2M;
	shape.m_p.Set(0, 0);

	b2FixtureDef fixturedef;
	fixturedef.shape=&shape;
	fixturedef.density=1.0;
	fixturedef.friction=0.7;

	if (test)
		fixturedef.filter.groupIndex = -1;

	body->CreateFixture(&fixturedef);
	return(body);
}

void rotateFloorTile(b2Vec2 *coords, float angle) {
	b2Vec2 center = b2Vec2(0,0);
	b2Vec2 temp[4];
	for (int i = 0; i < 4; i++)
	{
		temp[i].x = cos(angle)*(coords[i].x - center.x) - sin(angle)*(coords[i].y - center.y) + center.x;
		temp[i].y = sin(angle)*(coords[i].x - center.x) + cos(angle)*(coords[i].y - center.y) + center.y;
	}

	for (int i = 0; i < 4; i++)
	{
		coords[i] = temp[i];
	}
}

b2Body *createFloorTile(b2Vec2 position, float angle) {
	b2BodyDef body_def;

	body_def.position.Set(position.x, position.y);
	b2Body *body = world->CreateBody(&body_def);
	
	b2PolygonShape shape;
	b2Vec2 coords[4];
	coords[0] = b2Vec2(0,0);
	coords[1] = b2Vec2(0,-0.5);
	coords[2] = b2Vec2(5,-0.5);
	coords[3] = b2Vec2(5,0);
	rotateFloorTile(coords, angle);
	shape.Set(coords, 4);

	b2FixtureDef fix_def;
	fix_def.friction = 0.2;
	fix_def.shape = &shape;
	body->CreateFixture(&fix_def);

	return body;
}



void createFloor() {
	std::seed_seq seed2 (worldseed.begin(),worldseed.end());
	std::mt19937 engine(seed2);
  b2Body *last_tile = 0;
  b2Vec2 tile_position = b2Vec2(0,3);
  //Math.seedrandom(floorseed);
  for(int k = 0; k < maxFloorTiles; k++) {
	float angle = (dist(engine) - 1.5) * 1.5*k/maxFloorTiles;
    last_tile = createFloorTile(tile_position, angle);
    floorList.push_back(last_tile);
    b2Fixture *last_fixture = last_tile->GetFixtureList();
	b2Vec2 last_world_coords;
	if (angle >= 0) {
		last_world_coords = last_tile->GetWorldPoint(((b2PolygonShape*)last_fixture->GetShape())->m_vertices[1]);
	}
	else {
		last_world_coords = last_tile->GetWorldPoint(((b2PolygonShape*)last_fixture->GetShape())->m_vertices[0]);
	}
    tile_position = last_world_coords;
  }
}


