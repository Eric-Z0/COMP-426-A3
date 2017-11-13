#pragma once
#include "Boid.h"
#include <vector>
#include <mutex>
#include <thread>

class Flock {

public:

	static int num_of_flocks;
	static int flock_counter;
	int flock_sn = 0;
	bool allBoidsPosUpdated = true;
	bool collionDetected = false;
	static bool flocksTimeToUpdate;
	float flockGoalWeight = 1.0f;
	float dirAjustmentCtr = 2;

	//std::vector<Boid> boids;
	Boid* boids;			// change vector to pointer
	int flockSize = 0;		// the total number of boids a flock has

	float flockTargetPos[2] = { 0.0f, 0.0f };
	float flock_color[3] = { 1.0f, 1.0f, 1.0f };

	float flockCenter[2] = { 0.0f, 0.0f };
	float flockCollisionPrevent[2] = { 0.0f, 0.0f };

	Flock();

	void initializeFlock(int numOfBoids);
	void addBoid(Boid &boid, int index);
	void calFlockCenter();
	bool boidIsAroundFlockCenter(int boid_id);
	void setFlockColor(float color_set[]);

};
