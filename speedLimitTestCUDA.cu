#include <cuda.h>
#include <cuda_runtime.h>
#include <stdio.h>
#include "Boid.h"
#include "Flock.h"

__global__
void kernel_speedLimitTest(Boid* boids)
{
	int index = blockIdx.x * blockDim.x + threadIdx.x;
	float boidMaxSpeed = 0.005f;

	float magnitude = powf((powf(boids[index].velocity[0], 2) + powf(boids[index].velocity[1], 2)), 0.5);

	if (magnitude > boidMaxSpeed)
	{
		boids[index].velocity[0] = boids[index].velocity[0] / magnitude * boidMaxSpeed;
		boids[index].velocity[1] = boids[index].velocity[1] / magnitude * boidMaxSpeed;
	}
}

void speedLimitTestCuda(Flock &flock)
{
	int N = flock.flockSize;
	int flockSize = N * sizeof(Boid);
	
	Boid* d_boids;
	cudaMalloc((void**)&d_boids, flockSize);

	// Copy boids from the host to the device
	cudaMemcpy(d_boids, flock.boids, flockSize, cudaMemcpyHostToDevice);

	// Run kernel
	kernel_speedLimitTest<<<1, flock.flockSize>>>(d_boids);

	// Wait for GPU to finish before accessing on host
	cudaDeviceSynchronize();

	// Copy boids from the device to the host
	cudaMemcpy(flock.boids, d_boids, flockSize, cudaMemcpyDeviceToHost);

	cudaFree(d_boids);
}