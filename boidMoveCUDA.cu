#include <cuda.h>
#include <cuda_runtime.h>
#include <stdio.h>
#include "Boid.h"
#include "Flock.h"


__global__
void kernel_boidMove(Boid* boids)
{
	int index = blockIdx.x * blockDim.x + threadIdx.x;
	
	// Update the graphical point of a boid
	boids[index].tri_v0[0] += boids[index].velocity[0];
	boids[index].tri_v1[0] += boids[index].velocity[0];
	boids[index].tri_v2[0] += boids[index].velocity[0];

	boids[index].tri_v0[1] += boids[index].velocity[1];
	boids[index].tri_v1[1] += boids[index].velocity[1];
	boids[index].tri_v2[1] += boids[index].velocity[1];

	// Update the centroid position
	boids[index].position[0] += boids[index].velocity[0];
	boids[index].position[1] += boids[index].velocity[1];
}


void boidMoveCuda(Flock &flock)
{
	int N = flock.flockSize;
	int flockSize = N * sizeof(Boid);
	
	Boid* d_boids;
	cudaMalloc((void**)&d_boids, flockSize);

	// Copy boids from the host to the device
	cudaMemcpy(d_boids, flock.boids, flockSize, cudaMemcpyHostToDevice);

	// Run kernel
	kernel_boidMove<<<1, flock.flockSize>>>(d_boids);
	     
	// Wait for GPU to finish before accessing on host
	cudaDeviceSynchronize();

	// Copy boids from the device to the host
	cudaMemcpy(flock.boids, d_boids, flockSize, cudaMemcpyDeviceToHost);

	cudaFree(d_boids);
}
