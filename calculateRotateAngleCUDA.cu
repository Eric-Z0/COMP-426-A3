#include <cuda.h>
#include <cuda_runtime.h>
#include <stdio.h>
#include <math.h>
#include "Boid.h"
#include "Flock.h"


__global__
void kernel_calculateRoatateAngle(Boid* boids)
{
	int index = blockIdx.x * blockDim.x + threadIdx.x;
	
	float rotateAngle = 0.0f;
	double pi = atan(1.0) * 4;

	float vec_assit[2] = { 0.0f, 0.0f };
	float new_v2[2] = { 0.0f, 0.0f };

	vec_assit[0] = (boids[index].tri_v0[0] + boids[index].tri_v1[0]) / 2;
	vec_assit[1] = (boids[index].tri_v0[1] + boids[index].tri_v1[1]) / 2;

	new_v2[0] = boids[index].tri_v2[0] - vec_assit[0];
	new_v2[1] = boids[index].tri_v2[1] - vec_assit[1];

	float product_vec = new_v2[0] * boids[index].velocity[1] - new_v2[1] * boids[index].velocity[0];
	float factor = 1;

	if (product_vec < 0)
		factor = -1;

	float dot_product = (new_v2[0] * boids[index].velocity[0]) + (new_v2[1] * boids[index].velocity[1]);

	float magnitude1 = powf((powf(new_v2[0], 2) + powf(new_v2[1], 2)), 0.5);
	float magnitude2 = powf((powf(boids[index].velocity[0], 2) + powf(boids[index].velocity[1], 2)), 0.5);

	float angle_rad = factor * acos(dot_product / (magnitude1 * magnitude2));
	rotateAngle = angle_rad / pi * 180;

	boids[index].rotate_angle = rotateAngle;
}

void calculateRoatateAngleCuda(Flock &flock)
{
	int N = flock.flockSize;
	int flockSize = N * sizeof(Boid);
	
	Boid* d_boids;
	cudaMalloc((void**)&d_boids, flockSize);

	// Copy boids from the host to the device
	cudaMemcpy(d_boids, flock.boids, flockSize, cudaMemcpyHostToDevice);

	// Run kernel
	kernel_calculateRoatateAngle<<<1, flock.flockSize>>>(d_boids);

	// Wait for GPU to finish before accessing on host
	cudaDeviceSynchronize();

	// Copy boids from the device to the host
	cudaMemcpy(flock.boids, d_boids, flockSize, cudaMemcpyDeviceToHost);

	cudaFree(d_boids);
}
