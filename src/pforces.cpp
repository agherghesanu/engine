#include <cyclone/pforces.h>

using namespace cyclone;

void ParticleGravity::updateForce(Particle* particle, real duration) {
	if (particle->inverseMass <= 0) {
		return;// inverse mass = 0 -> infinite mass
	}

	// F = m * g  => F = g / inverseMass

	particle->addForce(gravity * particle->getmass());
}

void ParticleDrag::updateForce(Particle* particle, real duration) {
	if (particle->inverseMass <= 0)
		return;

	// calculate the drag force
	Vector3 force = particle->velocity;

	real dragCoeff = force.magnitude();
	real totalDrag = k1 * dragCoeff + k2 * dragCoeff * dragCoeff;

	/*
	* calculate the final force
	* direction is opposite to velocity
	* we normalize the force vector and multiply by -totalDrag
	*/

	force.normalize();

	particle->addForce(force * -totalDrag);
}