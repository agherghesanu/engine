#ifndef CYCLONE_PFORCES_H
#define CYCLONE_PFORCES_H

#include "pfgen.h"

namespace cyclone {
	/*
	* a force geneerator that applies a graitaitonal force 
	*/

	class ParticleGravity : public ParticleForceGenerator {
	public:
		Vector3 gravity;

		ParticleGravity(const Vector3& gravity) : gravity(gravity) {}

		virtual void updateForce(Particle* particle, real duration);
	};

	/*
	* a force generator that applies drag force / friction
	*/

	class ParticleDrag : public ParticleForceGenerator {
	public:
		real k1; // velocity drag coefficient
		real k2; // velocity squared drag coefficient
		ParticleDrag(real k1, real k2) : k1(k1), k2(k2) {}
		virtual void updateForce(Particle* particle, real duration);
	};
}

#endif // !CYCLONE_PFORCES_H
