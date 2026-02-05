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

	/*
	* one particle is connected to another by a spring
	* this generator creates the connection
	*/
	class ParticleSpring : public ParticleForceGenerator {
	public:
		Particle* other; // the other end of the spring

		real springConstant; // stiffness of the spring

		real restLength; // length of the spring at rest

		ParticleSpring(Particle* other, real springConstant, real restLength):
			other(other), springConstant(springConstant), restLength(restLength) {
		}

		virtual void updateForce(Particle* other, real duration);
	};
}

#endif // !CYCLONE_PFORCES_H
