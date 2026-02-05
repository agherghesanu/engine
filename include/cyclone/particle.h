#ifndef CYCLONE_PARTICLE_H
#define CYCLONE_PARTICLE_H

#include "core.h"

#include <cfloat>

namespace cyclone {
	class Particle {
	public:
		Vector3 position;

		Vector3 velocity;

		Vector3 accelaration;

		/*
		* force accumulator for the particle, this is used to accumulate
		*forces applied to the particle over a time step
		*/
		Vector3 forceAccum;

		/*
		* Holds the amount of damping applied to linear motion
		* required to remove energy added through numerical instability
		* 0 -> stopped completely, 1 -> no fricition, 0.99 -> a little bit of friction
		*/
		real damping;

		/*
		* Holds the inverses of the mass of the particlle
		* useful in calculations when having to use division repeadtedly
		* storing this value allows to use multiplication when needed
		* 0 - infinite mass, -
		*/
		real inverseMass;

		//default constructor
		Particle() : position(), velocity(), accelaration(), forceAccum(), damping(0.995), inverseMass(1.0) {}

		//constructor with parameters
		// use setter to set the mass to ensure inverse mass is calculated correctly
		Particle(const Vector3& position, const Vector3& velocity, const Vector3& acceelaration, const 
			Vector3& forceAccum, real damping, real inverseMass, real )
			: position(position), velocity(velocity), accelaration(acceelaration), damping(damping), forceAccum(forceAccum) {
			setmass(inverseMass);
		}

		/*
		* adds forces to the particle, these forces will be applied during the next integration step
		*/
		void addForce(const Vector3& force);

		/*
		* clears the forces applied to the particle, this should be called after each integration step
		*/
		void clearAccumulator();

		//integrate the particle forward in time by the given duration
		void integrate(real duration);

		//setter for mass that also calculates the inverse mass
		void setmass(real mass) {
			if (mass > 0.0) {
				inverseMass = 1.0 / mass;
			}
			else {
				inverseMass = 0.0;
			}
		}

		[[nodiscard]] real getmass() const {
			return inverseMass == 0.0 ? DBL_MAX : 1.0 / inverseMass;
		}
	};
}

#endif // !CYCLONE_PARTICLE_H
