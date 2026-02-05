//abstract class for particle force generators

#ifndef CYCLONE_PFGEN_H
#define CYCLONE_PFGEN_H
#include "particle.h"
#include <vector>

namespace cyclone {
	/*a generator can be asked to add a force to one
	* or more particles
	*/

	class ParticleForceGenerator {
	public:
		/*makes the particle force genereator an abstact class
		* =0 forces implemetation in derived classes
		*/
		virtual void updateForce(Particle* particle, real duration) = 0;
	};
	class ParticleForceRegister {
	protected:
		
		/* we are keeping track of the particles and their
		* force generators in a struct for easy management
		*/
		struct ParticleForceRegistration {
			Particle* particle;
			ParticleForceGenerator* fg;
		};

		using Registry = std::vector<ParticleForceRegistration>;

		Registry registrations;

	public:
		/*
		* registers the given particle to be updated by the given force generator
		*/
		void add(Particle* particle, ParticleForceGenerator* fg);

		/*
		* removes the pair from the registry
		* so that the force generator will no longer apply to the particle
		*/
		void remove(Particle* particle, ParticleForceGenerator* fg);

		/*
		* clears all registrations
		*/
		void clear();

		/*
		* calls all the force generators to update their
		* corresponding particles
		*/
		void updateForces(real duration);

	};
}


#endif