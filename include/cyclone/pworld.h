#ifndef CYCLONE_PWORLD_H
#define CYCLONE_PWORLD_H

#include <cyclone/plinks.h>
#include <cyclone/pfgen.h>
#include <vector>

using namespace std;

namespace cyclone {
	/*
	* we keep track of a set of particles
	* and we provide a way to update all of them
	*/
	class ParticleWorld {
	public:
		using Particles = vector<Particle*>;

		using ContactGenerators = vector<ParticleContactGenerator*>;

		/*
		* creates the new world simulation
		* max contactsL: maximum number of contacts that can be handled / frame
		* iterations: number of iterations to give to the contact resolver
		*/
		ParticleWorld(unsigned maxContacts, unsigned iterations = 0);

		/*
		* destrcutor for the world
		*/
		~ParticleWorld();

		/*
		* calls each of the registered contact generators to create contacts
		* returns the number of contacts that have been created
		*/
		unsigned generateContacts();

		/*
		* integrates all particles in the world forward in time by the given duration
		*/
		void integrate(real duration);

		/*
		* processes all the physics for the world
		*/
		void runPhysics(real duration);

		/*
		* initializes the world for a frame. clears forces accumulators
		*/
		void startFrame();

		/*
		* returns the list of particles in the world
		*/
		Particles& getParticles();

		/*
		* returns the list of contact generators in the world
		*/
		ContactGenerators& getContactGenerators();

		/*
		* returns the force registry
		*/

		ParticleForceRegister& getForceRegistry();


	protected:
		Particles particles;

		/*
		* true if the world needs to pass the number of iteration to 
		* give to the contact resolver at each frame
		*/
		bool calculateIterations;

		ParticleForceRegister forceRegistry;

		ParticleContactResolver resolver;

		/*
		* holds the list of contact generators
		* like rosds, cables...
		*/
		ContactGenerators contactGenerators;

		/*
		* holds the list of contacts needed
		*/
		ParticleContact* contacts;

		/*
		* size of the array of particle contacts
		*/
		unsigned maxContacts;
	};
}


#endif // !CYCLONE_PWORLD_H
