#ifndef CYCLONE_PCONTACTS_H
#define CYCLONE_PCONTACTS_H

#include "particle.h"

namespace cyclone {
	
	/*
	* a contact represents two objects coming into contact
	* (colissions, cable rods)
	* resolving a contact involves removing their interpenetration and applying sufficient
	* impulse to keep them apart
	*/
	class ParticleContact {
		/*
		* the resolver needs acces to the contact for resolution
		*/
		friend class ParticleContactResolver;
	public:
		/*
		* holds the particles that are involved in contact
		* second can be null when contact is with the immovable world
		*/

		Particle* particles[2];

		/*
		* restituiton is the bouncines of the contact
		* 1 -> perfectly elastic collision, no energy loss, 0 -> inelastic collision full energy loss
		*/

		real restitution;

		/*
		* direction of the contact in world space
		*/
		Vector3 contactNormal;

		/*
		* holds the depth of penetration 
		*/
		real penetration;

	protected:
		/*
		* resolves this contact for both velocity and interpenetration(objects is calculated inside the 
		*other object)
		*/
		void resolve(real duration);

		/*
		* calculated the separating velocity at this contact
		*/
		real calculateSeparatingVelocity() const;

	private:
		/*
		* handles the impulse calculation for this contact 
		*/
		void resolveVelocity(real duration);

		/*
		* handles interpretation resolution for this contact
		*/
		void resolveInterpenetration(real duration) const;
	};


	/*
	* contact resolution. one instance can be used for the entire simulation
	*/
	class ParticleContactResolver {
	protected:
		/*
		* number of iterations allowed
		*/
		unsigned iterations;
		/*
		* max number of iterations used so we can reset for each frame
		*/
		unsigned iterationsUsed;

	public:
		ParticleContactResolver(unsigned iterations);

		/*
		* setter for iterations 
		*/
		void setIterations(unsigned iterations);

		/*
		* resolves a set of particle contacts for both penetration and velocity
		*/
		void resolveContacts(ParticleContact* contactArray, 
			unsigned numContacts,
			real duration);



	
	};
}

#endif // !CYCLONE_PCONTACTS_H
