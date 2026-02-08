#ifndef CYCLONE_PLINKS_H
#define CYCLONE_PLINKS_H

#include "cyclone/pcontacts.h"

namespace cyclone {

	class ParticleContactGenerator {
		public:
		/*
		* fills the contact structure with the contact needed
		* from violatng the constraint
		* returns the number of contacts that it has filled in
		*/
		virtual unsigned addContact(ParticleContact* contact, unsigned limit) const = 0;
	};

	/*
	* links two particle together with a constraint, generating a contact
	* on constraint vioaltion
	*/
	class ParticleLink : public ParticleContactGenerator {
	public:
		// the two particles that are connected by this link
		Particle* particles[2];
		
		// the length of this link 
		real currentLength() const;

		/*
		* fills the contact structure with the contact needed
		* from violatng the constraint
		* returns the number of contacts that it has filled in
		*/
		virtual unsigned addContact(ParticleContact* contact, unsigned limit) const = 0;
	};

	class ParticleCable : public ParticleLink {
	public:
		real maxLength;

		real restitution; // bounciness of the cable 0.0 -> inelastic, 0.5 somewhatbouncy

		virtual unsigned addContact(ParticleContact* contact, unsigned limit) const;
	};
}



#endif // !CYCLONE_PLINKS_H
