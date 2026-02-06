#include <cyclone/plinks.h>

using namespace cyclone;

using namespace std;

real ParticleLink::currentLength() const {
	Vector3 relative = particles[0]->position - particles[1]->position;

	return relative.magnitude();
}

unsigned ParticleCable::addContact(ParticleContact* contact, unsigned limit) const {
	real length = currentLength();
	if (length < maxLength)
		return 0;

	//we return the contact
	contact->particles[0] = particles[0];

	contact->particles[1] = particles[1];

	//calculate the direction of the pull, normal vector

	Vector3 normal = particles[1]->position - particles[0]->position;
	normal.normalize();
	contact->contactNormal = normal;

	// penetration is the amount of length we are over the max length

	contact->penetration = length - maxLength;

	contact->restitution = restitution;

	return 1;
}

