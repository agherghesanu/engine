#include <cyclone/pworld.h>

using namespace cyclone;

using namespace std;

ParticleWorld::ParticleWorld(unsigned maxContacts, unsigned iterations) : resolver(iterations), maxContacts(maxContacts) {
	contacts = new ParticleContact[maxContacts];
	calculateIterations = (iterations == 0);
}

ParticleWorld::~ParticleWorld() {
	delete[] contacts;
}

void ParticleWorld::startFrame() {
		for (auto particle : particles) {
			particle->clearAccumulator();
	}
}

unsigned ParticleWorld::generateContacts() {
	unsigned limit = maxContacts;
	ParticleContact *nextContact = contacts;
	

	for (auto generator : contactGenerators) {
		if (limit <= 0) {
			break;
		}
		unsigned used = generator->addContact(nextContact, limit);
		limit -= used;
		nextContact += used;

		
	}

	return maxContacts - limit;
}

void ParticleWorld::integrate(real duration) {
	for (auto particle : particles) {
		particle->integrate(duration);
	}
}

void ParticleWorld::runPhysics(real duration) {
	// we apply the force generators;
	forceRegistry.updateForces(duration);

	// then we integrate the objects
	integrate(duration);

	// generate contacts
	unsigned usedContacts = generateContacts();

	if (usedContacts > 0) {
		if (calculateIterations) {
			resolver.setIterations(usedContacts * 2);
		}
		resolver.resolveContacts(contacts, usedContacts, duration);
	}
}

ParticleWorld::Particles& ParticleWorld::getParticles() {
	return particles;
}

ParticleWorld::ContactGenerators& ParticleWorld::getContactGenerators() {
	return contactGenerators;
}

ParticleForceRegister& ParticleWorld::getForceRegistry() {
	return forceRegistry;
}