#include <algorithm>

#include <cyclone/pfgen.h>

using namespace cyclone;

void ParticleForceRegister::add(Particle* particle, ParticleForceGenerator* fg) {
	ParticleForceRegistration registration;
	registration.particle = particle;
	registration.fg = fg;
	registrations.push_back(registration);
}

void ParticleForceRegister::remove(Particle* particle, ParticleForceGenerator* fg) {
	/*
	* used a lambda function to find the matching particle-force generator pair
	* the remove_if algorithm shifts all the elements that do not match to the front of the vector
	*/

	auto it = std::remove_if(registrations.begin(), registrations.end(),
		[particle, fg](const ParticleForceRegistration& entry) {
			return entry.particle == particle && entry.fg == fg; });

	registrations.erase(it, registrations.end());
}

void ParticleForceRegister::clear() {
	registrations.clear();
}

/*
* we loop through all the registrations and call the 
* updateForce method on each force generator based
*/
void ParticleForceRegister::updateForces(real duration) {
	for (auto& registration : registrations) {
		registration.fg->updateForce(registration.particle, duration);
	}
}
