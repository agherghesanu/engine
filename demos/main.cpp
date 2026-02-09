#include <iostream>
#include <vector>

#include <iomanip>

#include <cyclone/core.h>
#include <cyclone/particle.h>
#include <cyclone/pcontacts.h>	
#include <cyclone/pfgen.h>
#include <cyclone/pforces.h>
#include <cyclone/plinks.h>
#include <cyclone/pworld.h>

using namespace cyclone;

using namespace std;

void printBridge(const vector<Particle*> particles) {
	Particle* mid = particles[particles.size() / 2];

	cout << "Mid-Deck Height (Y): " << std::fixed << std::setprecision(2)
		<< mid->position.y << " m";
}

int main() {
	ParticleWorld world(1000, 1200);

	vector<Particle*> particles;

	vector<ParticleCable*> links;

	for (int i = 0; i < 12; i++) {
		Particle* p = new Particle();
		p->position = Vector3(-12.0 + i * 2.0, 0, 0);
		p->velocity = Vector3(0, 0, 0);
		p->accelaration = Vector3(0, 0, 0);
		p->setmass(2.0);
		p->damping = 0.95;

		particles.push_back(p);

		world.getParticles().push_back(p);
	}

	Particle anchorLeft, anchorRight;
	anchorLeft.position = Vector3(-12.5, 0, 0);
	anchorLeft.velocity = Vector3(0, 0, 0);
	anchorLeft.accelaration = Vector3(0, 0, 0);
	anchorLeft.setmass(0.0); // imovable

	anchorRight.position = Vector3(11, 0, 0);
	anchorRight.velocity = Vector3(0, 0, 0);
	anchorRight.accelaration = Vector3(0, 0, 0);
	anchorRight.setmass(0.0); // imovable

	ParticleCable* leftLink = new ParticleCable();
	leftLink->particles[0] = &anchorLeft;
	leftLink->particles[1] = particles[0];
	leftLink->maxLength = 2.0;
	leftLink->restitution = 0.0;

	world.getContactGenerators().push_back(leftLink);
	links.push_back(leftLink);

	ParticleCable* rightLink = new ParticleCable();
	rightLink->particles[0] = particles[particles.size() - 1];
	rightLink->particles[1] = &anchorRight;
	rightLink->maxLength = 2.0;
	rightLink->restitution = 0.0;


	for (int i = 0; i < 11; i++) {
		ParticleCable* link = new ParticleCable();
		link->particles[0] = particles[i];
		link->particles[1] = particles[i + 1];
		link->maxLength = 2.2;
		link->restitution = 0.0;

		world.getContactGenerators().push_back(link);
		links.push_back(link);
	}

	world.getContactGenerators().push_back(rightLink);
	links.push_back(rightLink);

	ParticleGravity* gravity = new ParticleGravity(Vector3(0, -9.81, 0));
	for (auto p : particles) {
		world.getForceRegistry().add(p, gravity);
	}

	const real duration = 0.00025;

	for (int i = 0; i < 10000; i++) {
		world.startFrame();
		world.runPhysics(duration);

		if (i % 5 == 0) {
			std::cout << "T=" << std::setw(4) << (i * duration) << "s | ";
			printBridge(particles);
			std::cout << " | " << (i < 10 ? "Falling..." : "Settling...") << std::endl;
		}

	}

}




