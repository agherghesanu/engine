#include <iostream>
#include <vector>
#include <cyclone/core.h>
#include <cyclone/particle.h>
#include <cyclone/pfgen.h>   // The Registry
#include <cyclone/pforces.h> // The Concrete Generators

using namespace std;

using namespace cyclone;

int main() {
	ParticleForceRegister registry;

	Particle anchor;

	anchor.position = Vector3(0, 15, 0);

	anchor.setmass(0.0); // infinite mass

	Particle human;

	human.position = Vector3(0, 0, 0);

	human.setmass(2.0); // 70 kg

	human.damping = 0.95;

	ParticleGravity gravity(Vector3(0, -10, 0));

	ParticleSpring spring(&anchor, 10.0, 10.0);

	registry.add(&human, &gravity);

	registry.add(&human, &spring);

	const real duration = 0.01; // 20 ms time step


	for (int i = 0; i <= 150; i++) {
		registry.updateForces(duration);
		human.integrate(duration);

		cyclone::Vector3 dist = human.position - anchor.position;

		std::cout << "T=" << (i * duration) << " | "
			<< "human Y: " << human.position.y << " | "
			<< "Dist: " << dist.magnitude()
			<< std::endl;
	}

    
}