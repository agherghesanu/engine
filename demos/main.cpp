#include <iostream>
#include <vector>
#include <cyclone/core.h>
#include <cyclone/particle.h>
#include <cyclone/pfgen.h>   // The Registry
#include <cyclone/pforces.h> // The Concrete Generators


int main() {

    // 1. Create the Registry
    cyclone::ParticleForceRegister registry;

    // 2. Create Generators
    // A. Gravity (Standard Earth Gravity)
    cyclone::Vector3 gravityVector(0, -10.0, 0);
    cyclone::ParticleGravity gravityGen(gravityVector);

    // B. Drag (Air Resistance)
    // k1 = 0.1 (velocity drag), k2 = 0.0 (velocity squared drag)
    cyclone::ParticleDrag dragGen(0.1, 0.0);

    // 3. Create a Particle (The "Paratrooper")
    cyclone::Particle paratrooper;
    paratrooper.position = cyclone::Vector3(0, 100, 0); // Start high up
    paratrooper.velocity = cyclone::Vector3(0, 0, 0);
    paratrooper.setmass(2.0f); // 2kg mass
    paratrooper.damping = 0.99;
    // CRITICAL: acceleration is now (0,0,0). Gravity is applied via GENERATOR, not hardcoded.
    paratrooper.accelaration = cyclone::Vector3(0, 0, 0);

    // 4. Register the forces
    // "Apply Gravity to the Paratrooper"
    registry.add(&paratrooper, &gravityGen);
    // "Apply Drag to the Paratrooper"
    registry.add(&paratrooper, &dragGen);

    // 5. Simulation Loop
    cyclone::real duration = 0.1; // 100ms per frame

    for (int i = 0; i <= 20; i++) {
        // A. Update Forces (The Registry Magic)
        // This calls gravityGen.updateForce() and dragGen.updateForce() automatically
        registry.updateForces(duration);

        // B. Integrate (Move the particle)
        paratrooper.integrate(duration);

        // Output Status
        std::cout << "Time: " << (i * duration) << "s | "
            << "Pos Y: " << paratrooper.position.y << " | "
            << "Vel Y: " << paratrooper.velocity.y << " | "
            << "Forces Applied"
            << std::endl;
    }

    return 0;
}