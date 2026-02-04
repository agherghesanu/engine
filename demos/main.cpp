#include <iostream>
#include <cyclone/particle.h>

#include <vector>
#include <iomanip>

struct AmmoRound {
	cyclone::Particle particle;
	std::string type;

	AmmoRound(const cyclone::Particle& p, const std::string& t) : particle(p), type(t) {}
};

int main() {
	std::vector<AmmoRound> rounds;

    cyclone::Particle pistol;
    pistol.position = cyclone::Vector3(0, 0, 0);
    pistol.velocity = cyclone::Vector3(35, 0, 0);
    pistol.accelaration = cyclone::Vector3(0, -10.0, 0); // Gravity
    pistol.damping = 0.99;
	rounds.push_back(AmmoRound(pistol, "Pistol"));

    // B. Artillery
    // Upward angle (40 up, 30 forward), Standard Gravity, Little Damping
    cyclone::Particle artillery;
    artillery.position = cyclone::Vector3(0, 0, 0);
    artillery.velocity = cyclone::Vector3(30, 40, 0); // Shooting up/forward
    artillery.accelaration = cyclone::Vector3(0, -20.0, 0); // Heavy gravity feel
    artillery.damping = 0.99;
    rounds.push_back(AmmoRound(artillery, "Artillery"));

    // C. Fireball
    // Slow, 'Floaty' (Gravity -0.6), High Damping (0.9) simulates air resistance
    cyclone::Particle fireball;
    fireball.position = cyclone::Vector3(0, 0, 0);
    fireball.velocity = cyclone::Vector3(10, 0, 0);
    fireball.accelaration = cyclone::Vector3(0, -0.6, 0); // Very low gravity
    fireball.damping = 0.9;
    rounds.push_back(AmmoRound(fireball, "fireball"));
    // D. Laser Beam
    // Super fast, No Gravity, No Damping (goes forever)
    cyclone::Particle laser;
    laser.position = cyclone::Vector3(0, 0, 0);
    laser.velocity = cyclone::Vector3(100, 0, 0);
    laser.accelaration = cyclone::Vector3(0, 0, 0); // No gravity
    laser.damping = 1.0; // No damping
    rounds.push_back(AmmoRound(laser, "laser"));

	cyclone::real duration = 1.0; // time step

	std::cout << std::fixed << std::setprecision(2); 


    for (int i = 0; i < 10; i++) {
        std::cout << "\n[ Time: " << i << "s ]" << std::endl;
        std::cout << "-----------------------------------------------------" << std::endl;
        std::cout << "| Type      | Pos X   | Pos Y   | Vel Y   | Status" << std::endl;

        for (auto& round : rounds) {
			cyclone::Particle& p = round.particle;

            std::string status = "Flying";
            if (p.position.y < 0 && i > 0) status = "HIT GROUND";

            std::cout << "| " << std::setw(9) << round.type
                << " | " << std::setw(7) << p.position.x
                << " | " << std::setw(7) << p.position.y
                << " | " << std::setw(7) << p.velocity.y
                << " | " << status << std::endl;

            // PHYSICS STEP: Move the particle
            // We only integrate if it hasn't hit the ground yet (simple check)
            if (p.position.y >= 0) {
                p.integrate(duration);
            }
        }
    }
}