#include <iostream>
#include <vector>
#include <iomanip>
#include <cyclone/pcontacts.h>

using namespace cyclone;

int main() {
   
    // We only have 1 collision max, so 2 iterations is plenty safe.
    ParticleContactResolver resolver(2);

	// set up the particle
    Particle ball;
    ball.position = Vector3(0, 10.0, 0); // Start 10m high
    ball.velocity = Vector3(0, 0, 0);
    ball.accelaration = Vector3(0, -10.0, 0); // Gravity
    ball.setmass(1.0f);
    ball.damping = 0.99f; // Small air resistance

  
    const real duration = 0.02f; // 50 FPS

    // We will simulate 4 seconds of bouncing
    for (int frame = 0; frame < 200; frame++) {

        ball.integrate(duration);

        std::vector<ParticleContact> contacts;

        // Check: Did we hit the floor (Y = 0)?
        if (ball.position.y < 0.0f) {
            ParticleContact contact;

            //collison??
            contact.particles[0] = &ball;
            contact.particles[1] = nullptr; // imovable

 
            //ball is pushed up
            contact.contactNormal = Vector3(0, 1, 0);

            
            contact.restitution = 0.7f; // Lose 30% energy per bounce

            
            // how deep are we? if y is -0.5, penetration is 0.5
            contact.penetration = -ball.position.y;

            contacts.push_back(contact);
        }

        // fix colssions
        if (!contacts.empty()) {
            resolver.resolveContacts(
                contacts.data(),
                contacts.size(),
                duration
            );
        }


        if (frame % 5 == 0) {
            std::cout << "Time: " << std::setw(4) << (frame * duration) << "s | "
                << "Pos Y: " << std::setw(7) << ball.position.y << " | "
                << "Vel Y: " << std::setw(7) << ball.velocity.y << " | "
                << (contacts.empty() ? " " : "BOUNCE!") // Label bounces
                << std::endl;
        }
    }

    return 0;
}