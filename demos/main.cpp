#include <iostream>
#include <vector>
#include <iomanip>
#include <cmath>
#include <cyclone/core.h>
#include <cyclone/particle.h>
#include <cyclone/pcontacts.h>
#include <cyclone/plinks.h>

using namespace cyclone;

using namespace std;


int main() {
    ParticleContactResolver resolver(1);

    //Setup the Particles
  
    Particle anchor;
    anchor.position = Vector3(0, 10, 0);
    anchor.setmass(0); // Infinite Mass (Immovable)
    anchor.velocity = Vector3(0, 0, 0);
    anchor.accelaration = Vector3(0, 0, 0);

    // Particle B: The Wrecking Ball
    Particle ball;
    ball.position = Vector3(10, 10, 0); // Start 10m to the right (Horizontal)
    ball.setmass(200.0); // Heavy
    ball.velocity = Vector3(0, 0, 0);
    ball.accelaration = Vector3(0, -10.0, 0); // Gravity
    ball.damping = 0.99f;

    //Setup the Link 
    ParticleCable cable;
    cable.particles[0] = &anchor;
    cable.particles[1] = &ball;
    cable.maxLength = 10.0; 
    cable.restitution = 0.0; 

    // We use a small timestep for stability with hard constraints
    const real duration = 0.01;

    vector<ParticleContact> contacts;

    for (int frame = 0; frame < 300; frame++) {

      
        ball.integrate(duration);

 
        contacts.clear(); // Reset the list

        // Create a temporary contact object to fill
        ParticleContact tempContact;

        // Ask the cable: "Are we overextended?"
        // If yes, it returns 1 and fills 'tempContact'
        if (cable.addContact(&tempContact, 1) > 0) {
            contacts.push_back(tempContact);
        }

        if (!contacts.empty()) {
            resolver.resolveContacts(
                contacts.data(),
                contacts.size(),
                duration
            );
        }

   
        if (frame % 10 == 0) {
          
            Vector3 distVec = ball.position - anchor.position;
            real currentDist = distVec.magnitude();
            string status = (contacts.empty()) ? "Slack" : "TAUT!";

            cout << "T=" << setw(4) << (frame * duration) << " | "
                << "Ball Pos: (" << setw(5) << fixed << setprecision(2)
                << ball.position.x << ", " << ball.position.y << ") | "
                << "Len: " << currentDist << " | "
                << status << endl;
        }
    }

    return 0;
}