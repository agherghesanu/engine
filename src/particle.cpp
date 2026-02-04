#include <cmath>

#include <cyclone/particle.h>

using namespace cyclone;

void Particle::integrate(real duration) {
	if(inverseMass <= 0.0 || duration <=0.0) {
		return; // infinite mass objects do not move
	}

	//update position
	position += velocity * duration;

	//calculate the resulting acceleration for the force
	Vector3 resultingAcc = accelaration;

	//update velocity from the acceleration
	velocity += resultingAcc * duration;

	// we apply friction -> damping = e^(-k*t), k is the damping coefficient, t 
	// t is time; derived from the m*v'+m*k*v = 0 differential equation
	velocity *= std::pow(damping, duration);

}