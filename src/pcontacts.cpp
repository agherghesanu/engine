#include <cyclone/pcontacts.h>

using namespace cyclone;

void ParticleContact::resolve(real duration) {
	resolveVelocity(duration);
	resolveInterpenetration(duration);
}

void ParticleContact::resolveInterpenetration(real duration) const {
	
	//no penetration to resolve
	if (penetration <= 0) return;

	//movement is based on each particles inverse mass
	real totalInverseMass = particles[0]->inverseMass;

	if (particles[1]) totalInverseMass += particles[1]->inverseMass;

	if (totalInverseMass == 0) return; // infinite mass


	//amount of penetration resolution per unit of inverse mass
	Vector3 movePerIMass = contactNormal * (penetration / totalInverseMass);

	//apply the penetration resolution

	particles[0]->position += movePerIMass * particles[0]->inverseMass;

	if (particles[1]) 
		particles[1]->position -= movePerIMass * particles[1]->inverseMass;

	const_cast<ParticleContact*>(this)->penetration = 0; // reset penetration after resolving
}

void ParticleContact::resolveVelocity(real duration)
{
    // Find the velocity in the direction of the contact
    real separatingVelocity = calculateSeparatingVelocity();

    // If it is already separating, do nothing.
    if (separatingVelocity > 0) {
        return;
    }

    // Calculate the new separating velocity
    // newSepVelocity = -restitution * oldSepVelocity
    real newSepVelocity = -separatingVelocity * restitution;

    // -- Check for velocity buildup due to acceleration only --
    // This prevents "jitter" when resting on the ground
    Vector3 accCausedVelocity = particles[0]->accelaration;
    if (particles[1]) accCausedVelocity -= particles[1]->accelaration;

    real accCausedSepVelocity = accCausedVelocity * contactNormal * duration;

    // If we've got a closing velocity due to acceleration buildup, remove it
    // from the new separating velocity
    if (accCausedSepVelocity < 0) {
        newSepVelocity += restitution * accCausedSepVelocity;

        // Make sure we haven't removed more than was there
        if (newSepVelocity < 0) newSepVelocity = 0;
    }

    real deltaVelocity = newSepVelocity - separatingVelocity;

    // We apply the change in velocity to each object in proportion to
    // their inverse mass (heavier objects move less)
    real totalInverseMass = particles[0]->inverseMass;
    if (particles[1]) totalInverseMass += particles[1]->inverseMass;

    // If all particles have infinite mass, impulses have no effect
    if (totalInverseMass <= 0) return;

    // Calculate the impulse to apply
    real impulse = deltaVelocity / totalInverseMass;

    // Find the amount of impulse per unit of inverse mass
    Vector3 impulsePerIMass = contactNormal * impulse;

    // Apply impulses:
    // Particle 1 goes in direction of normal
    particles[0]->velocity = particles[0]->velocity + impulsePerIMass * particles[0]->inverseMass;

    // Particle 2 goes in opposite direction
    if (particles[1]) {
        particles[1]->velocity = particles[1]->velocity + impulsePerIMass * -particles[1]->inverseMass;
    }
}

real ParticleContact::calculateSeparatingVelocity() const
{
    Vector3 relativeVelocity = particles[0]->velocity;
    if (particles[1]) {
        relativeVelocity -= particles[1]->velocity;
    }
    return relativeVelocity * contactNormal; // Dot product
}

ParticleContactResolver::ParticleContactResolver(unsigned iterations)
	: iterations(iterations), iterationsUsed(0) {
}

void ParticleContactResolver::setIterations(unsigned iterations) {
    this->iterations = iterations;
}

void cyclone::ParticleContactResolver::resolveContacts(ParticleContact* contactArray,
    unsigned numContacts,
    real duration) {

    iterationsUsed = 0;
    while (iterationsUsed < iterations) {
        real max = DBL_MAX;

    
        unsigned maxIndex = numContacts;

    
        for (unsigned i = 0; i < numContacts; i++) {
            real selVelocity = contactArray[i].calculateSeparatingVelocity();

         
            if (selVelocity < max && (selVelocity < 0 || contactArray[i].penetration > 0)) {
                max = selVelocity;
                maxIndex = i;
            }
        }


        if (maxIndex == numContacts)
            break;

        contactArray[maxIndex].resolve(duration);

        iterationsUsed++;
    }
}
