#ifndef CYCLONE_CONTACTS_H
#define CYCLONE_CONTACTS_H

#include <body.h>

namespace cyclone {

	/*
	* contact means two bodies that touch each other
	* this resolves interpenetration and applies sufficient
	* impulse too keep them apart
	*/
	class Contact {

		/*
		* contact resolver needs access to the contact info
		* in order to set and effect the contact
		*/
		friend class ContactResolver;

	public:
		/*
		* holds the bodies that make contact
		* second can be null if contact is with scenary
		*/
		RigidBody* contact[2];

		real friction;

		real restitution;

		Vector3 contactPoint;

		Vector3 contactNormal;

		real penetration;

		/*
		* sets the data that doesn't normally depend on the contact 
		* position
		*/
		void setBodyData(RigidBody* one, RigidBody* two, real friction, real restitution);

	protected:
		/*
		* transform matrix that converts the coords int the contacts frame of 
		* refrence to world coordinates
		*/
		Matrix3 contactToWorld;

		/*
		* holds the closing velocity at the point of contact
		*/

		Vector3 contactVelocity;

		/*
		* required change in velocity after contact is resolved
		*/

		real desiredDeltaVelocity;

		/*
		* holds the world space position fo the contact point relative
		* to the center of each body
		*/

		Vector3 relativeContactPosition[2];

		/*
		* calculates the transform matrix that will convert coordinates from contact space
		* to world space
		*/
		void calculateContactBasis();

		/*
		* calculates the data that depends on the relative position of the 
		* contact to the bodies
		*/
		void calculateInternals(real duration);

		/*
		* updates the awake state of rigid boides that are
		* taking place in the given contact
		*/
		void matchAwakeState();

		/*
		* calculates and applies the impulse to seperatte the bodies
		*/
		void applyVelocityChange(Vector3 velocityChange[2], Vector3 rotationChange[2]);


		/*
		* calculates and applie the corrective position change to resolve interpenetration
		*/
		void applyPositonChange(Vector3 linearChange[2], Vector3 angularChange[2]);

		/*
		* calculates impulse needed to resolve velocity in fricionless state
		*/
		void calculateFrictionlessImpulse(Matrix3* inverseInertiaTensor);

		/*
		* calculates the impulse neede to resolve the velocity with friction
		*/
		Vector3 calculateFrictionImpulse(Matrix3* inertiaInverseTensor);

	};

	class ContactResolver {
	protected:
		/*
		* holds the number of iterations to perform when resolving velocity
		*/
		unsigned velocityIterations;
		/**
		 * Holds the number of iterations to perform when resolving
		 * position.
		 */
		unsigned positionIterations;

		/**
		 * Stores the number of velocity iterations used in the
		 * last call to resolve.
		 */
		unsigned velocityIterationsUsed;

		/**
		 * Stores the number of position iterations used in the
		 * last call to resolve.
		 */
		unsigned positionIterationsUsed;

	public:
		/**
		 * Creates a new contact resolver.
		 */
		ContactResolver(unsigned iterations, real velocityEpsilon = (real)0.01, real positionEpsilon = (real)0.01);

		/**
		 * Creates a new contact resolver with separate iteration counts
		 * for velocity and position.
		 */
		ContactResolver(unsigned velocityIterations, unsigned positionIterations, real velocityEpsilon = (real)0.01, real positionEpsilon = (real)0.01);

		/**
		 * Sets the number of iterations for each resolution stage.
		 */
		void setIterations(unsigned velocityIterations, unsigned positionIterations);

		/**
		 * Resolves a set of contacts for both penetration and velocity.
		 */
		void resolveContacts(Contact* contactArray, unsigned numContacts, real duration);

	protected:
		/**
		 * Sets up contacts ready for processing. This ensures that
		 * their internal data is configured correctly and the correct
		 * set of bodies is made alive.
		 */
		void prepareContacts(Contact* contactArray, unsigned numContacts, real duration);

		/**
		 * Resolves the velocity issues with the given array of constraints.
		 */
		void adjustVelocities(Contact* contactArray, unsigned numContacts, real duration);

		/**
		 * Resolves the positional issues with the given array of constraints.
		 */
		void adjustPositions(Contact* contacts, unsigned numContacts, real duration);

	};
}

#endif // !CYCLONE_CONTACTS_H
