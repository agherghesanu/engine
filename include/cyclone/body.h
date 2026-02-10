#ifndef CYCLONE_BODY_H
#define CYCLONE_BODY_H

#include "core.h"

namespace cyclone {
	/*
	* the rigid body - basic simuation object in the engine
	* position and orientation, reacts to forces and torques, can be rendered
	*/
	class RigidBody {
	protected:
		real inverseMass;
		Vector3 position;
		Vector3 velocity;
		Vector3 accelaration;
		Vector3 lastFrameAccelaration;

		real linearDamping;

		Quaternion orientation; //angular state

		/*
		* angular velocity
		* driection axis of rotation
		* magnitude: speed of rotation
		*/
		Vector3 rotation;

		/*
		* describes the objects "rotational mass"
		*/
		Matrix3 inverseInertiaTensor;

		/*
		* as the object tubles, its resistance to rotation changes relative to the
		* world axes
		* needs to be updated evrey frame
		*/
		Matrix3 inverseInertiaTensorWorld;
		
		real angularDamping;

		/*
		* a transform matrix for converting body space intro world space
		* calculated from position and orientation
		*/
		Matrix4 transformMatrix;

		Vector3 forceAccum;
		Vector3 torqueAccum;

	public:
		RigidBody();

		/*
		* integration
		*/

		/*
		* calcualte the derived data -> transform matrix and world inertioa from internal world state
		* needs to be called after manually setting postion/orienation
		*/
		void calculateDerivedData();

		/*
		* simulates the object forward in time
		*/
		void integrate(real duration);

		/*
		* add force to the center of mass
		*/

		void addForce(const Vector3 &force);


		/*
		* add force at a specific point in the world
		* generateds both linear acc and torque
		*/

		void addForceAtPoint(const Vector3& force, const Vector3& point);

		/*
		* add force at a point on the body relative to the local coordinates
		*/

		void addForceAtBodyPoint(const Vector3& force, const Vector3& point);

		void clearAccumulators();

		void setMass(real mass);
		real getMass() const;
		void setInverseMass(real inverseMass);
		real getInverseMass() const;

		bool hasFiniteMass() const;

		void setInertiaTensor(const Matrix3& inertiaTensor);
		void getInertiaTensor(Matrix3* inertiaTensor) const;
		void getInertiaTensorWorld(Matrix3* inertiaTensor) const;

		void setDamping(real linearDamping, real angularDamping);

		void setPosition(const Vector3& position);
		void setPosition(const real x, const real y, const real z);
		void getPosition(Vector3* position) const;
		Vector3 getPosition() const;

		void setOrientation(const Quaternion& orientation);
		void getOrientation(Quaternion* orientation) const;
		Quaternion getOrientation() const;
		void getOrientation(Matrix3* matrix) const;
		void getOrientation(real matrix[9]) const; // OpenGL helper

		void getTransform(Matrix4* transform) const;
		void getGLTransform(float matrix[16]) const; // OpenGL helper
		Matrix4 getTransform() const;

		void setVelocity(const Vector3& velocity);
		void setVelocity(const real x, const real y, const real z);
		Vector3 getVelocity() const;
		void addVelocity(const Vector3& deltaVelocity);

		void setRotation(const Vector3& rotation);
		void setRotation(const real x, const real y, const real z);
		Vector3 getRotation() const;
		void addRotation(const Vector3& deltaRotation);

		// Additional helpers for points
		Vector3 getPointInLocalSpace(const Vector3& point) const;
		Vector3 getPointInWorldSpace(const Vector3& point) const;
		Vector3 getDirectionInLocalSpace(const Vector3& direction) const;
		Vector3 getDirectionInWorldSpace(const Vector3& direction) const;


	};
}

#endif