#ifndef CYCLONE_COLLISION_FINE_H
#define CYCLONE_COLLISION_FINE_H

#include "contacts.h"

namespace cyclone {

    class IntersectionTests;
    class CollisionDetector;

 
    struct CollisionData {
        // Pointer to the start of the contact array
        Contact* contactArray;

        // Pointer to the current active contact (moves forward as we add contacts)
        Contact* contacts;

        // How many more contacts we can safely add before overflowing the array
        int contactsLeft;

        // How many contacts we have found so far
        unsigned contactCount;

        // The physics properties to apply to the generated contacts
        real friction;
        real restitution;

        bool hasMoreContacts() {
            return contactsLeft > 0;
        }

        void reset(unsigned maxContacts) {
            contactsLeft = maxContacts;
            contactCount = 0;
            contacts = contactArray;
        }

        void addContacts(unsigned count) {
            contactsLeft -= count;
            contactCount += count;
            contacts += count;
        }
    };

    /**
     * Represents a geometric shape attached to a rigid body.
     */
    class CollisionPrimitive {
    public:
        // The rigid body that this primitive represents.
        RigidBody* body;

        // The offset of this shape from the body's center of mass.
        Matrix4 offset;

        // Calculates the resultant transform (Body Transform * Offset)
        void calculateInternals();

        // Gets a specific local axis in world space (0=X, 1=Y, 2=Z)
        Vector3 getAxis(unsigned index) const {
            return transform.transformDirection(Vector3(
                index == 0 ? 1 : 0,
                index == 1 ? 1 : 0,
                index == 2 ? 1 : 0
            ));
        }

        const Matrix4& getTransform() const {
            return transform;
        }

    protected:
        // Cache: The primitive's actual position/rotation in the world.
        Matrix4 transform;
    };

    /**
     * A Sphere Primitive. Very fast for collision detection.
     */
    class CollisionSphere : public CollisionPrimitive {
    public:
        real radius;
    };

    /**
     * A Plane. It is not attached to a Rigid Body.
     * It represents immovable world geometry (like the ground).
     * Defined by a normal vector and a distance from the origin.
     */
    class CollisionPlane {
    public:
        Vector3 direction; // The normal of the plane
        real offset;       // Distance from origin along the normal
    };

    /**
     * An Oriented Bounding Box (OBB).
     */
    class CollisionBox : public CollisionPrimitive {
    public:
        // Holds the distance from the center to the edge along local X, Y, Z.
        // E.g., A 2x2x2 cube has halfSizes of (1, 1, 1).
        Vector3 halfSize;
    };

    /**
     * Fast intersection tests. These just return true if the objects
     * are intersecting, but don't calculate the exact contact point,
     * normal, or penetration depth.
     */
    class IntersectionTests {
    public:
        static bool sphereAndHalfSpace(const CollisionSphere& sphere, const CollisionPlane& plane);
        static bool sphereAndSphere(const CollisionSphere& one, const CollisionSphere& two);
        static bool boxAndBox(const CollisionBox& one, const CollisionBox& two);

        // A half-space is like a plane, but it considers everything "behind" the plane 
        // to be solid (like the ground).
        static bool boxAndHalfSpace(const CollisionBox& box, const CollisionPlane& plane);
    };

    /**
     * A wrapper class that holds the fine grained collision detection routines.
     * * Each function takes two shapes, checks for overlap, and if they touch,
     * writes the collision data (Point, Normal, Penetration) into CollisionData.
     */
    class CollisionDetector {
    public:
        static unsigned sphereAndHalfSpace(const CollisionSphere& sphere, const CollisionPlane& plane, CollisionData* data);
        static unsigned sphereAndTruePlane(const CollisionSphere& sphere, const CollisionPlane& plane, CollisionData* data);
        static unsigned sphereAndSphere(const CollisionSphere& one, const CollisionSphere& two, CollisionData* data);

        static unsigned boxAndHalfSpace(const CollisionBox& box, const CollisionPlane& plane, CollisionData* data);
        static unsigned boxAndBox(const CollisionBox& one, const CollisionBox& two, CollisionData* data);
        static unsigned boxAndPoint(const CollisionBox& box, const Vector3& point, CollisionData* data);
        static unsigned boxAndSphere(const CollisionBox& box, const CollisionSphere& sphere, CollisionData* data);
    };

} // namespace cyclone

#endif // CYCLONE_COLLISION_FINE_H