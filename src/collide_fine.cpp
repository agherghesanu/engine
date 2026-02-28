#include <cyclone/collide_fine.h>
#include <assert.h>
#include <cmath>

using namespace cyclone;

void CollisionPrimitive::calculateInternals() {


    /*
    * calculate the transform matrix for this shape
    * it translates rotates the shape into the world space on the rigid body
    */
	Matrix4 bodyTransform = body->getTransform();

    transform.data[0] = bodyTransform.data[0] * offset.data[0] + bodyTransform.data[1] * offset.data[4] + bodyTransform.data[2] * offset.data[8];
    transform.data[1] = bodyTransform.data[0] * offset.data[1] + bodyTransform.data[1] * offset.data[5] + bodyTransform.data[2] * offset.data[9];
    transform.data[2] = bodyTransform.data[0] * offset.data[2] + bodyTransform.data[1] * offset.data[6] + bodyTransform.data[2] * offset.data[10];
    transform.data[3] = bodyTransform.data[0] * offset.data[3] + bodyTransform.data[1] * offset.data[7] + bodyTransform.data[2] * offset.data[11] + bodyTransform.data[3];

    transform.data[4] = bodyTransform.data[4] * offset.data[0] + bodyTransform.data[5] * offset.data[4] + bodyTransform.data[6] * offset.data[8];
    transform.data[5] = bodyTransform.data[4] * offset.data[1] + bodyTransform.data[5] * offset.data[5] + bodyTransform.data[6] * offset.data[9];
    transform.data[6] = bodyTransform.data[4] * offset.data[2] + bodyTransform.data[5] * offset.data[6] + bodyTransform.data[6] * offset.data[10];
    transform.data[7] = bodyTransform.data[4] * offset.data[3] + bodyTransform.data[5] * offset.data[7] + bodyTransform.data[6] * offset.data[11] + bodyTransform.data[7];

    transform.data[8] = bodyTransform.data[8] * offset.data[0] + bodyTransform.data[9] * offset.data[4] + bodyTransform.data[10] * offset.data[8];
    transform.data[9] = bodyTransform.data[8] * offset.data[1] + bodyTransform.data[9] * offset.data[5] + bodyTransform.data[10] * offset.data[9];
    transform.data[10] = bodyTransform.data[8] * offset.data[2] + bodyTransform.data[9] * offset.data[6] + bodyTransform.data[10] * offset.data[10];
    transform.data[11] = bodyTransform.data[8] * offset.data[3] + bodyTransform.data[9] * offset.data[7] + bodyTransform.data[10] * offset.data[11] + bodyTransform.data[11];

}

bool IntersectionTests::sphereAndHalfSpace(const CollisionSphere& sphere, const CollisionPlane& plane) {
    // Extract the position of the sphere from its transform matrix
    Vector3 position(sphere.getTransform().data[3], sphere.getTransform().data[7], sphere.getTransform().data[11]);

    // Find the distance from the origin
    real ballDistance = plane.direction * position - sphere.radius;

    // Check for intersection
    return ballDistance <= plane.offset;
}

bool IntersectionTests::sphereAndSphere(const CollisionSphere& one, const CollisionSphere& two) {
    // Find the vectors between the centers
    Vector3 positionOne(one.getTransform().data[3], one.getTransform().data[7], one.getTransform().data[11]);
    Vector3 positionTwo(two.getTransform().data[3], two.getTransform().data[7], two.getTransform().data[11]);

    Vector3 midline = positionOne - positionTwo;

    // See if it is large enough
    return midline.squareMagnitude() < (one.radius + two.radius) * (one.radius + two.radius);
}

unsigned CollisionDetector::sphereAndHalfSpace(const CollisionSphere& sphere, const CollisionPlane& plane, CollisionData* data) {
    // Make sure we have space for the contact
    if (data->contactsLeft <= 0) return 0;

    // Get position of the sphere
    Vector3 position(sphere.getTransform().data[3], sphere.getTransform().data[7], sphere.getTransform().data[11]);

    // Calculate distance from plane
    real ballDistance = plane.direction * position - plane.offset;

    if (ballDistance <= sphere.radius) {
        // We have a collision! Create the contact.
        Contact* contact = data->contacts;

        // The normal of the collision is the normal of the plane
        contact->contactNormal = plane.direction;

        // The penetration is how far the sphere crossed the threshold
        contact->penetration = sphere.radius - ballDistance;

        // The point of contact is halfway between the edge of the sphere and the plane
        contact->contactPoint = position - plane.direction * (ballDistance + sphere.radius) * 0.5;

        // Assign the bodies (body[1] is null because the plane is immovable scenery)
        contact->contact[0] = sphere.body;
        contact->contact[1] = nullptr;
        contact->friction = data->friction;
        contact->restitution = data->restitution;

        data->addContacts(1);
        return 1;
    }
    return 0;
}

unsigned CollisionDetector::sphereAndTruePlane(const CollisionSphere& sphere, const CollisionPlane& plane, CollisionData* data) {
    if (data->contactsLeft <= 0) return 0;

    Vector3 position(sphere.getTransform().data[3], sphere.getTransform().data[7], sphere.getTransform().data[11]);

    // For a true plane, we check distance on BOTH sides (absolute value)
    real centerDistance = plane.direction * position - plane.offset;

    if (centerDistance * centerDistance <= sphere.radius * sphere.radius) {
        Contact* contact = data->contacts;

        // If the sphere is behind the plane, the normal needs to be flipped
        Vector3 normal = plane.direction;
        real penetration = -centerDistance;
        if (centerDistance < 0) {
            normal.invert();
            penetration = -penetration;
        }
        penetration += sphere.radius;

        contact->contactNormal = normal;
        contact->penetration = penetration;
        contact->contactPoint = position - plane.direction * centerDistance;
        contact->contact[0] = sphere.body;
        contact->contact[1] = nullptr;
        contact->friction = data->friction;
        contact->restitution = data->restitution;

        data->addContacts(1);
        return 1;
    }
    return 0;
}

unsigned CollisionDetector::sphereAndSphere(const CollisionSphere& one, const CollisionSphere& two, CollisionData* data) {
    if (data->contactsLeft <= 0) return 0;

    Vector3 positionOne(one.getTransform().data[3], one.getTransform().data[7], one.getTransform().data[11]);
    Vector3 positionTwo(two.getTransform().data[3], two.getTransform().data[7], two.getTransform().data[11]);

    Vector3 midline = positionOne - positionTwo;
    real size = midline.magnitude();

    // Check if there is an intersection
    if (size <= 0.0f || size >= one.radius + two.radius) {
        return 0;
    }

    Contact* contact = data->contacts;

    // Normal points from Two to One
    contact->contactNormal = midline * (1.0f / size);

    // Point of contact is between them
    contact->contactPoint = positionOne + midline * 0.5f;

    // Penetration is overlaps
    contact->penetration = (one.radius + two.radius) - size;

    contact->contact[0] = one.body;
    contact->contact[1] = two.body;
    contact->friction = data->friction;
    contact->restitution = data->restitution;

    data->addContacts(1);
    return 1;
}

/**
 * Helper function: Projects a Box onto a given axis.
 * Returns the "radius" (half-width) of the box along that specific axis.
 */
static inline real transformToAxis(const CollisionBox& box, const Vector3& axis) {
    // We project each local axis of the box onto the test axis, 
    // scaled by the box's size along that local axis.
    return
        box.halfSize.x * std::abs(axis * box.getAxis(0)) +
        box.halfSize.y * std::abs(axis * box.getAxis(1)) +
        box.halfSize.z * std::abs(axis * box.getAxis(2));
}

/**
 * Helper function: Tests a single axis to see if the boxes overlap.
 * Returns the penetration depth (negative if they don't overlap).
 */
static inline real penetrationOnAxis(
    const CollisionBox& one, const CollisionBox& two, const Vector3& axis, const Vector3& toCentre
) {
    // Project the half-sizes of both boxes onto the test axis
    real oneProject = transformToAxis(one, axis);
    real twoProject = transformToAxis(two, axis);

    // Project the distance between the two centers onto the test axis
    real distance = std::abs(toCentre * axis);

    // Return the overlap (positive = intersecting, negative = separated)
    return oneProject + twoProject - distance;
}

static inline bool tryAxis(
    const CollisionBox& one, const CollisionBox& two, Vector3 axis, const Vector3& toCentre,
    unsigned index, real& smallestPenetration, unsigned& smallestCase
) {
    // Make sure we have a valid axis (length > 0)
    // Cross products of parallel edges result in a zero-vector, which we ignore.
    if (axis.squareMagnitude() < 0.0001) return true;
    axis.normalize();

    real penetration = penetrationOnAxis(one, two, axis, toCentre);

    // If penetration is negative, there is a gap of light! No collision.
    if (penetration < 0) return false;

    // If it's positive, we keep track of the SMALLEST overlap.
    if (penetration < smallestPenetration) {
        smallestPenetration = penetration;
        smallestCase = index;
    }
    return true;
}



unsigned CollisionDetector::boxAndHalfSpace(const CollisionBox& box, const CollisionPlane& plane, CollisionData* data) {
    if (data->contactsLeft <= 0) return 0;

    // Check for intersection: Project the box onto the plane's normal
    real projectedRadius = transformToAxis(box, plane.direction);

    // Calculate how far the center of the box is from the plane
    Vector3 boxPos(box.getTransform().data[3], box.getTransform().data[7], box.getTransform().data[11]);
    real boxDistance = (plane.direction * boxPos) - plane.offset;

    // If the distance is less than the projected radius, we have a collision
    if (boxDistance <= projectedRadius) {
        Contact* contact = data->contacts;

        // The normal of the bounce is the plane's normal
        contact->contactNormal = plane.direction;
        contact->penetration = projectedRadius - boxDistance;

        // Calculate the contact point. 
        // We find the corner of the box that is pushing deepest into the plane.
        // We do this by checking the direction of each local axis against the plane normal.
        Vector3 localVertex(
            (box.getAxis(0) * plane.direction < 0) ? box.halfSize.x : -box.halfSize.x,
            (box.getAxis(1) * plane.direction < 0) ? box.halfSize.y : -box.halfSize.y,
            (box.getAxis(2) * plane.direction < 0) ? box.halfSize.z : -box.halfSize.z
        );

        // Convert that local corner into World Space
        contact->contactPoint = box.getTransform().transform(localVertex);

        // Assign bodies and physics properties
        contact->contact[0] = box.body;
        contact->contact[1] = nullptr; // Plane is immovable scenery
        contact->friction = data->friction;
        contact->restitution = data->restitution;

        data->addContacts(1);
        return 1;
    }
    return 0;
}

unsigned CollisionDetector::boxAndBox(const CollisionBox& one, const CollisionBox& two, CollisionData* data) {
    if (data->contactsLeft <= 0) return 0;

    // Find the vector between the two centers
    Vector3 centerOne(one.getTransform().data[3], one.getTransform().data[7], one.getTransform().data[11]);
    Vector3 centerTwo(two.getTransform().data[3], two.getTransform().data[7], two.getTransform().data[11]);
    Vector3 toCenter = centerTwo - centerOne;

    // We assume there is no collision, until we find the smallest overlap
    real pen = DBL_MAX;
    unsigned best = 0xffffff;

    // Test the 3 Face Axes of Box One
    if (!tryAxis(one, two, one.getAxis(0), toCenter, 0, pen, best)) return 0;
    if (!tryAxis(one, two, one.getAxis(1), toCenter, 1, pen, best)) return 0;
    if (!tryAxis(one, two, one.getAxis(2), toCenter, 2, pen, best)) return 0;

    // Test the 3 Face Axes of Box Two
    if (!tryAxis(one, two, two.getAxis(0), toCenter, 3, pen, best)) return 0;
    if (!tryAxis(one, two, two.getAxis(1), toCenter, 4, pen, best)) return 0;
    if (!tryAxis(one, two, two.getAxis(2), toCenter, 5, pen, best)) return 0;

    // Test the 9 Edge-to-Edge Cross Products 

    if (!tryAxis(one, two, one.getAxis(0) ^ two.getAxis(0), toCenter, 6, pen, best)) return 0;
    if (!tryAxis(one, two, one.getAxis(0) ^ two.getAxis(1), toCenter, 7, pen, best)) return 0;
    if (!tryAxis(one, two, one.getAxis(0) ^ two.getAxis(2), toCenter, 8, pen, best)) return 0;
    if (!tryAxis(one, two, one.getAxis(1) ^ two.getAxis(0), toCenter, 9, pen, best)) return 0;
    if (!tryAxis(one, two, one.getAxis(1) ^ two.getAxis(1), toCenter, 10, pen, best)) return 0;
    if (!tryAxis(one, two, one.getAxis(1) ^ two.getAxis(2), toCenter, 11, pen, best)) return 0;
    if (!tryAxis(one, two, one.getAxis(2) ^ two.getAxis(0), toCenter, 12, pen, best)) return 0;
    if (!tryAxis(one, two, one.getAxis(2) ^ two.getAxis(1), toCenter, 13, pen, best)) return 0;
    if (!tryAxis(one, two, one.getAxis(2) ^ two.getAxis(2), toCenter, 14, pen, best)) return 0;

    // If we made it here, there is a collision! All 15 axes have an overlap.
    assert(best != 0xffffff);

    Contact* contact = data->contacts;

    // Calculate the normal based on the best axis we found
    Vector3 normal;
    if (best < 3) {
        normal = one.getAxis(best);
    }
    else if (best < 6) {
        normal = two.getAxis(best - 3);
    }
    else {
        // Edge-Edge contact normal is the cross product of the two edges
        int oneAxisIndex = (best - 6) / 3;
        int twoAxisIndex = (best - 6) % 3;
        normal = one.getAxis(oneAxisIndex) ^ two.getAxis(twoAxisIndex);
    }

    normal.normalize();

    // Ensure the normal points from Box 1 to Box 2
    if ((normal * toCenter) > 0) {
        normal.invert();
    }

    contact->contactNormal = normal;
    contact->penetration = pen;

    // (Simplified midpoint contact for Engine stability without heavy Vertex Math)
    contact->contactPoint = centerOne + (toCenter * 0.5);

    contact->contact[0] = one.body;
    contact->contact[1] = two.body;
    contact->friction = data->friction;
    contact->restitution = data->restitution;

    data->addContacts(1);
    return 1;
}