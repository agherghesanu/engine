#include <cyclone/body.h>
#include <memory.h>

using namespace cyclone;

RigidBody::RigidBody() :
	inverseMass(1.0),
	linearDamping(0.99),
	angularDamping(0.8) {
	position = Vector3(0, 0, 0);
	orientation = Quaternion(1, 0, 0, 0);
	velocity = Vector3(0, 0, 0);
	rotation = Vector3(0, 0, 0);
	inverseInertiaTensor = Matrix3(1, 0, 0, 0, 1, 0, 0, 0, 1);

	calculateDerivedData();
}

void RigidBody::calculateDerivedData() {
	orientation.normalize();

	transformMatrix.setOrientationAndPos(orientation, position);

	Matrix3 rot;
	rot.setOrientation(orientation);

	Matrix3 iitWorld = rot * inverseInertiaTensor;


	real r0 = rot.data[0], r1 = rot.data[1], r2 = rot.data[2];
	real r3 = rot.data[3], r4 = rot.data[4], r5 = rot.data[5];
	real r6 = rot.data[6], r7 = rot.data[7], r8 = rot.data[8];

	inverseInertiaTensorWorld.data[0] = iitWorld.data[0] * r0 + iitWorld.data[1] * r1 + iitWorld.data[2] * r2;
	inverseInertiaTensorWorld.data[1] = iitWorld.data[0] * r3 + iitWorld.data[1] * r4 + iitWorld.data[2] * r5;
	inverseInertiaTensorWorld.data[2] = iitWorld.data[0] * r6 + iitWorld.data[1] * r7 + iitWorld.data[2] * r8;

	inverseInertiaTensorWorld.data[3] = iitWorld.data[3] * r0 + iitWorld.data[4] * r1 + iitWorld.data[5] * r2;
	inverseInertiaTensorWorld.data[4] = iitWorld.data[3] * r3 + iitWorld.data[4] * r4 + iitWorld.data[5] * r5;
	inverseInertiaTensorWorld.data[5] = iitWorld.data[3] * r6 + iitWorld.data[4] * r7 + iitWorld.data[5] * r8;

	inverseInertiaTensorWorld.data[6] = iitWorld.data[6] * r0 + iitWorld.data[7] * r1 + iitWorld.data[8] * r2;
	inverseInertiaTensorWorld.data[7] = iitWorld.data[6] * r3 + iitWorld.data[7] * r4 + iitWorld.data[8] * r5;
	inverseInertiaTensorWorld.data[8] = iitWorld.data[6] * r6 + iitWorld.data[7] * r7 + iitWorld.data[8] * r8;
}

void RigidBody::integrate() {
	if (inverseMass <= 0.0)
		return; // imovable object

	lastFrameAccelaration = accelaration;
	lastFrameAccelaration += forceAccum * inverseMass;

	velocity += lastFrameAccelaration * duration;

	velocity *= std::pow(linearDamping, duration);

	position += velocity * duration;

	Vector3 angularAcceleration = inverseInertiaTensorWorld * torqueAccum;

	rotation += angularAcceleration * duration;

	rotation *= std::pow(angularDamping, duration);

	orientation.addScaledVector(rotation, duration);

	calculateDerivedData();

	clearAccumulators();
}

void RigidBody::addForce(const Vector3& force) {
	forceAccum += force;
}

void RigidBody::addForceAtPoint(const Vector3& force, const Vector3& point) {
	forceAccum += force;

	Vector3 pt = point;

	pt -= position;

	torqueAccum += pt % force;
}

void RigidBody::addForceAtBodyPoint(const Vector3& force, const Vector3& point) {
	Vector3 pt = getPointinWorldSpace(point);
}

void RigidBody::addForceAtBodyPoint(const Vector3& force, const Vector3& point)
{
    // Convert the local point (e.g., 1,0,0) to world coordinates
    Vector3 pt = getPointInWorldSpace(point);
    addForceAtPoint(force, pt);
}

void RigidBody::clearAccumulators()
{
    forceAccum = Vector3(0, 0, 0);
    torqueAccum = Vector3(0, 0, 0);
}

/*
 * ============================================================================
 * ACCESSORS (GETTERS/SETTERS)
 * ============================================================================
 */

void RigidBody::setMass(real mass) {
    assert(mass != 0); // Mass cannot be zero (use setInverseMass(0) for immovable)
    inverseMass = 1.0f / mass;
}

real RigidBody::getMass() const {
    if (inverseMass == 0) return DBL_MAX; // Represent infinite mass
    return 1.0f / inverseMass;
}

void RigidBody::setInverseMass(real inverseMass) {
    RigidBody::inverseMass = inverseMass;
}

real RigidBody::getInverseMass() const {
    return inverseMass;
}

bool RigidBody::hasFiniteMass() const {
    return inverseMass >= 0.0f;
}

void RigidBody::setInertiaTensor(const Matrix3& inertiaTensor) {
    // We store the INVERSE inertia tensor because physics uses (1/I) * Torque.
    // In a full engine, we might calculate the inverse here.
    // For this engine, we assume the user passes the inverse directly.
    inverseInertiaTensor = inertiaTensor;
}

void RigidBody::getInertiaTensor(Matrix3* inertiaTensor) const {
    *inertiaTensor = inverseInertiaTensor;
}

void RigidBody::getInertiaTensorWorld(Matrix3* inertiaTensor) const {
    *inertiaTensor = inverseInertiaTensorWorld;
}

void RigidBody::setDamping(real linearDamping, real angularDamping) {
    RigidBody::linearDamping = linearDamping;
    RigidBody::angularDamping = angularDamping;
}

void RigidBody::setPosition(const Vector3& position) {
    RigidBody::position = position;
}

void RigidBody::setPosition(const real x, const real y, const real z) {
    position.x = x;
    position.y = y;
    position.z = z;
}

void RigidBody::getPosition(Vector3* position) const {
    *position = RigidBody::position;
}

Vector3 RigidBody::getPosition() const {
    return position;
}

void RigidBody::setOrientation(const Quaternion& orientation) {
    RigidBody::orientation = orientation;
    RigidBody::orientation.normalize(); // Must keep orientation normalized
}

void RigidBody::getOrientation(Quaternion* orientation) const {
    *orientation = RigidBody::orientation;
}

Quaternion RigidBody::getOrientation() const {
    return orientation;
}

// Fills a matrix with the orientation rotation (useful for rendering)
void RigidBody::getOrientation(real matrix[9]) const {
    matrix[0] = transformMatrix.data[0];
    matrix[1] = transformMatrix.data[1];
    matrix[2] = transformMatrix.data[2];

    matrix[3] = transformMatrix.data[4];
    matrix[4] = transformMatrix.data[5];
    matrix[5] = transformMatrix.data[6];

    matrix[6] = transformMatrix.data[8];
    matrix[7] = transformMatrix.data[9];
    matrix[8] = transformMatrix.data[10];
}

void RigidBody::getTransform(Matrix4* transform) const {
    *transform = transformMatrix;
}

Matrix4 RigidBody::getTransform() const {
    return transformMatrix;
}

// Converts the Transform Matrix to an OpenGL compatible array (column-major)
void RigidBody::getGLTransform(float matrix[16]) const {
    matrix[0] = (float)transformMatrix.data[0];
    matrix[1] = (float)transformMatrix.data[4];
    matrix[2] = (float)transformMatrix.data[8];
    matrix[3] = 0;

    matrix[4] = (float)transformMatrix.data[1];
    matrix[5] = (float)transformMatrix.data[5];
    matrix[6] = (float)transformMatrix.data[9];
    matrix[7] = 0;

    matrix[8] = (float)transformMatrix.data[2];
    matrix[9] = (float)transformMatrix.data[6];
    matrix[10] = (float)transformMatrix.data[10];
    matrix[11] = 0;

    matrix[12] = (float)transformMatrix.data[3];
    matrix[13] = (float)transformMatrix.data[7];
    matrix[14] = (float)transformMatrix.data[11];
    matrix[15] = 1;
}

void RigidBody::setVelocity(const Vector3& velocity) {
    RigidBody::velocity = velocity;
}

void RigidBody::setVelocity(const real x, const real y, const real z) {
    velocity.x = x;
    velocity.y = y;
    velocity.z = z;
}

Vector3 RigidBody::getVelocity() const {
    return velocity;
}

void RigidBody::addVelocity(const Vector3& deltaVelocity) {
    velocity += deltaVelocity;
}

void RigidBody::setRotation(const Vector3& rotation) {
    RigidBody::rotation = rotation;
}

void RigidBody::setRotation(const real x, const real y, const real z) {
    rotation.x = x;
    rotation.y = y;
    rotation.z = z;
}

Vector3 RigidBody::getRotation() const {
    return rotation;
}

void RigidBody::addRotation(const Vector3& deltaRotation) {
    rotation += deltaRotation;
}


Vector3 RigidBody::getPointInLocalSpace(const Vector3& point) const
{
    Matrix4 transform = transformMatrix;
    Vector3 temp = point;
    temp.x -= transform.data[3];
    temp.y -= transform.data[7];
    temp.z -= transform.data[11];

    return Vector3(
        temp.x * transform.data[0] + temp.y * transform.data[4] + temp.z * transform.data[8],
        temp.x * transform.data[1] + temp.y * transform.data[5] + temp.z * transform.data[9],
        temp.x * transform.data[2] + temp.y * transform.data[6] + temp.z * transform.data[10]
    );
}

Vector3 RigidBody::getPointInWorldSpace(const Vector3& point) const
{
    return transformMatrix.transform(point);
}

Vector3 RigidBody::getDirectionInLocalSpace(const Vector3& direction) const
{
    Matrix4 transform = transformMatrix;
    return Vector3(
        direction.x * transform.data[0] + direction.y * transform.data[4] + direction.z * transform.data[8],
        direction.x * transform.data[1] + direction.y * transform.data[5] + direction.z * transform.data[9],
        direction.x * transform.data[2] + direction.y * transform.data[6] + direction.z * transform.data[10]
    );
}

Vector3 RigidBody::getDirectionInWorldSpace(const Vector3& direction) const
{
    return transformMatrix.transformDirection(direction);
}

