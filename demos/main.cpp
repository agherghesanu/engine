#include <cyclone/core.h>
#include <cyclone/body.h>

#include <iostream>
#include <iomanip>

using namespace cyclone;

using namespace std;

void printVector(const char* label, const Vector3& v) {
    cout << label << ": ["
        << fixed << setprecision(2) << v.x << ", "
        << v.y << ", "
        << v.z << "]";
}

// Helper to print quaternion cleanly
void printQuat(const char* label, const Quaternion& q) {
    cout << label << ": ["
        << fixed << setprecision(2) << q.r << ", "
        << q.i << ", "
        << q.j << ", "
        << q.k << "]";
}

int main() {
	RigidBody body;

	body.setPosition(0, 0, 0);

    body.setOrientation(Quaternion(1, 0, 0, 0));

    body.setMass(2.0);

	body.setDamping(0.99, 0.99);

    Matrix3 tensor;
	tensor.setInertiaTensorCoeffs(0.333, 0.333, 0.333); // cube with 0.5 side

    tensor.invert();
    body.setInertiaTensor(tensor);

    Vector3 force(0, 0, 100);
    Vector3 point(0.5, 0.5, 0.5);

	body.addForceAtPoint(force, point);

	real duration = 0.01; // 10 ms time step

    for (int i = 0; i < 400; i++) {
        body.integrate(duration);

        // Print every 0.1 seconds
        if (i % 10 == 0) {
            cout << "T=" << setw(4) << (i * duration) << "s | ";
            printVector("Pos", body.getPosition());
            cout << " | ";
            printQuat("Ori", body.getOrientation());
            cout << endl;
        }

        // Note: The force is only applied once at the start.
        // After frame 0, 'forceAccum' is cleared, so the box 
        // will drift and spin based on its momentum.
    }



}




