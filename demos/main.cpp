#include <iostream>
#include <cyclone/core.h>



int main() {
	cyclone::Vector3 pos(1.0, 2.0, 3.0);
	cyclone::Vector3 velocity(4.0, 5.0, 6.0);
	std::cout << "Initial Position: (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";

	pos += velocity;

	std::cout << "Updated Position: (" << pos.x << ", " << pos.y << ", " << pos.z << ")\n";

	velocity *= 0.5;

	std::cout << "Scaled Velocity: (" << velocity.x << ", " << velocity.y << ", " << velocity.z << ")\n";

	cyclone::Vector3 v(3.0, 4.0, 0.0);
	std::cout << "Vector v Magnitude: " << v.magnitude() << "\n";

	v.normalize();

	std::cout << "Normalized Vector v: (" << v.x << ", " << v.y << ", " << v.z << ")\n";
}