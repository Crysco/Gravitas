#include "gContact.h"

using namespace Gravitas;

gContact::gContact() : penetration(0), friction(0), restitution(0.8), contact_point()
{
	body[0] = NULL;
	body[1] = NULL;
}

void gContact::setBodyData(gBody* body_1, gBody* body_2, real friction, real restitution)
{
	body[0] = body_1;
	body[1] = body_2;
	gContact::friction = friction;
	gContact::restitution = restitution;
}

void gContact::calculateContactAxes() 
{
	// Check whether the Z-axis is nearer to the X or Y axis
	if (abs(contact_normal.x) > abs(contact_normal.y))
	{
		// Scaling factor to ensure the results are normalised
		const real s = (real)1.0f / sqrtf(contact_normal.z*contact_normal.z + contact_normal.x*contact_normal.x);

		// The new X-axis is at right angles to the world Y-axis
		contact_tangents[0].x = contact_normal.z*s;
		contact_tangents[0].y = 0;
		contact_tangents[0].z = -contact_normal.x*s;

		// The new Y-axis is at right angles to the new X- and Z- axes
		contact_tangents[1].x = contact_normal.y*contact_tangents[0].x;
		contact_tangents[1].y = contact_normal.z*contact_tangents[0].x - contact_normal.x*contact_tangents[0].z;
		contact_tangents[1].z = -contact_normal.y*contact_tangents[0].x;
	}
	else
	{
		// Scaling factor to ensure the results are normalised
		const real s = (real)1.0 / sqrtf(contact_normal.z*contact_normal.z +
			contact_normal.y*contact_normal.y);

		// The new X-axis is at right angles to the world X-axis
		contact_tangents[0].x = 0;
		contact_tangents[0].y = -contact_normal.z*s;
		contact_tangents[0].z = contact_normal.y*s;

		// The new Y-axis is at right angles to the new X- and Z- axes
		contact_tangents[1].x = contact_normal.y*contact_tangents[0].z - contact_normal.z*contact_tangents[0].y;
		contact_tangents[1].y = -contact_normal.x*contact_tangents[0].z;
		contact_tangents[1].z = contact_normal.x*contact_tangents[0].y;
	}
}

void gContact::calculateVelocityChanges(real duration)
{
	// local velocity of bodies 
	gVector3 local_vel[2];
	local_vel[0].x = body[0]->getLinearAcceleration() * duration * contact_normal;
	local_vel[0].y = body[0]->getLinearAcceleration() * duration * contact_tangents[0];
	local_vel[0].z = body[0]->getLinearAcceleration() * duration * contact_tangents[1];
	
	local_vel[1].x = body[0]->getLinearAcceleration() * duration * contact_normal;
	local_vel[1].y = body[0]->getLinearAcceleration() * duration * contact_tangents[0];
	local_vel[1].z = body[0]->getLinearAcceleration() * duration * contact_tangents[1];

	closing_velocity = local_vel[0] - local_vel[1];

	real impulse = calculateImpulseMagnitude(local_vel[0], local_vel[1]);
}

real& gContact::calculateImpulseMagnitude(gVector3& vel_1, gVector3& vel_2)
{
	real k = (closing_velocity * contact_normal * (restitution + (real)1.0)) / ((body[0]->getInverseMass + body[1]->getInverseMass) * contact_normal * contact_normal);
	
	return k;
}