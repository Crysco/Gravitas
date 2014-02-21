#include "gBody.h"
#include <memory.h>

using namespace Gravitas;

/**
* Calculates the transformation matrix of a body that holds its position and orienation in world coordinates.
*
* @param orient: Input the orientation of the body.
* @param pos: Input the position of the body.
*/
static inline void _calculateTransformMatrix(gMatrix4& transform_matrix, const gQuaternion& orient, const gVector3& pos)
{
	transform_matrix.data[0] = 1 - 2 * orient.y * orient.y - 2 * orient.z * orient.z;
	transform_matrix.data[1] = 2 * orient.x * orient.y - 2 * orient.z * orient.w;
	transform_matrix.data[2] = 2 * orient.x * orient.w + 2 * orient.y * orient.w;
	transform_matrix.data[3] = pos.x;

	transform_matrix.data[4] = 2 * orient.x * orient.y + 2 * orient.z * orient.w;
	transform_matrix.data[5] = 1 - 2 * orient.x * orient.x - 2 * orient.z * orient.z;
	transform_matrix.data[6] = 2 * orient.y * orient.z - 2 * orient.x * orient.w;
	transform_matrix.data[7] = pos.y;

	transform_matrix.data[8] = 2 * orient.x * orient.z - 2 * orient.y * orient.w;
	transform_matrix.data[9] = 2 * orient.y * orient.z + 2 * orient.x * orient.w;
	transform_matrix.data[10] = 1 - 2 * orient.x * orient.x - 2 * orient.y * orient.y;
	transform_matrix.data[11] = pos.z;
}

gBody::gBody() :
	position(), linear_velocity(), linear_acceleration(),
	orientation(), angular_velocity(), angular_acceleration(),
	transform_matrix(), mass(0), mass_center(), inertia_tensor(),
	has_finite_mass(true)
{
}

void gBody::integrate(real duration)
{
	linear_acceleration = total_force * inverse_mass;
	angular_acceleration = inverse_inertia_tensor.transform(total_torque);

	linear_velocity += linear_acceleration * duration;
	angular_velocity += angular_acceleration * duration;

	position += linear_velocity * duration;
	orientation += angular_velocity * duration;

	_calculateTransformMatrix(transform_matrix, orientation, position);

	clearTotals();
}

gVector3 gBody::convertLocalToWorld(const gVector3& point)
{
	gVector3 pt = gBody::transform_matrix.transform(point);
	return pt;
}

gVector3 gBody::convertWorldToLocal(const gVector3& point)
{
	gVector3 pt = gBody::transform_matrix.transformInverse(point);
	return pt;
}

gVector3 gBody::getPosition() const
{
	return position;
}

void gBody::getPosition(gVector3& pos)
{
	pos = gBody::position;
}

gQuaternion gBody::getOrientation() const
{
	return orientation;
}

void gBody::getOrientation(gQuaternion& quat)
{
	quat = gBody::orientation;
}

gMatrix4 gBody::getTransformMatrix() const
{
	return transform_matrix;
}

void gBody::getTransformMatrix(gMatrix4& mat)
{
	memcpy(&mat, &transform_matrix.data, sizeof(gMatrix4));
}

void gBody::getGLTransformMatrix(real matrix[16])
{
	matrix[0] = transform_matrix.data[0];
	matrix[4] = transform_matrix.data[1];
	matrix[8] = transform_matrix.data[2];
	matrix[12] = transform_matrix.data[3];

	matrix[1] = transform_matrix.data[4];
	matrix[5] = transform_matrix.data[5];
	matrix[9] = transform_matrix.data[6];
	matrix[13] = transform_matrix.data[7];

	matrix[2] = transform_matrix.data[8];
	matrix[6] = transform_matrix.data[9];
	matrix[10] = transform_matrix.data[10];
	matrix[14] = transform_matrix.data[11];

	matrix[3] = transform_matrix.data[12];
	matrix[7] = transform_matrix.data[13];
	matrix[11] = transform_matrix.data[14];
	matrix[15] = transform_matrix.data[15];
}

gVector3 gBody::getLinearVelocity() const
{
	return linear_velocity;
}

void gBody::getLinearVelocity(gVector3& vec)
{
	vec = gBody::linear_velocity;
}

gVector3 gBody::getAngularVelocity() const
{
	return angular_velocity;
}

void gBody::getAngularVelocity(gVector3& vec)
{
	vec = gBody::angular_velocity;
}

gVector3 gBody::getLinearAcceleration() const
{
	return linear_acceleration;
}

void gBody::getLinearAcceleration(gVector3& vec)
{
	vec = gBody::linear_acceleration;
}

gVector3 gBody::getAngularAcceleration() const
{
	return angular_acceleration;
}

void gBody::getAngularAcceleration(gVector3& vec)
{
	vec = gBody::angular_acceleration;
}

void gBody::setPosition(real x, real y, real z)
{
	position.x = x;
	position.y = y;
	position.z = z;
}

void gBody::setPosition(const gVector3& pos)
{
	position = pos;
}

void gBody::setOrientation(real w, real x, real y, real z)
{
	orientation.w = w;
	orientation.x = x;
	orientation.y = y;
	orientation.z = z;
}

void gBody::setOrientation(const gQuaternion& orient)
{
	orientation = orient;
}

void gBody::setLinearVelocity(real lin_vel_x, real lin_vel_y, real lin_vel_z)
{
	linear_velocity.x = lin_vel_x;
	linear_velocity.y = lin_vel_y;
	linear_velocity.z = lin_vel_z;
}

void gBody::setLinearVelocity(const gVector3& lin_vel) 
{
	linear_velocity = lin_vel;
}

void gBody::setAngularVelocity(real ang_vel_x, real ang_vel_y, real ang_vel_z)
{
	angular_velocity.x = ang_vel_x;
	angular_velocity.y = ang_vel_y;
	angular_velocity.z = ang_vel_z;
}

void gBody::setAngularVelocity(const gVector3& ang_vel)
{
	angular_velocity = ang_vel;
}

void gBody::setLinearAcceleration(real lin_acc_x, real lin_acc_y, real lin_acc_z)
{
	linear_acceleration.x = lin_acc_x;
	linear_acceleration.y = lin_acc_y;
	linear_acceleration.z = lin_acc_z;
}

void gBody::setLinearAcceleration(const gVector3& lin_acc)
{
	linear_acceleration = lin_acc;
}

void gBody::setAngularAcceleration(real ang_acc_x, real ang_acc_y, real ang_acc_z)
{
	angular_acceleration.x = ang_acc_x;
	angular_acceleration.y = ang_acc_y;
	angular_acceleration.z = ang_acc_z;
}

void gBody::setAngularAcceleration(const gVector3& ang_acc)
{
	angular_acceleration = ang_acc;
}

real gBody::getMass() const
{
	return mass;
}

gVector3 gBody::getCenterOfMass() const
{
	return mass_center;
}

gMatrix3 gBody::getInertiaTensor() const
{
	return inertia_tensor;
}

void gBody::setMass(real m)
{
	mass = m;

	if (has_finite_mass) { inverse_mass = 1 / m; }
	else { inverse_mass = 0;  }
}


void gBody::setCenterOfMass(real x, real y, real z)
{
	mass_center.x = x;
	mass_center.y = y;
	mass_center.z = z;
}

void gBody::setInertiaTensorCoeffs(real c1, real c2, real c3)
{
	gBody::inertia_tensor.setDiagonal(c1, c2, c3);
	gBody::inverse_inertia_tensor = inertia_tensor.inverse();
}

void gBody::addForce(const gVector3& force)
{
	total_force += force;
}

void gBody::addForceAtPoint(const gVector3& force, const gVector3& point)
{
	total_force += force;

	gVector3 pt = convertWorldToLocal(point);
	total_torque += pt % force;
}

void gBody::addTorque(const gVector3& torque)
{
	total_torque += torque;
}

void gBody::clearTotals()
{
	total_force.x = total_force.y = total_force.z = 0;
	total_torque.x = total_torque.y = total_torque.z = 0;
}