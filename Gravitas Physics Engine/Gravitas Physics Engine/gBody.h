#ifndef GRAVITAS_GBody_H
#define GRAVITAS_GBody_H

#include "gMath.h"

namespace Gravitas
{
	class gBody
	{
		struct
		{
			/** Holds the body's position. */
			gVector3 position;

			/** Holds the body's orientation. */
			gQuaternion orientation;

			/** Holds the body's linear velocity. */
			gVector3 linear_velocity;

			/** Holds the angular velocity of the body. */
			gVector3 angular_velocity;

			/** Holds the linear acceleration of the body. */
			gVector3 linear_acceleration;

			/** Hold the angular acceleration of the body. */
			gVector3 angular_acceleration;

			/** Holds the matrix that holds the position and orientaion of the body in world coordinates. */
			gMatrix4 transform_matrix;
		};

		/** Holds whether or not the body has infinite mass. */
		bool has_finite_mass;

		/** Holds the mass of the body. */
		real mass;

		/** Holds the inverser mass of the body. */
		real inverse_mass;

		/** Holds the body's center of mass relative to origin. */
		gVector3 mass_center;

		/** Holds the body's inertia moment tensor. */
		gMatrix3 inertia_tensor;

		/** Holds the inverse of the inertia tensor. */
		gMatrix3 inverse_inertia_tensor;

		/** Holds the accumulated forces on the body. */
		gVector3 total_force;

		/** Holds the accumulated torques on the body. */
		gVector3 total_torque;

	public:
		/** Constructor */
		gBody();

		/** 
		* Calculates the change in kinetics based on a duration of time.
		*
		* @param duration: The amount of time passed in the frame. 
		*/
		void integrate(real duration);

		/** 
		* Returns a copy of the position.
		*/
		gVector3 getPosition() const;

		/** 
		* Fills a variable with the position of the body.
		*
		* @param vec: The vector to be filled.
		*/
		void getPosition(gVector3& vec);

		/** 
		* Returns a copy of the orientation.
		*/
		gQuaternion getOrientation() const;

		/**
		* Fills a variable with the orientation of the body.
		*
		* @param quat: The 3x3 matrix to be filled.
		*/
		void getOrientation(gQuaternion& quat);

		/** 
		* Returns a copy of the transformation matrix. 
		*/
		gMatrix4 getTransformMatrix() const;

		/** 
		* Fills a matrix with the data from the transformation matrix. 
		*
		* @param mat: The 4x4 matrix to be filled. 
		*/
		void getTransformMatrix(gMatrix4& mat);

		/**
		* Fills the given matrix with the transposed data from the transformation matrix.
		*
		* @param t_matrix: The 4x4 matrix to be filled.
		*/
		void getGLTransformMatrix(real matrix[16]);

		/** 
		* Returns a copy the linear velocity. 
		*/
		gVector3 getLinearVelocity() const;

		/**
		* Fills a variable with the linear velocity of the body.
		*
		* @param vec: The vector to be filled.
		*/
		void getLinearVelocity(gVector3& vec);

		/** 
		* Returns a copy of the angular velocity. 
		*/
		gVector3 getAngularVelocity() const;

		/**
		* Fills a variable with the angular velocity of the body.
		*
		* @param vec: The vector to be filled.
		*/
		void getAngularVelocity(gVector3& vec);

		/** Returns the linear acceleration. */
		gVector3 getLinearAcceleration() const;

		/**
		* Fills a variable with the linear acceleration of the body.
		*
		* @param vec: The vector to be filled.
		*/
		void getLinearAcceleration(gVector3& vec);

		/** Returns the angular acceleration. */
		gVector3 getAngularAcceleration() const;

		/**
		* Fills a variable with the angular acceleration of the body.
		*
		* @param vec: The vector to be filled.
		*/
		void getAngularAcceleration(gVector3& vec);

		/**
		* Sets the position of the body.
		*
		* @param pos_x: Input the x component of the desired position.
		* @param pos_y: Input the y component of the desired position.
		* @param pos_z: Input the z component of the desired position.
		*/
		void setPosition(real pos_x, real pos_y, real pos_z);

		/**
		* Sets the position of the body.
		*
		* @param pos: Input the position vector of the desired position.
		*/
		void setPosition(const gVector3 &pos);

		/**
		* Sets the orientation the body.
		*
		* @param s: Input the real number of the desired orientation.
		* @param x: Input the imaginery x component of the desired orientation.
		* @param y: Input the imaginery y component of the desired orientation.
		* @param z: Input the imaginery z component of the desired orientation.
		*/
		void setOrientation(real w, real x, real y, real z);

		/**
		* Sets the orientation the body.
		* 
		* @param orient: Input the quaternion of the desired quaternion.
		*/
		void setOrientation(const gQuaternion &orient);

		/**
		* Converts a point from local coordinates to world coordinates.
		*
		* @param point: Input the desired point to be converted from local to world coodinates.
		*/
		gVector3 convertLocalToWorld(const gVector3& point);

		/**
		* Converts a point from local coordinates to world coordinates.
		*
		* @param point: Input the desired point to be converted from local to world coodinates.
		*/
		gVector3 convertWorldToLocal(const gVector3& point);

		/**
		* Sets the linear velocity of the body.
		*
		* @param lin_vel_x: Input the x component of the desired linear velocity.
		* @param lin_vel_y: Input the y component of the desired linear velocity.
		* @param lin_vel_z: Input the z component of the desired linear velocity.
		*/
		void setLinearVelocity(real lin_vel_x, real lin_vel_y, real lin_vel_z);

		/**
		* Sets the linear velocity of the body.
		*
		* @param lin_vel: Input the vector of the desired linear velocity.
		*/
		void setLinearVelocity(const gVector3 &lin_vel);

		/**
		* Sets the angular velocity of the body.
		*
		* @param ang_vel_x: Input the x component of the desired angular velocity.
		* @param ang_vel_y: Input the y component of the desired angular velocity.
		* @param ang_vel_z: Input the z component of the desired angular velocity.
		*/
		void setAngularVelocity(real ang_vel_x, real ang_vel_y, real ang_vel_z);

		/**
		* Sets the angular velocity of the body.
		*
		* @param ang_vel: Input the vector of the desired angular velocity.
		*/
		void setAngularVelocity(const gVector3 &ang_vel);

		/**
		* Sets the linear acceleration body.
		*
		* @param lin_acc_x: Input the x component of the desired linear acceleration.
		* @param lin_acc_y: Input the y component of the desired linear acceleration.
		* @param lin_acc_z: Input the z component of the desired linear acceleration.
		*/
		void setLinearAcceleration(real lin_acc_x, real lin_acc_y, real lin_acc_z);

		/**
		* Sets the linear acceleration of the body.
		* @param lin_acc: Input the vector of the desired linear acceleration.
		*/
		void setLinearAcceleration(const gVector3 &lin_acc);

		/**
		* Sets the angular acceleration of the body.
		*
		* @param ang_vel_x: Input the x component of the desired angular acceleration.
		* @param ang_vel_y: Input the y component of the desired angular acceleration.
		* @param ang_vel_z: Input the z component of the desired angular acceleration.
		*/
		void setAngularAcceleration(real ang_vel_x, real ang_vel_y, real ang_vel_z);

		/**
		* Sets the angular acceleration of the body.
		*
		* @param ang_vel: Input the vector of the desired angular acceleration.
		*/
		void setAngularAcceleration(const gVector3 &ang_vel);

		/**
		* Returns the mass of the body. 
		*/
		real getMass() const;

		/**
		* Returns the inverse mass of the body.
		*/
		real getInverseMass() const;

		/** 
		* Returns the body's center of mass relative to it's origin. 
		*/
		gVector3 getCenterOfMass() const;

		/** 
		* Returns the inertia moment tensor. 
		*/
		gMatrix3 getInertiaTensor() const;

		/**
		* Sets the mass of the body.
		*
		* @param m: Input the desired mass.
		*/
		void setMass(real m);

		/**
		* Set's the body's center of mass RELATIVE TO IT'S ORIGIN.
		*
		* @param x: Input the x component of the desired center of mass.
		* @param y: Input the y component of the desired center of mass.
		* @param z: Input the z component of the desired center of mass.
		*/
		void setCenterOfMass(real x, real y, real z);

		/**
		* Set's the inertia moment tensor of the body.
		* 
		* @param c1: Input the Ixx coefficient.
		* @param c2: Input eht Iyy coefficient.
		* @param c3: Input the Izz coefficient.
		*/
		void setInertiaTensorCoeffs(real c1, real c2, real c3);

		/**
		* Adds a force to the body at the center of mass.
		*
		* @param force: Input the force to be applied to the body.
		*/
		void addForce(const gVector3& force);

		/**
		* Adds a force to the body at specified point.
		*
		* @param force: Input the force to be applied to the body.
		* @param point: Input the point at which the force is applied IN WORLD COORDINATES.
		*/
		void addForceAtPoint(const gVector3& force, const gVector3& point);

		/**
		* Adds a torque to the body.
		*
		* @param torque: Input the torque to be applied to the body.
		*/
		void addTorque(const gVector3& torque);

		/** Clears the accumulated forces and torques acting on the body. */
		void clearTotals();
	};
}

#endif