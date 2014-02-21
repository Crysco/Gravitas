#ifndef GRAVITAS_GCONTACT_H
#define GRAVITAS_GCONTACT_H

#include "gBody.h"
#include <vector>

namespace Gravitas
{
	class gContactResolver;

	class gContact
	{
		friend gContactResolver;
	
	public:
		/**
		* An array that holds the two bodies involved in contact.
		*/
		gBody* body[2];

		/**
		* Holds the depth of penetration between the two bodies.
		*/
		real penetration;

		/**
		* Holds the coefficient of restitution between the two bodies.
		*/
		real restitution;

		/**
		* Holds the coefficient of friction between the two bodies.
		*/
		real friction;

		/** 
		* Holds the position of body the second body relative to the first. 
		*/
		gVector3 relative_position;

		/**
		* Holds the point at which the two bodies come in contact relative to the first body.
		*/
		gVector3 contact_point;

		/**
		* Holds the contact normal between the two bodies.
		*/
		gVector3 contact_normal;

	protected:
		gVector3 contact_tangents[2];

		/** Holds the closing velocity. */
		gVector3 closing_velocity;

	public:
		/** Default constructor. */
		gContact();

		/** Destructor. */
		~gContact();

		/** 
		* Set the collision data between the two bodies.
		*/
		void setBodyData(gBody* body_1, gBody* body_2, real friction, real restitution);

		/**
		* Calculates the contact normal on the orthographic axes of the contact point. 
		*/
		void calculateContactAxes();

		void calculateVelocityChanges(real duration);

		real& calculateImpulseMagnitude(gVector3& vel_1, gVector3& vel_2);

		void resolvePenetration();

	};
}

#endif