#ifndef GRAVITAS_GCOLLISION_BODY_H
#define GRAVITAS_GCOLLISION_BODY_H

#include "gContact.h"
#include <vector>

namespace Gravitas
{
	// Forward declarations of primitive friends
	class gCollisionDetector;

	/**
	* Represents a primitive to detect collisions against.
	*/

	class gCollisionPrimitive
	{
		friend class gIntersectionTests;
		friend class gCollisionDetector;

	protected:
		/** Holds a pointer to the body that the collision shape attaches too. */
		gBody* body;

		/** Holds the transformation matrix of the collision shape. */
		gMatrix4 transform_matrix;

	public:
		/**
		* Fills a 4x4 matrix with the transformation matrix of the attach body.
		*
		* @param mat: The 4x4 matrixt to be filled. 
		*/
		virtual void calculateInternals(real duration) = 0;
	};

	/**
	* Represents a rigid body that can be treated as a sphere
	* for collision detection.
	*/
	class gCollisionSphere : public gCollisionPrimitive
	{
	public:
		/** Holds the radius of the collision sphere. */
		real radius;
	};

	/**
	* Represents a rigid body that can be treated as an aligned bounding box for collision detection.
	*/
	class gCollisionBox : public gCollisionPrimitive
	{
	public:
		gVector3 half_sizes;
	};

	/**
	* The plane is not a primitive: it doesn't represent another rigid body.
	* It is used for contacts with the immovable world geometry.
	*/
	class gCollisionPlane
	{
		/** Holds the normal vector of the collision plane. */
		gVector3 direction;

	public:
		/** Default Constructor. */
		gCollisionPlane();

		/**
		* Sets the direction of the normal vector of the collision plane.
		*
		* @param dir: Input the desired direction of the normal vector.
		*/
		void setPlane(const gVector3& dir);
	};

	class gCollisionData
	{
	public:
		std::vector<gContact*> contact_list; 

	public:
		gCollisionData();
	};

	/** 
	* A wrapper class that holds the fine grained collision detection routines.
	* 
	* Each of the functions has the same format: it takes the details
	* of two objects, and a pointer to a contact array to fill. It
	* returns the number of contacts it wrote into the array.
	*/
	class gCollisionDetector
	{
		static unsigned performSphereAndSphere(const gCollisionSphere& sphere_1, const gCollisionSphere& sphere_2, gCollisionData* c_data);
		static unsigned performSphereAndBox(const gCollisionBox& box, const gCollisionSphere& sphere, gCollisionData* c_data);
		static unsigned performSphereAndPlane(const gCollisionSphere& sphere, const gCollisionPlane& plane, gCollisionData* c_data);
		static unsigned performBoxAndBox(const gCollisionBox& box_1, const gCollisionBox& box_2, gCollisionData* c_data);
		static unsigned performBoxAndPlane(const gCollisionBox& box, const gCollisionPlane& plane, gCollisionData* c_data);
	};
}

#endif
