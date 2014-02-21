#include "gCollision.h"
#include <memory.h>

using namespace Gravitas;

void gCollisionPrimitive::calculateInternals(real duration)
{
	body->integrate(duration);
	transform_matrix = body->getTransformMatrix();
}

gCollisionPlane::gCollisionPlane() : direction()
{
}

void gCollisionPlane::setPlane(const gVector3& dir)
{
	gCollisionPlane::direction = dir;
}

unsigned gCollisionDetector::performSphereAndSphere(const gCollisionSphere& sphere_1, const gCollisionSphere& sphere_2, gCollisionData* c_data)
{
	real distance = (sphere_1.body->getPosition() - sphere_2.body->getPosition()).magnitude();
	
	if (distance >= sphere_2.radius + sphere_1.radius) { return 0; }
	else 
	{ 
		gContact* contact = new gContact();					//********* DELETE THIS UPON CONTACT COMPLETETION!************//

		contact->setBodyData(sphere_1.body, sphere_2.body, 0.3, 0.8);

		contact->relative_position = sphere_1.body->getPosition() - sphere_2.body->getPosition();

		contact->penetration = sphere_2.radius + sphere_1.radius - contact->relative_position.magnitude();

		contact->contact_point = sphere_1.body->getPosition() + contact->relative_position.normalized() * sphere_1.radius;

		contact->calculateContactAxes();

		c_data->contact_list.push_back(contact);
		
		return 1; 
	}
}

unsigned gCollisionDetector::performSphereAndBox(const gCollisionBox& box, const gCollisionSphere& sphere, gCollisionData* c_data)
{

}

unsigned gCollisionDetector::performSphereAndPlane(const gCollisionSphere& sphere, const gCollisionPlane& plane, gCollisionData* c_data)
{

}

unsigned gCollisionDetector::performBoxAndBox(const gCollisionBox& box_1, const gCollisionBox& box_2, gCollisionData* c_data)
{

}

unsigned gCollisionDetector::performBoxAndPlane(const gCollisionBox& box, const gCollisionPlane& plane, gCollisionData* c_data)
{

}