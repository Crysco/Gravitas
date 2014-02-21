#include "gMath.h"
#include "gBody.h"
#include <iostream>

int main()
{
	Gravitas::gBody* body = new Gravitas::gBody();
	body->setPosition(1, 1, 1);
	Gravitas::gVector3 pos;
	body->getPosition(pos);

	std::cout << pos.x;

	int x;
	std::cin >> x;
	return 0;
}