#include <cmath>
#include <Util/exceptions.h>
#include "camera.h"
#include "shape.h"

using namespace Ray;
using namespace Util;

////////////
// Camera //
////////////

Ray3D Camera::getRay( int i , int j , int width , int height, int where ) const
{
	/////////////////////////////////////////////////
	// Get the ray through the (i,j)-th pixel here //
	/////////////////////////////////////////////////
	double angle = tan(this->heightAngle / 2.0);
	double ar = static_cast<double>(width) / static_cast<double>(height);
	Point3D x = 2.0 * (this->right) * (i + 0.5) / width * ar * angle;
	Point3D y = 2.0 * (-this->up) * (j + 0.5) / height * angle;
	Point3D dir = (this->forward - angle * (-this->up + this->right * ar) + x + y).unit();
	return Ray3D(this->position, dir);
}

void Camera::drawOpenGL( void ) const
{
	//////////////////////////////
	// Do OpenGL rendering here //
	//////////////////////////////
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(position[0], position[1], position[2], position[0] + forward[0], position[1] + forward[1], position[2] + forward[2], up[0], up[1], up[2]);

	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();
}

void Camera::rotateUp( Point3D center , float angle )
{
	///////////////////////////////////////////////////
	// Rotate the camera about the up direction here //
	///////////////////////////////////////////////////
	position = position * cos(-angle) + (Point3D::CrossProduct(up, position))*sin(-angle) + up * (Point3D::Dot(up, position))*(1 - cos(-angle));
	forward = (forward * cos(-angle) + Point3D::CrossProduct(up, forward)*sin(-angle) + up * Point3D::Dot(up, forward)*(1 - cos(-angle))).unit();
	right = (Point3D::CrossProduct(forward, up)).unit();
}

void Camera::rotateRight( Point3D center , float angle )
{
	//////////////////////////////////////////////////////
	// Rotate the camera about the right direction here //
	//////////////////////////////////////////////////////
	position = position * cos(-angle) + (Point3D::CrossProduct(right, position))*sin(-angle) + right * (Point3D::Dot(right, position))*(1 - cos(-angle));
	forward = (forward * cos(-angle) + Point3D::CrossProduct(right, forward)*sin(-angle) + right * Point3D::Dot(right, forward)*(1 - cos(-angle))).unit();
	up = (up * cos(-angle) + Point3D::CrossProduct(right, up)*sin(-angle) + right * Point3D::Dot(right, up)*(1 - cos(-angle))).unit();
}

void Camera::moveForward( float dist )
{
	//////////////////////////////////
	// Move the camera forward here //
	//////////////////////////////////
	this->position += this->forward.unit() * dist;
}

void Camera::moveRight( float dist )
{
	///////////////////////////////////////
	// Move the camera to the right here //
	///////////////////////////////////////
	this->position += this->right.unit() * dist;
}

void Camera::moveUp( float dist )
{
	/////////////////////////////
	// Move the camera up here //
	/////////////////////////////
	this->position += this->up.unit() * dist;

}