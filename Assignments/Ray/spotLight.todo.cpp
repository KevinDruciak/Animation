#include <cmath>
#include <Util/exceptions.h>
#include "scene.h"
#include "spotLight.h"

using namespace Ray;
using namespace Util;

///////////////
// SpotLight //
///////////////
Point3D SpotLight::getAmbient( Ray3D ray , const RayShapeIntersectionInfo& iInfo ) const
{
	////////////////////////////////////////////////////
	// Get the ambient contribution of the light here //
	////////////////////////////////////////////////////
	double I = ((this->_location - iInfo.position).unit()).dot(iInfo.normal);
	double distance = (this->_location - iInfo.position).length();
	double angle = this->_direction.unit().dot((iInfo.position - this->_location).unit());
	double intensity = pow(angle, this->_dropOffRate) / (this->_constAtten + this->_linearAtten * distance + this->_quadAtten * pow(distance, 2));
	return iInfo.material->ambient * intensity * (I >= 0 ? I : 0);
}

Point3D SpotLight::getDiffuse( Ray3D ray , const RayShapeIntersectionInfo& iInfo ) const
{
	////////////////////////////////////////////////////
	// Get the diffuse contribution of the light here //
	////////////////////////////////////////////////////
	double I = ((this->_location - iInfo.position).unit()).dot(iInfo.normal);
	double distance = (this->_location - iInfo.position).length();
	double angle = this->_direction.unit().dot((iInfo.position - this->_location).unit());
	if (angle <= cos(this->_cutOffAngle)) return Point3D();
	double intensity = pow(angle, this->_dropOffRate) / (this->_constAtten + this->_linearAtten * distance + this->_quadAtten * pow(distance, 2));
	return iInfo.material->diffuse * this->_diffuse * intensity * (I >= 0 ? I : 0);
}

Point3D SpotLight::getSpecular( Ray3D ray , const RayShapeIntersectionInfo& iInfo ) const
{
	/////////////////////////////////////////////////////
	// Get the specular contribution of the light here //
	/////////////////////////////////////////////////////
	Point3D L = (this->_location - iInfo.position).unit();
	Point3D V = (ray.position - iInfo.position).unit();
	Point3D R = iInfo.normal * iInfo.normal.dot(V) + -(V - iInfo.normal * iInfo.normal.dot(V));
	double theta = L.dot(R);
	if (theta < 0) return Point3D();
	double I = pow(theta, iInfo.material->specularFallOff);
	double angle = this->_direction.unit().dot((iInfo.position - this->_location).unit());
	if (angle <= cos(this->_cutOffAngle)) return Point3D();
	double distance = (this->_location - iInfo.position).length();
	double intensity = pow(angle, this->_dropOffRate) / (this->_constAtten + this->_linearAtten * distance + this->_quadAtten * pow(distance, 2));
	return iInfo.material->specular * this->_specular * intensity * I;
}

bool SpotLight::isInShadow( const RayShapeIntersectionInfo& iInfo , const Shape* shape ) const
{
	//////////////////////////////////////////////
	// Determine if the light is in shadow here //
	//////////////////////////////////////////////
	Point3D L = (this->_location - iInfo.position).unit();
	RayShapeIntersectionInfo temp = iInfo;
	return shape->intersect(Ray3D(iInfo.position + L * EPSILON, L), temp, BoundingBox1D(), std::function<bool(double)>()) == INFINITY ? true : false;
}

Point3D SpotLight::transparency( const RayShapeIntersectionInfo &iInfo , const Shape &shape , Point3D cLimit ) const
{
	//////////////////////////////////////////////////////////
	// Compute the transparency along the path to the light //
	//////////////////////////////////////////////////////////
	Point3D one; one[0] = 1; one[1] = 1; one[2] = 1;
	//Point3D translate; translate[0] = x / 10.0; translate[1] = 0; translate[2] = z / 10.0;
	RayShapeIntersectionInfo temp;
	Point3D L = (this->_location - iInfo.position);
	if (shape.intersect(Ray3D(iInfo.position + L.unit() * EPSILON, L.unit()), temp, BoundingBox1D(), std::function<bool(double)>()) == INFINITY) return one;
	return temp.normal.dot(-L) <= 0 ? one : temp.material->transparent * transparency(temp, shape, cLimit);
}

void SpotLight::drawOpenGL( int index , GLSLProgram * glslProgram ) const
{
	//////////////////////////////
	// Do OpenGL rendering here //
	//////////////////////////////
	glShadeModel(GL_SMOOTH);

	GLfloat light_diffuse[] = { _diffuse[0], _diffuse[1], _diffuse[2], 0 };
	GLfloat light_specular[] = { _specular[0], _specular[1], _specular[2], 0 };
	GLfloat position[] = { _location[0], _location[1], _location[2], 1.0 };
	GLfloat direction[] = { _direction[0], _direction[1], _direction[2] };
	GLfloat dropOff[] = { this->_dropOffRate };
	GLfloat light_ambient[] = { _ambient[0], _ambient[1], _ambient[2], 1.0f };
	GLfloat cutOff[] = { _cutOffAngle * 180 / Pi };
	GLfloat constAttenuation[] = { _constAtten };
	GLfloat linearAttenuation[] = { _linearAtten };
	GLfloat quadAttenuation[] = { _quadAtten };
	glLightfv(GL_LIGHT0 + index, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0 + index, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0 + index, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0 + index, GL_POSITION, position);
	glLightfv(GL_LIGHT0 + index, GL_SPOT_EXPONENT, dropOff);
	glLightfv(GL_LIGHT0 + index, GL_SPOT_CUTOFF, cutOff);
	glLightfv(GL_LIGHT0 + index, GL_SPOT_DIRECTION, direction);
	glLightfv(GL_LIGHT0 + index, GL_CONSTANT_ATTENUATION, constAttenuation);
	glLightfv(GL_LIGHT0 + index, GL_QUADRATIC_ATTENUATION, quadAttenuation);
	glLightfv(GL_LIGHT0 + index, GL_LINEAR_ATTENUATION, linearAttenuation);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0 + index);
	glEnable(GL_DEPTH_TEST);
}

void SpotLight::drawOpenGLoff(int index, GLSLProgram * glslProgram) const
{
	//////////////////////////////
	// Do OpenGL rendering here //
	//////////////////////////////
	glShadeModel(GL_SMOOTH);

	GLfloat light_diffuse[] = { 0, 0, 0, 0 };
	GLfloat light_specular[] = { 0, 0, 0, 0 };
	GLfloat position[] = { _location[0], _location[1], _location[2], 1.0 };
	GLfloat direction[] = { _direction[0], _direction[1], _direction[2] };
	GLfloat dropOff[] = { this->_dropOffRate };
	GLfloat light_ambient[] = { 0, 0, 0, 1.0f };
	GLfloat cutOff[] = { _cutOffAngle * 180 / Pi };
	GLfloat constAttenuation[] = { _constAtten };
	GLfloat linearAttenuation[] = { _linearAtten };
	GLfloat quadAttenuation[] = { _quadAtten };
	glLightfv(GL_LIGHT0 + index, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0 + index, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0 + index, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0 + index, GL_POSITION, position);
	glLightfv(GL_LIGHT0 + index, GL_SPOT_EXPONENT, dropOff);
	glLightfv(GL_LIGHT0 + index, GL_SPOT_CUTOFF, cutOff);
	glLightfv(GL_LIGHT0 + index, GL_SPOT_DIRECTION, direction);
	glLightfv(GL_LIGHT0 + index, GL_CONSTANT_ATTENUATION, constAttenuation);
	glLightfv(GL_LIGHT0 + index, GL_QUADRATIC_ATTENUATION, quadAttenuation);
	glLightfv(GL_LIGHT0 + index, GL_LINEAR_ATTENUATION, linearAttenuation);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0 + index);
	glEnable(GL_DEPTH_TEST);
}