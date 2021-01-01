#include <cmath>
#include <Util/exceptions.h>
#include "pointLight.h"
#include "scene.h"

using namespace Ray;
using namespace Util;

////////////////
// PointLight //
////////////////

Point3D PointLight::getAmbient(Ray3D ray, const RayShapeIntersectionInfo& iInfo) const
{
	////////////////////////////////////////////////////
	// Get the ambient contribution of the light here //
	////////////////////////////////////////////////////
	const double distance = (iInfo.position - ray.position).length();
	return Point3D(this->_ambient * iInfo.material->ambient / (this->_constAtten + this->_linearAtten * distance + this->_quadAtten * pow(distance, 2)));
}

Point3D PointLight::getDiffuse(Ray3D ray, const RayShapeIntersectionInfo& iInfo) const
{
	////////////////////////////////////////////////////
	// Get the diffuse contribution of the light here //
	////////////////////////////////////////////////////
	const double distance = (this->_location - iInfo.position).length();
	Point3D color = this->_diffuse / (this->_constAtten + this->_linearAtten * distance + this->_quadAtten * pow(distance, 2));
	return color * iInfo.normal.dot((this->_location - iInfo.position).unit()) * iInfo.material->diffuse;
}

Point3D PointLight::getSpecular(Ray3D ray, const RayShapeIntersectionInfo& iInfo) const
{
	/////////////////////////////////////////////////////
	// Get the specular contribution of the light here //
	/////////////////////////////////////////////////////
	Point3D L = (this->_location - iInfo.position).unit();
	Point3D V = (ray.position - iInfo.position).unit();
	Point3D R = iInfo.normal * iInfo.normal.dot(V) + -(V - iInfo.normal * iInfo.normal.dot(V));
	double theta = L.dot(R.unit());
	if (theta < 0) return Point3D();
	double distance = (this->_location - iInfo.position).length();
	double I = pow(theta, iInfo.material->specularFallOff) / (this->_constAtten + this->_linearAtten * distance + this->_quadAtten * pow(distance, 2));
	return iInfo.material->specular * I * this->_specular;
}

bool PointLight::isInShadow(const RayShapeIntersectionInfo& iInfo, const Shape* shape) const
{
	//////////////////////////////////////////////
	// Determine if the light is in shadow here //
	//////////////////////////////////////////////
	Point3D L = (this->_location - iInfo.position).unit();
	RayShapeIntersectionInfo temp = iInfo; //apple
	return shape->intersect(Ray3D(iInfo.position + L * EPSILON, L), temp, BoundingBox1D(), std::function<bool(double)>()) == INFINITY ? true : false;
}

Point3D PointLight::transparency(const RayShapeIntersectionInfo &iInfo, const Shape &shape, Point3D cLimit) const
{
	//////////////////////////////////////////////////////////
	// Compute the transparency along the path to the light //
	//////////////////////////////////////////////////////////
	Point3D one; one[0] = 1; one[1] = 1; one[2] = 1;
	RayShapeIntersectionInfo temp;
	Point3D L = (this->_location - iInfo.position);
	if (shape.intersect(Ray3D(iInfo.position + L.unit() * EPSILON, L.unit()), temp, BoundingBox1D(), std::function<bool(double)>()) == INFINITY) return one;
	return temp.normal.dot(-L) <= 0 ? one : temp.material->transparent * transparency(temp, shape, cLimit);
}

void PointLight::drawOpenGL(int index, GLSLProgram * glslProgram) const
{
	//////////////////////////////
	// Do OpenGL rendering here //
	//////////////////////////////
	glShadeModel(GL_SMOOTH);

	GLfloat light_diffuse[] = { _diffuse[0], _diffuse[1], _diffuse[2], 0 };
	GLfloat light_specular[] = { _specular[0], _specular[1], _specular[2], 0 };
	GLfloat light_ambient[] = { 0.f, 0.f, 0.f, 1.0f };
	GLfloat light_position[] = { _location[0], _location[1], _location[2], 1 };
	GLfloat light_constant_atten = _constAtten;
	GLfloat light_linear_atten = _linearAtten;
	GLfloat light_quad_atten = _quadAtten;

	glLightfv(GL_LIGHT0 + index, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0 + index, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0 + index, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0 + index, GL_POSITION, light_position);
	glLightf(GL_LIGHT0 + index, GL_CONSTANT_ATTENUATION, light_constant_atten);
	glLightf(GL_LIGHT0 + index, GL_LINEAR_ATTENUATION, light_linear_atten);
	glLightf(GL_LIGHT0 + index, GL_QUADRATIC_ATTENUATION, light_quad_atten);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0 + index);
	glEnable(GL_DEPTH_TEST);
}

void PointLight::drawOpenGLoff(int index, GLSLProgram * glslProgram) const
{
	//////////////////////////////
	// Do OpenGL rendering here //
	//////////////////////////////
	glShadeModel(GL_SMOOTH);

	GLfloat light_diffuse[] = { 0, 0, 0, 0 };
	GLfloat light_specular[] = { 0, 0, 0, 0 };
	GLfloat light_ambient[] = { 0, 0, 0, 1.0 };
	GLfloat light_position[] = { _location[0], _location[1], _location[2], 1 };
	GLfloat light_constant_atten = _constAtten;
	GLfloat light_linear_atten = _linearAtten;
	GLfloat light_quad_atten = _quadAtten;

	glLightfv(GL_LIGHT0 + index, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0 + index, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0 + index, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0 + index, GL_POSITION, light_position);
	glLightf(GL_LIGHT0 + index, GL_CONSTANT_ATTENUATION, light_constant_atten);
	glLightf(GL_LIGHT0 + index, GL_LINEAR_ATTENUATION, light_linear_atten);
	glLightf(GL_LIGHT0 + index, GL_QUADRATIC_ATTENUATION, light_quad_atten);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0 + index);
	glEnable(GL_DEPTH_TEST);
}