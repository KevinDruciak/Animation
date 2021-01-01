#include <cmath>
#include <Util/exceptions.h>
#include "directionalLight.h"
#include "scene.h"

using namespace Ray;
using namespace Util;

//////////////////////
// DirectionalLight //
//////////////////////

Point3D DirectionalLight::getAmbient( Ray3D ray , const RayShapeIntersectionInfo& iInfo ) const
{
	////////////////////////////////////////////////////
	// Get the ambient contribution of the light here //
	////////////////////////////////////////////////////
	return iInfo.material->ambient * this->_ambient;
}

Point3D DirectionalLight::getDiffuse( Ray3D ray , const RayShapeIntersectionInfo& iInfo ) const
{
	////////////////////////////////////////////////////
	// Get the diffuse contribution of the light here //
	////////////////////////////////////////////////////
	return iInfo.material->diffuse * (iInfo.normal).dot(-this->_direction) * this->_diffuse;

}

Point3D DirectionalLight::getSpecular( Ray3D ray , const RayShapeIntersectionInfo& iInfo ) const
{
	/////////////////////////////////////////////////////
	// Get the specular contribution of the light here //
	/////////////////////////////////////////////////////
	if (iInfo.normal.dot(-this->_direction) < 0 || (iInfo.position - ray.position).dot(iInfo.normal) >= 0) return Point3D();
	Point3D R = (this->_direction - iInfo.normal * 2.0 * (this->_direction.dot(iInfo.normal))).unit();
	Point3D V = (ray.position - iInfo.position).unit();
	return iInfo.material->specular * pow(V.dot(R), iInfo.material->specularFallOff) * this->_specular;
}

bool DirectionalLight::isInShadow( const RayShapeIntersectionInfo& iInfo , const Shape* shape ) const
{
	//////////////////////////////////////////////
	// Determine if the light is in shadow here //
	//////////////////////////////////////////////
	RayShapeIntersectionInfo temp;
	return shape->intersect(Ray3D(iInfo.position + -(this->_direction) * EPSILON, -(this->_direction))
		, temp, BoundingBox1D(), std::function<bool(double)>()) == INFINITY ? true : false;
}

Point3D DirectionalLight::transparency( const RayShapeIntersectionInfo &iInfo , const Shape &shape , Point3D cLimit ) const
{
	//////////////////////////////////////////////////////////
	// Compute the transparency along the path to the light //
	//////////////////////////////////////////////////////////
	Point3D one; one[0] = 1; one[1] = 1; one[2] = 1;
	RayShapeIntersectionInfo info;

	if (shape.intersect(Ray3D(iInfo.position + -this->_direction.unit() * EPSILON, -this->_direction.unit()), info, BoundingBox1D(), std::function<bool(double)>()) != INFINITY) return one;
	return info.normal.dot(this->_direction) <= 0 ? one : info.material->transparent * transparency(info, shape, cLimit);
}

void DirectionalLight::drawOpenGL( int index , GLSLProgram * glslProgram ) const
{
	//////////////////////////////
	// Do OpenGL rendering here //
	//////////////////////////////

	glShadeModel(GL_SMOOTH);
	GLfloat light_diffuse[] = { _diffuse[0], _diffuse[1], _diffuse[2], 0 };
	GLfloat light_specular[] = { _specular[0], _specular[1], _specular[2], 0 };
	GLfloat light_ambient[] = { _ambient[0], _ambient[1], _ambient[2], 1.0f };
	GLfloat light_position[] = { -_direction[0], -_direction[1], -_direction[2], 0 };
	glLightfv(GL_LIGHT0 + index, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0 + index, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0 + index, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0 + index, GL_POSITION, light_position);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0 + index);
	glEnable(GL_DEPTH_TEST);
}

void DirectionalLight::drawOpenGLoff(int index, GLSLProgram * glslProgram) const
{
	glShadeModel(GL_SMOOTH);
	GLfloat light_diffuse[] = { 0, 0, 0, 0 };
	GLfloat light_specular[] = { 0, 0, 0, 0 };
	GLfloat light_ambient[] = { 0.f, 0.f, 0.f, 1.0f };
	GLfloat light_position[] = { -_direction[0], -_direction[1], -_direction[2], 0 };

	glLightfv(GL_LIGHT0 + index, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0 + index, GL_SPECULAR, light_specular);
	glLightfv(GL_LIGHT0 + index, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT0 + index, GL_POSITION, light_position);

	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0 + index);
	glEnable(GL_DEPTH_TEST);
}

