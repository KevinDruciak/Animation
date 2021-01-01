#include <cmath>
#include <Util/exceptions.h>
#include "scene.h"
#include "cylinder.h"

using namespace Ray;
using namespace Util;

//////////////
// Cylinder //
//////////////

void Cylinder::init( const LocalSceneData &data )
{
	// Set the material pointer
	if( _materialIndex<0 ) THROW( "negative material index: %d" , _materialIndex );
	else if( _materialIndex>=data.materials.size() ) THROW( "material index out of bounds: %d <= %d" , _materialIndex , (int)data.materials.size() );
	else _material = &data.materials[ _materialIndex ];

	//////////////////////////////////
	// Do any necessary set-up here //
	//////////////////////////////////
	WARN_ONCE( "method undefined" );
}

void Cylinder::updateBoundingBox( void )
{
	///////////////////////////////
	// Set the _bBox object here //
	///////////////////////////////
	Point3D p(radius, height / 2, radius);
	_bBox = BoundingBox3D(center - p, center + p);
}

void Cylinder::initOpenGL( void )
{
	/////////////////////////////////////////
	// Do any necessary OpenGL set-up here //
	/////////////////////////////////////////
	WARN_ONCE( "method undefined" );

	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();
}

double Cylinder::intersectBase(Ray3D ray, Point3D c) const
{
	Point3D normal;
	if (c[0]<center[0] + radius && c[0]>center[0] - radius && c[2]<center[2] + radius && c[2]>center[2] - radius) {
		if (c[1] < center[1] + height + EPSILON && c[1]>center[1] + height - EPSILON) normal[0] = 0; normal[1] = 1; normal[2] = 0;
		if (c[1] < center[1] + EPSILON && c[1]>center[1] - EPSILON) normal[0] = 0; normal[1] = -1; normal[2] = 0;
	} else {
		normal = (c - Point3D(center[0], c[1], center[2])).unit();
	}
	double D = -(normal.dot(c - center));
	if (normal.dot(ray.direction) == 0) return INFINITY;
	double distance = -(normal.dot(ray.position) + D) / (normal.dot(ray.direction));
	if (distance < EPSILON) return INFINITY;
	Point3D p = ray(distance);

	if (p[0]*p[0] + p[2] *p[2] - radius * radius > EPSILON) return INFINITY;
	return distance;
}

double Cylinder::intersect( Ray3D ray , RayShapeIntersectionInfo& iInfo , BoundingBox1D range , std::function< bool (double) > validityLambda ) const
{
	RayTracingStats::IncrementRayPrimitiveIntersectionNum();

	/////////////////////////////////////////////////////////////
	// Compute the intersection of the shape with the ray here //
	/////////////////////////////////////////////////////////////
	Ray3D pt(ray.position - center, ray.direction);
	double a = pt.direction[0] * pt.direction[0] + pt.direction[2] * pt.direction[2];
	double b = pt.direction[0] * pt.position[0] + pt.direction[2] * pt.position[2];
	double c = pt.position[0] * pt.position[0] + pt.position[2] * pt.position[2] - radius * radius;

	double d1 = b * b - a * c;
	if (d1 < EPSILON) return INFINITY;
	double d2 = (-b - sqrt(d1)) / a;
	if (d2 < EPSILON) return INFINITY;

	double rayY = pt.position[1] + d2 * pt.direction[1];
	bool bottom = false;
	bool top = false;
	if (rayY < -EPSILON || rayY > height + EPSILON) {
		double base1 = intersectBase(pt, center + Point3D(0, height, 0));
		if (base1 != INFINITY) {
			d2 = base1;
			top = true;
		}
		double base2 = intersectBase(pt, center);
		if (top && d2 >= base2) {
			d2 = base2;
			bottom = true;
		}
	}
	Point3D intersection = ray(d2);
	iInfo.position = intersection;
	Point3D newcenter = center - center / 2;
	iInfo.normal = top ? Point3D(0, 1, 0)
		: bottom ? Point3D(0, -1, 0)
		: ((intersection - newcenter) - (Point3D(0, 1, 0).dot(intersection - newcenter)*Point3D(0, 1, 0))).unit();
	iInfo.material = this->_material;
	return d2;
}

bool Cylinder::isInside( Point3D p ) const
{
	////////////////////////////////////////////////////////
	// Determine if the point is inside the cylinder here //
	////////////////////////////////////////////////////////
	THROW( "method undefined" );
	return false;
}

void Cylinder::drawOpenGL(GLSLProgram *glslProgram) const
{
	//////////////////////////////
	// Do OpenGL rendering here //
	//////////////////////////////
	_material->drawOpenGL(glslProgram);

	GLUquadric *q = gluNewQuadric();

	glPushMatrix();

	glTranslatef(center[0], center[1] - height / 2, center[2]);
	glRotatef(90, -1, 0, 0);
	gluCylinder(q, radius, radius, height, Shape::OpenGLTessellationComplexity, Shape::OpenGLTessellationComplexity);

	glPushMatrix();
	glTranslatef(0, 0, height);
	gluDisk(q, 0, radius, Shape::OpenGLTessellationComplexity, Shape::OpenGLTessellationComplexity);
	glPopMatrix();

	glRotatef(180, 1, 0, 0); // Normals pointing out
	gluDisk(q, 0, radius, Shape::OpenGLTessellationComplexity, Shape::OpenGLTessellationComplexity);

	glPopMatrix();

	gluDeleteQuadric(q);

	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();

}