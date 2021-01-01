#include <cmath>
#include <Util/exceptions.h>
#include "scene.h"
#include "sphere.h"

using namespace Ray;
using namespace Util;

////////////
// Sphere //
////////////

void Sphere::init( const LocalSceneData &data )
{
	// Set the material pointer
	if( _materialIndex<0 ) THROW( "negative material index: %d" , _materialIndex );
	else if( _materialIndex>=data.materials.size() ) THROW( "material index out of bounds: %d <= %d" , _materialIndex , (int)data.materials.size() );
	else _material = &data.materials[ _materialIndex ];

	///////////////////////////////////
	// Do any additional set-up here //
	///////////////////////////////////
	WARN_ONCE( "method undefined" );
}
void Sphere::updateBoundingBox( void )
{
	///////////////////////////////
	// Set the _bBox object here //
	///////////////////////////////
	Point3D p(radius, radius, radius);
	_bBox = BoundingBox3D(center - p, center + p);
}
void Sphere::initOpenGL( void )
{
	///////////////////////////
	// Do OpenGL set-up here //
	///////////////////////////
	WARN_ONCE( "method undefined" );

	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();
}

double Sphere::intersect( Ray3D ray , RayShapeIntersectionInfo &iInfo , BoundingBox1D range , std::function< bool (double) > validityLambda ) const
{
	RayTracingStats::IncrementRayPrimitiveIntersectionNum();

	//////////////////////////////////////////////////////////////
	// Compute the intersection of the sphere with the ray here //
	//////////////////////////////////////////////////////////////
	Polynomial1D<2> equation;
	equation.coefficient(0) = (ray.position - this->center).squareNorm() - pow(this->radius, 2);
	equation.coefficient(1) = 2.0 * ray.direction.dot(ray.position - this->center);
	equation.coefficient(2) = 1;

	auto *roots = new double[2];
	const auto numRoots = equation.roots(roots);

	if (numRoots == 0) return INFINITY;

	const auto distance = (roots[0] >= 0 && roots[1] < 0) || (roots[0] < 0 && roots[1] >= 0)
		? std::max(roots[0], roots[1]) : (roots[0] >= 0 && roots[1] >= 0)
		? std::min(roots[0], roots[1]) : INFINITY;

	if (distance == INFINITY) return INFINITY;

	Point3D intersection = ray(distance);
	iInfo.position = intersection;
	iInfo.normal = (iInfo.position - this->center).unit();
	iInfo.material = this->_material;

	float theta = atan2(-(intersection[2] - this->center[2]), intersection[0] - this->center[0]);
	float u = (theta + Pi) / (2.0 * Pi);
	float gamma = acos(-(intersection[1] - this->center[1]) / this->radius);
	float v = gamma / Pi;

	iInfo.texture[0] = u;
	iInfo.texture[1] = v;

	return distance;
}

bool Sphere::isInside( Point3D p ) const
{
	//////////////////////////////////////////////////////
	// Determine if the point is inside the sphere here //
	//////////////////////////////////////////////////////
	THROW( "method undefined" );
	return false;
}

void Sphere::drawOpenGL( GLSLProgram * glslProgram ) const
{
	//////////////////////////////
	// Do OpenGL rendering here //
	//////////////////////////////
	_material->drawOpenGL(glslProgram);
	glPushMatrix();
	glTranslatef(center[0], center[1], center[2]);
	GLUquadric *q = gluNewQuadric();
	gluSphere(q, radius, 2 * Shape::OpenGLTessellationComplexity, Shape::OpenGLTessellationComplexity);
	gluDeleteQuadric(q);
	glPopMatrix();

	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();

	/*glDisable(GL_CULL_FACE);
	this->_material->drawOpenGL(glslProgram);

	double twoPi = 2 * Pi;

	double x, y, z;
	double latStep = static_cast<double>(Pi) / OpenGLTessellationComplexity;
	double lonStep = static_cast<double>(twoPi) / OpenGLTessellationComplexity;

	for (double lat = 0; lat <= Pi; lat += latStep) {
		glBegin(GL_QUAD_STRIP);
		for (double lon = 0; lon <= twoPi; lon += lonStep) {
			x = radius * cos(lon) * sin(lat);
			y = radius * sin(lon) * sin(lat);
			z = radius * cos(lat);
			glNormal3d(x, y, z);
			glTexCoord2d(lon / twoPi, lat / Pi);
			glVertex3d(x + center[0], y + center[1], z + center[2]);

			x = radius * cos(lon) * sin((lat + latStep));
			y = radius * sin(lon) * sin((lat + latStep));
			z = radius * cos((lat + latStep));
			glNormal3d(x, y, z);
			glTexCoord2d(lon / twoPi, (lat + latStep) / Pi);
			glVertex3d(x + center[0], y + center[1], z + center[2]);

			x = radius * cos((lon + lonStep)) * sin(lat);
			y = radius * sin((lon + lonStep)) * sin(lat);
			z = radius * cos(lat);
			glNormal3d(x, y, z);
			glTexCoord2d((lon + lonStep) / twoPi, lat / Pi);
			glVertex3d(x + center[0], y + center[1], z + center[2]);

			x = radius * cos((lon + lonStep)) * sin((lat + latStep));
			y = radius * sin((lon + lonStep)) * sin((lat + latStep));
			z = radius * cos((lat + latStep));
			glNormal3d(x, y, z);
			glTexCoord2d((lon + lonStep) / twoPi, (lat + latStep) / Pi);
			glVertex3d(x + center[0], y + center[1], z + center[2]);
		}
		glEnd();
	}*/
}
