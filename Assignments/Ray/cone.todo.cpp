#include <cmath>
#include <Util/exceptions.h>
#include <Util/polynomial.h>
#include "scene.h"
#include "cone.h"

using namespace Ray;
using namespace Util;

//////////
// Cone //
//////////

void Cone::init( const LocalSceneData &data )
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

void Cone::updateBoundingBox( void )
{
	///////////////////////////////
	// Set the _bBox object here //
	///////////////////////////////
	Point3D p(radius, height / 2, radius);
	_bBox = BoundingBox3D(center - p, center + p);
}

void Cone::initOpenGL( void )
{
	/////////////////////////////////////////
	// Do any necessary OpenGL set-up here //
	/////////////////////////////////////////
	WARN_ONCE( "method undefined" );

	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();
}

double Cone::intersect( Ray3D ray , RayShapeIntersectionInfo &iInfo , BoundingBox1D range , std::function< bool (double) > validityLambda ) const
{
	RayTracingStats::IncrementRayPrimitiveIntersectionNum();

	/////////////////////////////////////////////////////////////
	// Compute the intersection of the shape with the ray here //
	/////////////////////////////////////////////////////////////
	return INFINITY;
}

bool Cone::isInside( Point3D p ) const
{
	///////////////////////////////////////////////////
	// Determine if the point is inside the box here //
	///////////////////////////////////////////////////
	THROW( "method undefined" );
	return false;
}

void Cone::drawOpenGL( GLSLProgram * glslProgram ) const
{
	//////////////////////////////
	// Do OpenGL rendering here //
	//////////////////////////////
	glDisable(GL_CULL_FACE);
	this->_material->drawOpenGL(glslProgram);
	unsigned cplx = this->_material->foo == "roof" ? 4 : OpenGLTessellationComplexity;

	glBegin(GL_TRIANGLE_FAN);
	glNormal3f(0, 0, 1);
	glVertex3f(center[0], center[1], height);
	for (int i = 0; i < cplx + 2; i++) {
		glNormal3f(center[0] + (radius * cos(i *  (2 * Pi) / cplx)), center[1] + (radius * sin(i * (2 * Pi) / cplx)), 0);
		glVertex3f(center[0] + (radius * cos(i *  (2 * Pi) / cplx)), center[1] + (radius * sin(i * (2 * Pi) / cplx)), 0);
	}
	glEnd();

	glBegin(GL_TRIANGLE_FAN);
	glNormal3f(0, 0, -1);
	glVertex3f(center[0], center[1], 0);
	for (int i = 0; i <= cplx; i++) {
		glVertex3f(center[0] + (radius * cos(i *  (2 * Pi) / cplx)), center[1] + (radius * sin(i * (2 * Pi) / cplx)), 0);
	}
	glEnd();
}
