#include <cmath>
#include <Util/exceptions.h>
#include "triangle.h"

using namespace Ray;
using namespace Util;

//////////////
// Triangle //
//////////////

void Triangle::init( const LocalSceneData &data )
{
	// Set the vertex pointers
	for( int i=0 ; i<3 ; i++ )
	{
		if( _vIndices[i]==-1 ) THROW( "negative vertex index: %d" , _vIndices[i] );
		else if( _vIndices[i]>=data.vertices.size() ) THROW( "vertex index out of bounds: %d <= %d" , _vIndices[i] , (int)data.vertices.size() );
		else _v[i] = &data.vertices[ _vIndices[i] ];
	}

	///////////////////////////////////
	// Do any additional set-up here //
	///////////////////////////////////
	edge1 = _v[2]->position - _v[0]->position;
	edge2 = _v[1]->position - _v[0]->position;
	plane.normal = -Point3D::CrossProduct(edge1, edge2).unit();
	plane.distance = -(_v[2]->position.dot(plane.normal));
	det1 = edge1[0] * edge2[1] - edge2[0] * edge1[1];
	det2 = edge1[0] * edge2[2] - edge2[0] * edge1[2];
	det3 = edge1[1] * edge2[2] - edge2[1] * edge1[2];
}
void Triangle::updateBoundingBox( void )
{
	///////////////////////////////
	// Set the _bBox object here //
	///////////////////////////////
	Point3D pList[3];
	for (int i = 0; i < 3; i++) pList[i] = _v[i]->position;
	_bBox = BoundingBox3D(pList, 3);
	for (int i = 0; i < 3; i++) _bBox[0][i] -= Epsilon, _bBox[1][i] += Epsilon;
}
void Triangle::initOpenGL( void )
{
	///////////////////////////
	// Do OpenGL set-up here //
	///////////////////////////
	WARN_ONCE( "method undefined" );
}

double Triangle::intersect( Ray3D ray , RayShapeIntersectionInfo& iInfo , BoundingBox1D range , std::function< bool (double) > validityLambda ) const
{
	RayTracingStats::IncrementRayPrimitiveIntersectionNum();

	/////////////////////////////////////////////////////////////
	// Compute the intersection of the shape with the ray here //
	/////////////////////////////////////////////////////////////
	RayTracingStats::IncrementRayPrimitiveIntersectionNum();

	/////////////////////////////////////////////////////////////
	// Compute the intersection of the shape with the ray here //
	/////////////////////////////////////////////////////////////

	const double distance = (-plane.distance - plane.normal.dot(ray.position)) / plane.normal.dot(ray.direction);
	if (distance < 0) return INFINITY;

	Point3D p = ray(distance) - _v[0]->position;

	double gamma, beta;

	if (det1 != 0) {
		gamma = (1 / det1) * (edge2[1] * p[0] - edge2[0] * p[1]);
		beta = (1 / det1) * (-edge1[1] * p[0] + edge1[0] * p[1]);
	}
	else if (det2 != 0) {
		gamma = (1 / det2) * (edge2[2] * p[0] - edge2[0] * p[2]);
		beta = (1 / det2) * (-edge1[2] * p[0] + edge1[0] * p[2]);
	}
	else {
		gamma = (1 / det3) * (edge2[2] * p[1] - edge2[1] * p[2]);
		beta = (1 / det3) * (-edge1[2] * p[1] + edge1[1] * p[2]);
	}
	const double alpha = 1 - (gamma + beta);

	if (alpha < 0 || alpha > 1 || beta < 0 || beta > 1 || gamma < 0 || gamma > 1) return INFINITY;
	iInfo.position = ray(distance);
	iInfo.normal = this->plane.normal;
	iInfo.texture = this->_v[0]->texCoordinate * alpha + this->_v[1]->texCoordinate * beta + this->_v[2]->texCoordinate * gamma;
	return distance;
}

void Triangle::drawOpenGL( GLSLProgram * glslProgram ) const
{
	//////////////////////////////
	// Do OpenGL rendering here //
	//////////////////////////////
	glDisable(GL_CULL_FACE);
	Plane3D plane = Plane3D(this->_v[0]->position, this->_v[1]->position, this->_v[2]->position);
	glBegin(GL_TRIANGLES);
	glNormal3f(plane.normal[0], plane.normal[1], plane.normal[2]);
	glTexCoord2f(this->_v[0]->texCoordinate[0], this->_v[0]->texCoordinate[1]);
	glVertex3f(this->_v[0]->position[0], this->_v[0]->position[1], this->_v[0]->position[2]);
	glTexCoord2f(this->_v[1]->texCoordinate[0], this->_v[1]->texCoordinate[1]);
	glVertex3f(this->_v[1]->position[0], this->_v[1]->position[1], this->_v[1]->position[2]);
	glTexCoord2f(this->_v[2]->texCoordinate[0], this->_v[2]->texCoordinate[1]);
	glVertex3f(this->_v[2]->position[0], this->_v[2]->position[1], this->_v[2]->position[2]);
	glEnd();
}
