#ifndef TRIANGLE_INCLUDED
#define TRIANGLE_INCLUDED
#include <Util/geometry.h>
#include <Util/polynomial.h>
#include "shape.h"
#include "scene.h"

namespace Ray
{
	/** A simple class for storing a triplet of triangle indices.*/
	class TriangleIndex
	{
		/** The indices of the triangle vertices */
		GLuint _indices[3];

	public:
		/** The default constructor */
		TriangleIndex( void ){ _indices[0] = _indices[1] = _indices[2] = 0; }

		/** The constructor assigning the vertex indices */
		TriangleIndex( unsigned int i0 , unsigned int i1 , unsigned int i2 ){ _indices[0] = i0 , _indices[1] = i1 , _indices[2] = i2; }

		/** This operator returns a reference to the vertex index. */
		GLuint& operator[] ( int idx )       { return _indices[idx]; }

		/** This operator returns a reference to the vertex index. */
		const GLuint &operator[] ( int idx ) const { return _indices[idx]; }
	};

	/** This class represents a triangle and is specified by three pointers to the three vertices that define it. */
	class Triangle : public Shape
	{
		/** The indices of the vertices associated with the Triangle */
		size_t _vIndices[3];

		/** Edge found by _v3 - _v1 */
		Util::Point3D edge1;

		/** Edge found by _v2 - _v1 */
		Util::Point3D edge2;

		/** Plane spanned by the vertices of the this triangle */
		Util::Plane3D plane;

		/** Pre-compute determinants for calculating beta and gamma */
		double det1;
		double det2;
		double det3;

		/** The vertices associated with the triangle */
		const class Vertex* _v[3];

	public:
		/** This static method returns the directive describing the shape. */
		static std::string Directive( void ){ return "shape_triangle"; }

		/** The default constructor */
		Triangle( void );

		///////////////////
		// Shape methods //
		///////////////////
	private:
		void _write( std::ostream &stream ) const;
		void _read( std::istream &stream );
	public:
		std::string name( void ) const { return "triangle"; }
		void init( const class LocalSceneData &data );
		void initOpenGL( void );
		void updateBoundingBox( void );
		double intersect( Util::Ray3D ray , class RayShapeIntersectionInfo &iInfo , Util::BoundingBox1D range = Util::BoundingBox1D( Util::Epsilon , Util::Infinity ) , std::function< bool (double) > validityFunction = [] ( double t ){ return true; } ) const;
		bool isInside( Util::Point3D p ) const;
		void addTrianglesOpenGL( std::vector< TriangleIndex >& triangles );
		void drawOpenGL( GLSLProgram * glslProgram ) const;
		size_t primitiveNum( void ) const;
		bool implemented() { return true; }

	};
}
#endif // TRIANGLE_INCLUDED


