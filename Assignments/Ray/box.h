#ifndef BOX_INCLUDED
#define BOX_INCLUDED
#include <Util/geometry.h>
#include "shape.h"

namespace Ray
{
	/** This class represents a box and is defined by its center and the length of the sides. */
	class Box : public Shape
	{
		/** The index of the material associated with the box */
		int _materialIndex;

		/** The material associated with the box */
		const class Material *_material;

		std::vector<Util::Point3D> lengths;

		std::vector<Util::Plane3D> planes;

		std::vector<GLfloat> textures;
		std::vector<GLint> indices;
		std::vector<GLfloat> normals;
		GLfloat v[8][3];

	public:
		/** The center of the box */
		Util::Point3D center;

		/** The lengths of the sides of the box */
		Util::Point3D length;

		/** This static method returns the directive describing the shape. */
		static std::string Directive( void ){ return "shape_box"; }

		/** The default constructor */
		Box( void );

		///////////////////
		// Shape methods //
		///////////////////
	private:
		void _write( std::ostream &stream ) const;
		void _read( std::istream &stream );
	public:
		std::string name( void ) const { return "box"; }
		void init( const class LocalSceneData& data  );
		void initOpenGL( void );
		void updateBoundingBox( void );
		double intersect( Util::Ray3D ray , class RayShapeIntersectionInfo &iInfo , Util::BoundingBox1D range = Util::BoundingBox1D( Util::Epsilon , Util::Infinity ) , std::function< bool (double) > validityFunction = [] ( double t ){ return true; } ) const;
		bool isInside( Util::Point3D p ) const;
		void drawOpenGL( GLSLProgram *glslProgram ) const;
		size_t primitiveNum( void ) const;
		bool implemented() { return true; }
	};
}
#endif // BOX_INCLUDED

