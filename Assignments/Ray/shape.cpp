#include "shape.h"

using namespace Ray;
using namespace Util;

//////////////////////
// ShapeBoundingBox //
//////////////////////
BoundingBox1D ShapeBoundingBox::intersect( const Ray3D &ray ) const
{
	RayTracingStats::IncrementRayBoundingBoxIntersectionNum();
	return Util::BoundingBox3D::intersect( ray );
}

///////////
// Shape //
///////////
unsigned int Shape::OpenGLTessellationComplexity;

unsigned int Shape::WriteInsetSize = 0;

void Shape::WriteInset( std::ostream &stream ){ for( unsigned int i=0 ; i<WriteInsetSize ; i++ ) stream << "  "; }

ShapeBoundingBox Shape::boundingBox( void ) const { return _bBox; }

//////////////////////////
// RayIntersectionStats //
//////////////////////////
size_t RayTracingStats::_RayNum = 0;
size_t RayTracingStats::_RayPrimitiveIntersectionNum = 0;
size_t RayTracingStats::_RayBoundingBoxIntersectionNum = 0;

void RayTracingStats::Reset( void ){ _RayNum = _RayPrimitiveIntersectionNum = _RayBoundingBoxIntersectionNum = 0; }
void RayTracingStats::IncrementRayNum( void ){ _RayNum++; }
void RayTracingStats::IncrementRayPrimitiveIntersectionNum( void ){ _RayPrimitiveIntersectionNum++; }
void RayTracingStats::IncrementRayBoundingBoxIntersectionNum( void ){ _RayBoundingBoxIntersectionNum++; }
size_t RayTracingStats::RayNum( void ){ return _RayNum; }
size_t RayTracingStats::RayPrimitiveIntersectionNum( void ){ return _RayPrimitiveIntersectionNum; }
size_t RayTracingStats::RayBoundingBoxIntersectionNum( void ){ return _RayBoundingBoxIntersectionNum; }
