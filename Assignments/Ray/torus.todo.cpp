#include <cmath>
#include <Util/exceptions.h>
#include "scene.h"
#include "torus.h"

using namespace Ray;
using namespace Util;

///////////
// Torus //
///////////

void Torus::init(const LocalSceneData &data)
{
	// Set the material pointer
	if (_materialIndex < 0) THROW("negative material index: %d", _materialIndex);
	else if (_materialIndex >= data.materials.size()) THROW("material index out of bounds: %d <= %d", _materialIndex, (int)data.materials.size());
	else _material = &data.materials[_materialIndex];

	///////////////////////////////////
	// Do any additional set-up here //
	///////////////////////////////////
	WARN_ONCE("method undefined");
}
void Torus::updateBoundingBox(void)
{
	///////////////////////////////
	// Set the _bBox object here //
	///////////////////////////////
	Point3D p(iRadius + oRadius, oRadius, iRadius + oRadius);
	_bBox = BoundingBox3D(center - p, center + p);
}
void Torus::initOpenGL(void)
{
	///////////////////////////
	// Do OpenGL set-up here //
	///////////////////////////
	WARN_ONCE("method undefined");

	// Sanity check to make sure that OpenGL state is good
	ASSERT_OPEN_GL_STATE();
}

double Torus::intersect(Ray3D ray, RayShapeIntersectionInfo &iInfo, BoundingBox1D range, std::function< bool(double) > validityLambda) const
{
	RayTracingStats::IncrementRayPrimitiveIntersectionNum();

	/////////////////////////////////////////////////////////////
	// Compute the intersection of the shape with the ray here //
	/////////////////////////////////////////////////////////////
	//THROW( "method undefined" );
	return INFINITY;
}

bool Torus::isInside(Point3D p) const
{
	////////////////////////////////////////////////////////
	// Determine if the point is inside the cylinder here //
	////////////////////////////////////////////////////////
	THROW("method undefined");
	return false;
}

void Torus::drawOpenGL(GLSLProgram * glslProgram) const
{
	//////////////////////////////
	// Do OpenGL rendering here //
	//////////////////////////////
	glDisable(GL_CULL_FACE);

	this->_material->drawOpenGL(glslProgram);

	const double twoPi = 2 * Pi;

	for (int i = 0; i < OpenGLTessellationComplexity; i++) {
		glBegin(GL_QUAD_STRIP);
		for (int j = 0; j <= OpenGLTessellationComplexity; j++) {
			double x = (oRadius + iRadius * cos(i * twoPi / OpenGLTessellationComplexity)) * cos(j * twoPi / OpenGLTessellationComplexity);
			double y = (oRadius + iRadius * cos(i * twoPi / OpenGLTessellationComplexity)) * sin(j * twoPi / OpenGLTessellationComplexity);
			double z = iRadius * sin(i * twoPi / OpenGLTessellationComplexity);
			float alpha = oRadius / (sqrt(x * x + y * y));
			double u = i / static_cast<float>(OpenGLTessellationComplexity);
			double v = j / static_cast<float>(OpenGLTessellationComplexity);
			glTexCoord2d(u, v);
			glNormal3f((1 - alpha) * x, (1 - alpha) * y, z);
			glVertex3d(2 * x, 2 * y, 2 * z);

			x = (oRadius + iRadius * cos((i + 1) * twoPi / OpenGLTessellationComplexity)) * cos((j + 1) * twoPi / OpenGLTessellationComplexity);
			y = (oRadius + iRadius * cos((i + 1) * twoPi / OpenGLTessellationComplexity)) * sin((j + 1) * twoPi / OpenGLTessellationComplexity);
			z = iRadius * sin((i + 1) * twoPi / OpenGLTessellationComplexity);
			alpha = oRadius / (sqrt(x * x + y * y));
			u = (i + 1) / static_cast<float>(OpenGLTessellationComplexity);
			v = (j + 1) / static_cast<float>(OpenGLTessellationComplexity);
			glTexCoord2d(u, v);
			glNormal3f((1 - alpha) * x, (1 - alpha) * y, z);
			glVertex3d(2 * x, 2 * y, 2 * z);
		}
		glEnd();
	}
}
