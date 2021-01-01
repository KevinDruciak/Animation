#include <cmath>
#include  <Util/exceptions.h>
#include "scene.h"
#include "box.h"

using namespace Ray;
using namespace Util;

/////////
// Box //
/////////

void Box::init(const LocalSceneData& data)
{
	// Set the material pointer
	if (_materialIndex < 0) THROW("negative material index: %d", _materialIndex);
	else if (_materialIndex >= data.materials.size()) THROW("material index out of bounds: %d <= %d", _materialIndex, (int)data.materials.size());
	else _material = &data.materials[_materialIndex];

	//////////////////////////////////
	// Do any necessary set-up here //
	//////////////////////////////////
	lengths.insert(lengths.end(), { Point3D{ length[0], 0.0, 0.0 }, Point3D{ 0.0, 0.0, length[2] },
		Point3D{ -length[0], 0.0, 0.0 }, Point3D{ 0.0, -length[1], 0.0 },
		Point3D{ 0.0, length[1], 0.0 }, Point3D{ length[0], 0.0, 0.0 },
		Point3D{ 0.0, -length[1], 0.0 }, Point3D{ 0.0, 0.0, -length[2] },
		Point3D{ 0.0, 0.0, length[2] }, Point3D{ -length[0], 0.0, 0.0 },
		Point3D{ 0.0, 0.0, -length[2] }, Point3D{ 0.0, length[1], 0.0 } });
	for (int i = 0; i < 6; i++) {
		Point3D corner = i % 2 == 0 ? _bBox[0] : _bBox[1];
		Point3D a = corner;
		Point3D b = corner + lengths[i * 2];
		Point3D c = corner + lengths[i * 2 + 1];
		planes.emplace_back(Plane3D(a, b, c));
	}
}

void Box::updateBoundingBox(void)
{
	///////////////////////////////
	// Set the _bBox object here //
	///////////////////////////////
	_bBox = BoundingBox3D(center - (length / 2), center + (length / 2));
}

double intersectPlane(Plane3D& plane, Ray3D& ray) {
	if (ray.direction.dot(plane.normal) == 0.0)  return INFINITY;
	double t = -((plane(ray.position) + EPSILON) / ray.direction.dot(plane.normal));
	if (t < 0.0) return INFINITY;
	return t;
}

double Box::intersect(Ray3D ray, RayShapeIntersectionInfo &iInfo, BoundingBox1D range, std::function< bool(double) > validityLambda) const
{
	RayTracingStats::IncrementRayPrimitiveIntersectionNum();

	/////////////////////////////////////////////////////////////
	// Compute the intersection of the shape with the ray here //
	/////////////////////////////////////////////////////////////

	double tmin = INFINITY;
	for (auto plane : planes) {
		double t = intersectPlane(plane, ray);
		if (t == INFINITY) return INFINITY;
		Point3D intersection = ray(t);
		bool intersect = _bBox.isInside(intersection);
		if (!intersect) continue;
		if (t < tmin) {
			tmin = t;
			iInfo.material = _material;
			iInfo.position = intersection;
			iInfo.normal = plane.normal;
			iInfo.texture = Point2D(intersection[0] / (_bBox[1][0] - _bBox[0][0]) + 0.5, intersection[1] / (_bBox[1][1] - _bBox[0][1]) + 0.5);
		}
	}
	return tmin;
}

bool Box::isInside(Point3D p) const
{
	///////////////////////////////////////////////////
	// Determine if the point is inside the box here //
	///////////////////////////////////////////////////
	THROW("method undefined");
	return false;
}

void Box::initOpenGL(void)
{
	/////////////////////////////////////////
	// Do any necessary OpenGL set-up here //
	/////////////////////////////////////////
	textures.insert(textures.end(), { 0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0 });
	normals.insert(normals.end(), { -1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, -1.0 });
	indices.insert(indices.end(),   {0,1,2,3,3,2,6,7,7,6,5,4,4,5,1,0,5,6,2,1,7,4,0,3});
	v[4][0] = v[5][0] = v[6][0] = v[7][0] = this->length[0] / 2 + this->center[0];
	v[2][1] = v[3][1] = v[6][1] = v[7][1] = this->length[1] / 2 + this->center[1];
	v[1][2] = v[2][2] = v[5][2] = v[6][2] = this->length[2] / 2 + this->center[2];
	v[0][0] = v[1][0] = v[2][0] = v[3][0] = -this->length[0] / 2 + this->center[0];
	v[0][1] = v[1][1] = v[4][1] = v[5][1] = -this->length[1] / 2 + this->center[1];
	v[0][2] = v[3][2] = v[4][2] = v[7][2] = -this->length[2] / 2 + this->center[2];
}

void Box::drawOpenGL(GLSLProgram * glslProgram) const
{
	//////////////////////////////
	// Do OpenGL rendering here //
	//////////////////////////////
	glShadeModel(GL_SMOOTH);
	this->_material->drawOpenGL(glslProgram);

	for (int i = 0; i < 6; i++) {
		glBegin(GL_QUADS);
		glNormal3f(normals[i*3], normals[i * 3 + 1], normals[i * 3 + 2]);
		glTexCoord2f(0,0);
		glVertex3fv(&v[indices[i * 4]][0]);
		glTexCoord2f(0,1);
		glVertex3fv(&v[indices[i * 4 + 1]][0]);
		glTexCoord2f(1,1);
		glVertex3fv(&v[indices[i * 4 + 2]][0]);
		glTexCoord2f(1,0);
		glVertex3fv(&v[indices[i * 4 + 3]][0]);
		glEnd();
	}
}