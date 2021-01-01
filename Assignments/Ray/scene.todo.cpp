#include <cmath>
#include <Util/exceptions.h>
#include "scene.h"

using namespace Ray;
using namespace Util;
using namespace Image;

double clamp(double color)
{
	return std::max(0.0, std::min(1.0, color));
}

///////////
// Scene //
///////////
Point3D Scene::Reflect( Point3D v , Point3D n )
{
	//////////////////
	// Reflect here //
	//////////////////
	return v - n * (2.0 * (v).dot(n));
}

bool Scene::Refract( Point3D v , Point3D n , double ir , Point3D& refract )
{
	//////////////////
	// Refract here //
	//////////////////
	refract = v;

	double ni = 1.0;
	double nr = ir;

	if (v.dot(n) > 0) {
		ni = ir;
		nr = 1.0;
		n = -n;
	}

	double ti = acos(n.dot(-v) / v.length());
	double critical_angle = (ni / nr) * sin(ti);
	if (critical_angle > 1) return false;
	double tr = asin(ni * sin(ti) / nr);
	refract = n * ((ni / nr)*cos(ti) - cos(tr)) + v * ((ni / nr));
	return true;
}

Point3D Scene::getColor( Ray3D ray , int rDepth , Point3D cLimit, bool lightsOn )
{
	////////////////////////////////////////////////
	// Get the color associated with the ray here //
	////////////////////////////////////////////////
	if (rDepth <= 0) {
		GLfloat bg[4];
		glGetFloatv(GL_COLOR_CLEAR_VALUE, bg);
		Point3D background = { bg[0], bg[1], bg[2] };
		return background;
	}
	Point3D one; one[0] = 1; one[1] = 1; one[2] = 1;
	RayShapeIntersectionInfo iInfo;
	double dist = this->intersect(ray, iInfo, BoundingBox1D(), std::function< bool(double) >());
	if (dist == INFINITY) return Point3D();
	Point3D color;
	if (lightsOn) {
		color += iInfo.material->emissive;
		for (auto* light : this->_globalData.lights) {
			Point3D shadowTerm = light->transparency(iInfo, *this, cLimit);
			Point3D diffuse = light->getDiffuse(ray, iInfo);
			Point3D specular = light->getSpecular(ray, iInfo);
			if (iInfo.material->tex == NULL) {
				color += (diffuse + specular) * (shadowTerm);
				color += light->getAmbient(ray, iInfo);
			}
			else {
				Pixel32 texVal = iInfo.material->tex->_image.bilinearSample(iInfo.texture);
				Point3D tex{ texVal.r / 255.0, texVal.g / 255.0, texVal.b / 255.0 };
				color += (diffuse + specular) * shadowTerm * tex;
				color += light->getAmbient(ray, iInfo) * tex;
			}
		}
	} else {

		int count = 1;
		for (auto* light : this->_globalData.lights) {
			if (light->name() == "directional light")
			{
				Point3D shadowTerm = light->transparency(iInfo, *this, cLimit);
				Point3D diffuse = light->getDiffuse(ray, iInfo);
				Point3D specular = light->getSpecular(ray, iInfo);
				if (iInfo.material->tex == NULL) {
					color += (diffuse + specular) * (shadowTerm);
					color += light->getAmbient(ray, iInfo);
				}
				else {
					Pixel32 texVal = iInfo.material->tex->_image.bilinearSample(iInfo.texture);
					Point3D tex{ texVal.r / 255.0, texVal.g / 255.0, texVal.b / 255.0 };
					color += (diffuse + specular) * shadowTerm * tex;
					color += light->getAmbient(ray, iInfo) * tex;
				}
			}
			else {
				if (count == 1) {
					count++;
					if (iInfo.material->tex == NULL) {
						color += light->getAmbient(ray, iInfo);
					}
					else {
						Pixel32 texVal = iInfo.material->tex->_image.bilinearSample(iInfo.texture);
						Point3D tex{ texVal.r / 255.0, texVal.g / 255.0, texVal.b / 255.0 };
						color += light->getAmbient(ray, iInfo) * tex;
					}
				}
			}
		}
		return color;
	}

	//Reflection
	bool shouldReflect = false;
	for (int i = 0; i < 3; i++) {
		if (iInfo.material->specular[i] > cLimit[i]) {
			shouldReflect = true;
		}
	}
	if (rDepth > 0 && shouldReflect) {
		if (iInfo.normal.dot(ray.direction) < 0) {
			Ray3D reflect;
			reflect.direction = Reflect(ray.direction.unit(), iInfo.normal.unit());
			reflect.position = iInfo.position + reflect.direction * EPSILON;
			color += iInfo.material->specular * this->getColor(reflect, rDepth - 1, cLimit / iInfo.material->specular, lightsOn);
		}
	}
	color[0] = clamp(color[0]);
	color[1] = clamp(color[1]);
	color[2] = clamp(color[2]);

	//Refraction
	bool shouldRefract = false;
	for (int i = 0; i < 3; i++) {
		if (iInfo.material->transparent[i] > cLimit[i]) {
			shouldRefract = true;
		}
	}
	Point3D transparent;
	Point3D refractedDirection;
	if (rDepth > 0 && shouldRefract) {
		Point3D refractedDirection;
		if (Refract(ray.direction, iInfo.normal, iInfo.material->ir, refractedDirection)) {
			Ray3D refractedRay = Ray3D(iInfo.position + refractedDirection * EPSILON, refractedDirection);
			color += (iInfo.material->transparent * this->getColor(refractedRay, rDepth - 1, cLimit / iInfo.material->transparent, lightsOn));
		}
	}
	color[0] = clamp(color[0]);
	color[1] = clamp(color[1]);
	color[2] = clamp(color[2]);
	return color;
}

//////////////
// Material //
//////////////
void Material::drawOpenGL( GLSLProgram * glslProgram ) const
{
	//////////////////////////////
	// Do OpenGL rendering here //
	//////////////////////////////
	if (this->tex) {
		glEnable(GL_TEXTURE_2D);
		Texture temp = *tex;
		temp.initOpenGL();
	}
	GLfloat dif[] = { diffuse[0], diffuse[1], diffuse[2], 1.0 };
	GLfloat spec[] = { specular[0], specular[1], specular[2], 1.0 };
	if (Window::lightsON) {
		GLfloat emi[] = { emissive[0], emissive[1], emissive[2], 1.0 };
		glMaterialfv(GL_FRONT, GL_EMISSION, emi);
	} else {
		GLfloat emi[] = { 0, 0, 0, 1.0 };
		glMaterialfv(GL_FRONT, GL_EMISSION, emi);
	}
	GLfloat amb[] = { ambient[0], ambient[1], ambient[2], 1.0 };
	GLfloat shi[] = { this->specularFallOff };
	glMaterialfv(GL_FRONT, GL_DIFFUSE, dif);
	glMaterialfv(GL_FRONT, GL_SPECULAR, spec);
	glMaterialfv(GL_FRONT, GL_SHININESS, shi);
	glMaterialfv(GL_FRONT, GL_AMBIENT, amb);
	glClearColor(0.0, 0.0, 0.0, 0.0);
}

/////////////
// Texture //
/////////////
void Texture::initOpenGL(void)
{
	///////////////////////////////////
	// Do OpenGL texture set-up here //
	///////////////////////////////////
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
	glPixelStorei(GL_UNPACK_SKIP_PIXELS, 0);
	glPixelStorei(GL_UNPACK_SKIP_ROWS, 0);
	glBindTexture(GL_TEXTURE_2D, this->_openGLHandle);
	glGenTextures(1, &this->_openGLHandle);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, _image.width(), _image.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, _image.getData());
	glGenerateMipmap(GL_TEXTURE_2D);

}
