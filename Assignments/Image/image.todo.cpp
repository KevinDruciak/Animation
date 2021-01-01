#include <algorithm>
#include "image.h"
#include <stdlib.h>
#include <math.h>
#include <Util/exceptions.h>

using namespace Util;
using namespace Image;

/////////////
// Image32 //
/////////////
Image32 Image32::addRandomNoise( double noise ) const
{
	///////////////////////////
	// Add random noise here //
	///////////////////////////
	THROW( "method undefined" );
	return Image32();
}

Image32 Image32::brighten( double brightness ) const
{
	/////////////////////////
	// Do brightening here //
	/////////////////////////
	THROW( "method undefined" );
	return Image32();
}

Image32 Image32::luminance( void ) const
{
	//////////////////////////////////
	// Compute luminance image here //
	//////////////////////////////////
	THROW( "method undefined" );
	return Image32();
}

Image32 Image32::contrast( double contrast ) const
{
	//////////////////////////////////
	// Do contrast enhancement here //
	//////////////////////////////////
	THROW( "method undefined" );
	return Image32();
}

Image32 Image32::saturate( double saturation ) const
{
	////////////////////////////////////
	// Do saturation enhancement here //
	////////////////////////////////////
	THROW( "method undefined" );
	return Image32();
}

Image32 Image32::quantize( int bits ) const
{
	//////////////////////////
	// Do quantization here //
	//////////////////////////
	THROW( "method undefined" );
	return Image32();
}

Image32 Image32::randomDither( int bits ) const
{
	//////////////////////////////
	// Do random dithering here //
	//////////////////////////////
	THROW( "method undefined" );
	return Image32();
}

Image32 Image32::orderedDither2X2( int bits ) const
{
	///////////////////////////////
	// Do ordered dithering here //
	///////////////////////////////
	THROW( "method undefined" );
	return Image32();
}

Image32 Image32::floydSteinbergDither( int bits ) const
{
	///////////////////////////////////////
	// Do Floyd-Steinberg dithering here //
	///////////////////////////////////////
	THROW( "method undefined" );
	return Image32();
}

Image32 Image32::blur3X3( void ) const
{
	//////////////////////
	// Do blurring here //
	//////////////////////
	THROW( "method undefined" );
	return Image32();
}

Image32 Image32::edgeDetect3X3( void ) const
{
	////////////////////////////
	// Do edge detection here //
	////////////////////////////
	THROW( "method undefined" );
	return Image32();
}

Image32 Image32::scaleNearest( double scaleFactor ) const
{
	/////////////////////////////////////////////////
	// Do scaling with nearest-point sampling here //
	/////////////////////////////////////////////////
	THROW( "method undefined" );
	return Image32();
}

Image32 Image32::scaleBilinear( double scaleFactor ) const
{
	////////////////////////////////////////////
	// Do scaling with bilinear sampling here //
	////////////////////////////////////////////
	THROW( "method undefined" );
	return Image32();
}

Image32 Image32::scaleGaussian( double scaleFactor ) const
{
	////////////////////////////////////////////
	// Do scaling with Gaussian sampling here //
	////////////////////////////////////////////
	THROW( "method undefined" );
	return Image32();
}

Image32 Image32::rotateNearest( double angle ) const
{
	//////////////////////////////////////////////////
	// Do rotation with nearest-point sampling here //
	//////////////////////////////////////////////////
	THROW( "method undefined" );
	return Image32();
}

Image32 Image32::rotateBilinear( double angle ) const
{
	/////////////////////////////////////////////
	// Do rotation with bilinear sampling here //
	/////////////////////////////////////////////
	THROW( "method undefined" );
	return Image32();
}

Image32 Image32::rotateGaussian( double angle ) const
{
	/////////////////////////////////////////////
	// Do rotation with Gaussian sampling here //
	/////////////////////////////////////////////
	THROW( "method undefined" );
	return Image32();
}

void Image32::setAlpha( const Image32& matte )
{
	///////////////////////////
	// Set alpha values here //
	///////////////////////////
	THROW( "method undefined" );
}

Image32 Image32::composite( const Image32& overlay ) const
{
	/////////////////////////
	// Do compositing here //
	/////////////////////////
	THROW( "method undefined" );
	return Image32();
}

Image32 Image32::CrossDissolve( const Image32& source , const Image32& destination , double blendWeight )
{
	////////////////////////////
	// Do cross-dissolve here //
	////////////////////////////
	THROW( "method undefined" );
	return Image32();
}

Image32 Image32::warp( const OrientedLineSegmentPairs& olsp ) const
{
	/////////////////////
	// Do warping here //
	/////////////////////
	THROW( "method undefined" );
	return Image32();
}

Image32 Image32::funFilter( void ) const
{
	////////////////////////////
	// Do the fun-filter here //
	////////////////////////////
	THROW( "method undefined" );
	return Image32();
}

Image32 Image32::crop( int x1 , int y1 , int x2 , int y2 ) const
{
	//////////////////////
	// Do cropping here //
	//////////////////////
	THROW( "method undefined" );
	return Image32();
}

Pixel32 Image32::nearestSample( Point2D p ) const
{
	//////////////////////////////
	// Do nearest sampling here //
	//////////////////////////////
	THROW( "method undefined" );
	return Pixel32();
}

bool checkBounds(double val, int size)
{
	return val > size - 1 ? false : val < 0 ? false : true;
}

Pixel32 Image32::bilinearSample( Point2D p ) const
{
	///////////////////////////////
	// Do bilinear sampling here //
	///////////////////////////////
	Pixel32 newPix;
	double u = p[0] * (double)this->_width;
	double v = p[1] * (double)this->_height;

	double u1 = floor(u);
	double u2 = u1 + 1;
	double v1 = floor(v);
	double v2 = v1 + 1;
	double du = u - u1;
	double dv = v - v1;

	Pixel32 bl;
	Pixel32 br;
	Pixel32 tl;
	Pixel32 tr;

	if (!checkBounds(u1, _width) || !checkBounds(v1, _height)) {
		bl.a = 0;
		bl.r = 0;
		bl.b = 0;
		bl.g = 0;
	}
	else {
		bl.a = (*this)(u1, v1).a;
		bl.r = (*this)(u1, v1).r;
		bl.b = (*this)(u1, v1).b;
		bl.g = (*this)(u1, v1).g;
	}

	if (!checkBounds(u2, _width) || !checkBounds(v1, _height)) {
		br.a = 0;
		br.r = 0;
		br.b = 0;
		br.g = 0;
	}
	else {
		br.a = (*this)(u2, v1).a;
		br.r = (*this)(u2, v1).r;
		br.b = (*this)(u2, v1).b;
		br.g = (*this)(u2, v1).g;
	}

	if (!checkBounds(u1, _width) || !checkBounds(v2, _height)) {
		tl.a = 0;
		tl.r = 0;
		tl.b = 0;
		tl.g = 0;
	}
	else {
		tl.a = (*this)(u1, v2).a;
		tl.r = (*this)(u1, v2).r;
		tl.b = (*this)(u1, v2).b;
		tl.g = (*this)(u1, v2).g;
	}

	if (!checkBounds(u2, _width) || !checkBounds(v2, _height)) {
		tr.a = 0;
		tr.r = 0;
		tr.b = 0;
		tr.g = 0;
	}
	else {
		tr.a = (*this)(u2, v2).a;
		tr.r = (*this)(u2, v2).r;
		tr.b = (*this)(u2, v2).b;
		tr.g = (*this)(u2, v2).g;
	}

	double a = bl.a * (1 - du) + br.a * du;
	double b = tl.a * (1 - du) + tr.a * du;
	newPix.a = a * (1 - dv) + b * dv;

	a = bl.r * (1 - du) + br.r * du;
	b = tl.r * (1 - du) + tr.r * du;
	newPix.r = a * (1 - dv) + b * dv;

	a = bl.g * (1 - du) + br.g * du;
	b = tl.g * (1 - du) + tr.g * du;
	newPix.g = a * (1 - dv) + b * dv;

	a = bl.b * (1 - du) + br.b * du;
	b = tl.b * (1 - du) + tr.b * du;
	newPix.b = a * (1 - dv) + b * dv;

	return newPix;
}

Pixel32 Image32::gaussianSample( Point2D p , double variance , double radius ) const
{
	///////////////////////////////
	// Do Gaussian sampling here //
	///////////////////////////////
	THROW( "method undefined" );
	return Pixel32();
}
