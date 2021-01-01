/*
Copyright (c) 2019, Michael Kazhdan
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list of
conditions and the following disclaimer. Redistributions in binary form must reproduce
the above copyright notice, this list of conditions and the following disclaimer
in the documentation and/or other materials provided with the distribution.

Neither the name of the Johns Hopkins University nor the names of its contributors
may be used to endorse or promote products derived from this software without specific
prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
TO, PROCUREMENT OF SUBSTITUTE  GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#include <cmath>
#include <SVD/SVDFit.h>
#include <SVD/MatrixMNTC.h>
#include <Util/exceptions.h>
#include "geometry.h"

namespace Util
{
	////////////////////////////
	// EulerRotationParameter //
	////////////////////////////
	Matrix3D EulerRotationParameter::operator() ( void ) const
	{
		///////////////////////////////////////////////
		// Transform Euler angles to a rotation here //
		///////////////////////////////////////////////
		Matrix3D m = Matrix3D();
		double cx = cos(parameter[0]);
		double sx = sin(parameter[0]);
		double cy = cos(parameter[1]);
		double sy = sin(parameter[1]);
		double cz = cos(parameter[2]);
		double sz = sin(parameter[2]);

		m[0][0] = cy * cz;
		m[1][0] = sx * sy*cz - cx * sz;
		m[2][0] = cx * sy*cz + sx * sz;
		m[0][1] = cy * sz;
		m[1][1] = sx * sy*sz + cx * cz;
		m[2][1] = cx * sy*sz - sx * cz;
		m[0][2] = -sy;
		m[1][2] = cy * sx;
		m[2][2] = cy * cx;
		return m;
	}

	/////////////////////////////////
	// QuaternionRotationParameter //
	/////////////////////////////////
	Matrix3D QuaternionRotationParameter::operator()( void ) const
	{
		///////////////////////////////////////////////
		// Transform a quaternion to a rotation here //
		///////////////////////////////////////////////
		Quaternion q = this->parameter.unit();
		double a = q.real;
		double b = q.imag[0];
		double c = q.imag[1];
		double d = q.imag[2];

		Matrix3D m = Matrix3D();

		m[0][0] = 1 - 2 * (c*c) - 2 * (d*d);
		m[1][0] = 2 * (b*c) - 2 * (a*d);
		m[2][0] = 2 * (b*d) + 2 * (a*c);

		m[0][1] = 2 * (b*c) + 2 * (a*d);
		m[1][1] = 1 - 2 * (b*b) - 2 * (d*d);
		m[2][1] = 2 * (c* d) - 2 * (a*b);

		m[0][2] = 2 * (b*d) - 2 * (a*c);
		m[1][2] = 2 * (c*d) + 2 * (a*b);
		m[2][2] = 1 - 2 * (b*b) - 2 * (c*c);
		return m;
	}
}
