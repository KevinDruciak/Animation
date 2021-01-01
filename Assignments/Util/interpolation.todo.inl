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

#include <math.h>
#include <Util/exceptions.h>

namespace Util
{
	///////////////////
	// Interpolation //
	///////////////////
	template< typename SampleType >
	SampleType Interpolation::Sample( const std::vector< SampleType > &samples , double t , int interpolationType )
	{
		switch( interpolationType )
		{
		case NEAREST:
		{
			t *= samples.size();
			int it1 = (int)floor(t);
			int it2 = ( it1 + 1 ) % samples.size();
			t -= it1;
			if( t<0.5 ) return samples[it1];
			else        return samples[it2];
			break;
		}
		case LINEAR:
		{
			///////////////////////////////////////
			// Perform linear interpolation here //
			///////////////////////////////////////
			t *= samples.size();
			int it1 = static_cast<int>(floor(t)) % samples.size();
			int it2 = (it1 + 1) % samples.size();
			return samples[it1] * (1 - (t - it1)) + samples[it2 % samples.size()] * (t - it1);
			break;
		}
		case CATMULL_ROM:
			////////////////////////////////////////////
			// Perform Catmull-Rom interpolation here //
			////////////////////////////////////////////
		{
			t *= samples.size();
			int it1 = static_cast<int>(floor(t)) % samples.size();
			int it2 = (it1 + 1) % samples.size();
			float u1 = t - it1;
			float u2 = pow(u1, 2);
			float u3 = pow(u1, 3);
			auto a = -0.5 * u3 + u2 - 0.5 * u1;
			auto b = 1.5 * u3 + -2.5 * u2 + 1;
			auto c = -1.5 * u3 + 2 * u2 + 0.5 * u1;
			auto d = 0.5 * u3 - 0.5 * u2;
			return samples[(it1 - 1) % samples.size()] * a + samples[it1] * b + samples[(it2) % samples.size()] * c + samples[(it2) % samples.size()] * d;
			break;
		}
		case UNIFORM_CUBIC_B_SPLINE:
			///////////////////////////////////////////////////////
			// Perform uniform cubic b-spline interpolation here //
			///////////////////////////////////////////////////////
		{
			t *= samples.size();
			int it1 = static_cast<int>(floor(t)) % samples.size();
			int it2 = (it1 + 1) % samples.size();
			float u1 = t - it1;
			float u2 = pow(u1, 2);
			float u3 = pow(u1, 3);
			auto a = -1.0 / 6.0 * u3 + 0.5 * u2 - 0.5 * u1 + 1.0 / 6.0;
			auto b = 0.5 * u3 + -u2 + 2.0 / 3.0;
			auto c = -0.5 * u3 + 0.5 * u2 + 0.5 * u1 + 1.0 / 6.0;
			auto d = 1.0 / 6.0 * u3;
			return samples[(it1 - 1) % samples.size()] * a + samples[it1] * b + samples[it2] * c + samples[(it2 + 1) % samples.size()] * d;
			break;
		}
		default:
			ERROR_OUT( "unrecognized interpolation type" );
			return samples[0];
		}
	}
}