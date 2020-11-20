/*
MIT LICENSE

Copyright (c) 2014-2020 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <string.h>
#include <stdlib.h>
#include "ISMatrix.h"
#include "data_sets.h"

void LU( const f_t *M, i_t n, f_t *L, f_t *U );
char solve_upper( f_t *result, i_t n, f_t *A, f_t *b );
char solve_lower( f_t *result, i_t n, f_t *A, f_t *b );

void mul_MatMxN( void * result, const void * A_ptr, const void * B_ptr, i_t m, i_t n, i_t p, char transpose_B, char add )
{
	i_t i;
	i_t j;
	i_t k;

	f_t * matOut = (f_t*)result;
	const f_t * A = (const f_t*)A_ptr;
	const f_t * B = (const f_t*)B_ptr;

	for (i = 0; i < m; i++)
	{
		const f_t * A_i = A + i * n;
		f_t * O_i = matOut + i * p;

		for (j = 0; j < p; j++)
		{
			f_t s = 0;
			f_t * O_i_j = O_i + j;

			for (k = 0; k < n; k++)
			{
				const f_t * a = A_i + k;
				const f_t * b;

				if (is_zero(a))
					continue;

				if (transpose_B)
					b = B + j * n + k;
				else
					b = B + k * p + j;

				if (is_zero(b))
					continue;

				s += *a * *b;
			}

			if (add == 0)
				*O_i_j = s;
			else if (add > 0)
				*O_i_j += s;
			else
				*O_i_j -= s;
		}
	}
}

#if 0
void addNxM(void * result, const void * A_ptr, const void * B_ptr, i_t m, i_t n, char transpose_B)
{
    i_t i;
    i_t j;
    i_t k;

    f_t * OUT = result;
    const f_t * A = A_ptr;
    const f_t * B = B_ptr;

    for (i = 0; i < m; i++)
    {
        const f_t * A_i = A + i * n;
        f_t * O_i = OUT + i * p;

        for (j = 0; j < p; j++)
        {
//             f_t s = 0;
            f_t * O_i_j = O_i + j;


            for (k = 0; k < n; k++)
            {
                const f_t * a = A_i + k;
                const f_t * b;

//                 if (is_zero(a))
//                     continue;

                if (transpose_B)
                    b = B + j * n + k;
                else
                    b = B + k * p + j;

                if (is_zero(b))
                    continue;

                s += *a * *b;
            }

            *O_i_j = s;
        }
    }
}
#endif


void eye_MatN( f_t *A, i_t n )
{
	zero_MatMxN( A, n, n );

	// Set diagonals to 1
	for( int i=0; i < n; i++ )
		A[i*n + i] = 1;
}


// Compute the LU factorization of the square matrix A
void LU( const f_t *M, i_t n, f_t *L, f_t *U )
{
	int in, kn;

	f_t *A = (f_t*)MALLOC( sizeof( f_t )*n*n );
	if (A == 0) { return; }

	cpy_MatMxN( A, M, n, n );

	for( int k=0; k < n - 1; k++ )
	{
		for( int i=k + 1; i < n; i++ )
		{
			in = i*n;
			kn = k*n;
// 			f_t * Ai = A + i*n;

			A[in + k] = A[in + k] / A[kn + k];

			for( int j=k + 1; j < n; j++ )
			{
#ifdef NO_FPU
				T &		A_i_j( A[in + j] );
				const T &	A_i_k( A[in + k] );
				const T &	A_k_j( A[kn + j] );

				if( is_zero( A_i_k )
					|| is_zero( A_k_j )
					)
					continue;

				A_i_j -= A_i_k * A_k_j;
#else
				A[in + j] -= A[in + k] * A[kn + j];
#endif
			}
		}
	}

	eye_MatN( L, n );

	/* Separate the L matrix */
	for( int j=0; j < n - 1; j++ )
		for( int i=j + 1; i < n; i++ )
			L[i*n + j] = A[i*n + j];

	/* Separate the M matrix */
	zero_MatMxN( U, n, n );

	for( int i=0; i < n; i++ )
		for( int j=i; j < n; j++ )
			U[i*n + j] = A[i*n + j];
			
	FREE( A );
}

// Return 0 on success, -1 on numerical error
char solve_upper( f_t *result, i_t n, f_t *A, f_t *b )
{
	for( int i=n - 1; i >= 0; i-- )
	{
		f_t s = b[i];

		// Reference a row
		f_t *A_i = &A[i*n];

		for( int j=i + 1; j < n; ++j )
		{
#ifdef NO_FPU
			const T &	A_i_j( A_i[j] );
			const T &	x_j( x[j] );

			if( is_zero( A_i_j ) || is_zero( x_j ) )
				continue;

			s -= A_i_j * x_j;
#else
			s -= A_i[j] * result[j*n];
#endif
		}

		// Prevent divide by zero
		if( A_i[i]==0.0f )
			return -1;
		
		result[i*n] = s / A_i[i];
	}
	
	return 0;
}

// Return 0 on success, -1 on numerical error
char solve_lower( f_t *result, i_t n, f_t *A, f_t *b )
{
	for( int i=0; i < n; ++i )
	{
		f_t s = b[i];

		// Reference a row
		f_t *A_i = &A[i*n];

		for( int j=0; j < i; ++j )
		{
#ifdef NO_FPU
			const T &	A_i_j( A_i[j] );
			const T &	x_j( x[j] );

			if( is_zero( A_i_j )|| is_zero( x_j ) )
				continue;

			s -= A_i_j * x_j;
#else
			s -= A_i[j] * result[j*n];
#endif
		}

		// Prevent divide by zero
		if( A_i[i]==0.0f )
			return -1;

		result[i*n] = s / A_i[i];
	}
	
	return 0;
}


// Return 0 on success, -1 on numerical error
char inv_MatN( f_t *result, const f_t *M, i_t n )
{
	char error		= 0;

	f_t *L			= (f_t*)MALLOC( sizeof( f_t )*n*n );
	f_t *U			= (f_t*)MALLOC( sizeof( f_t )*n*n );
	f_t *invL		= (f_t*)MALLOC( sizeof( f_t )*n*n );
	f_t *invU		= (f_t*)MALLOC( sizeof( f_t )*n*n );
	f_t	*identCol	= (f_t*)MALLOC( sizeof( f_t )*n );
	while( L==NULL || U==NULL || invL==NULL || invU==NULL || identCol==NULL ) { /* Error check malloc */ }
	memset( identCol, 0, sizeof( f_t )*n );

	LU( M, n, L, U );

	for( int i=0; i < n; i++ )
	{
		identCol[i] = 1;
		if( solve_upper( &invU[i], n, U, identCol ) ||		// Fill a column
			solve_lower( &invL[i], n, L, identCol ) )
		{		
			error = -1;		
			break;
		}
		identCol[i] = 0;
	}

	// result = invU * invL
	if( !error )
		mul_MatMxN( result, invU, invL, n, n, n, 0, 0 );

	FREE( L );
	FREE( U );
	FREE( invL );
	FREE( invU );
	FREE( identCol );

	return error;
}


void trans_MatMxN( f_t *result, const f_t *M, int m, int n )
{
	i_t i;
	i_t j;

	const f_t * A = (const f_t*)M;

	for( i = 0; i < m; i++ )
	{
		f_t * O_i = result + i;

		for( j = 0; j < n; j++ )
		{
			// Copy value
			*O_i = *A;

			// Increment pointers
			A++;
			O_i += m;
		}
	}
}


void mul_Mat3x3_Mat3x3( Matrix3 result, const Matrix3 m1, const Matrix3 m2 )
{
	// Row 1
	result[0] = m1[0]*m2[0] + m1[1]*m2[3] + m1[2]*m2[6];
	result[1] = m1[0]*m2[1] + m1[1]*m2[4] + m1[2]*m2[7];
	result[2] = m1[0]*m2[2] + m1[1]*m2[5] + m1[2]*m2[8];
	// Row 2
	result[3] = m1[3]*m2[0] + m1[4]*m2[3] + m1[5]*m2[6];
	result[4] = m1[3]*m2[1] + m1[4]*m2[4] + m1[5]*m2[7];
	result[5] = m1[3]*m2[2] + m1[4]*m2[5] + m1[5]*m2[8];
	// Row 3
	result[6] = m1[6]*m2[0] + m1[7]*m2[3] + m1[8]*m2[6];
	result[7] = m1[6]*m2[1] + m1[7]*m2[4] + m1[8]*m2[7];
	result[8] = m1[6]*m2[2] + m1[7]*m2[5] + m1[8]*m2[8];
}

void mul_Mat3x3_Mat3x3_d(Matrix3d result, const Matrix3d m1, const Matrix3d m2)
{
    // Row 1
    result[0] = m1[0] * m2[0] + m1[1] * m2[3] + m1[2] * m2[6];
    result[1] = m1[0] * m2[1] + m1[1] * m2[4] + m1[2] * m2[7];
    result[2] = m1[0] * m2[2] + m1[1] * m2[5] + m1[2] * m2[8];
    // Row 2
    result[3] = m1[3] * m2[0] + m1[4] * m2[3] + m1[5] * m2[6];
    result[4] = m1[3] * m2[1] + m1[4] * m2[4] + m1[5] * m2[7];
    result[5] = m1[3] * m2[2] + m1[4] * m2[5] + m1[5] * m2[8];
    // Row 3
    result[6] = m1[6] * m2[0] + m1[7] * m2[3] + m1[8] * m2[6];
    result[7] = m1[6] * m2[1] + m1[7] * m2[4] + m1[8] * m2[7];
    result[8] = m1[6] * m2[2] + m1[7] * m2[5] + m1[8] * m2[8];
}

void mul_Mat3x3_Trans_Mat3x3( Matrix3 result, const Matrix3 m1, const Matrix3 m2 )
{
    // Row 1
    result[0] = m1[0]*m2[0] + m1[3]*m2[3] + m1[6]*m2[6];
    result[1] = m1[0]*m2[1] + m1[3]*m2[4] + m1[6]*m2[7];
    result[2] = m1[0]*m2[2] + m1[3]*m2[5] + m1[6]*m2[8];
    // Row 2
    result[3] = m1[1]*m2[0] + m1[4]*m2[3] + m1[7]*m2[6];
    result[4] = m1[1]*m2[1] + m1[4]*m2[4] + m1[7]*m2[7];
    result[5] = m1[1]*m2[2] + m1[4]*m2[5] + m1[7]*m2[8];
    // Row 3
    result[6] = m1[2]*m2[0] + m1[5]*m2[3] + m1[8]*m2[6];
    result[7] = m1[2]*m2[1] + m1[5]*m2[4] + m1[8]*m2[7];
    result[8] = m1[2]*m2[2] + m1[5]*m2[5] + m1[8]*m2[8];
}

void mul_Mat3x3_Trans_Mat3x3_d(Matrix3d result, const Matrix3d m1, const Matrix3d m2)
{
    // Row 1
    result[0] = m1[0] * m2[0] + m1[3] * m2[3] + m1[6] * m2[6];
    result[1] = m1[0] * m2[1] + m1[3] * m2[4] + m1[6] * m2[7];
    result[2] = m1[0] * m2[2] + m1[3] * m2[5] + m1[6] * m2[8];
    // Row 2
    result[3] = m1[1] * m2[0] + m1[4] * m2[3] + m1[7] * m2[6];
    result[4] = m1[1] * m2[1] + m1[4] * m2[4] + m1[7] * m2[7];
    result[5] = m1[1] * m2[2] + m1[4] * m2[5] + m1[7] * m2[8];
    // Row 3
    result[6] = m1[2] * m2[0] + m1[5] * m2[3] + m1[8] * m2[6];
    result[7] = m1[2] * m2[1] + m1[5] * m2[4] + m1[8] * m2[7];
    result[8] = m1[2] * m2[2] + m1[5] * m2[5] + m1[8] * m2[8];
}

void mul_Mat3x3_Mat3x3_Trans( Matrix3 result, const Matrix3 m1, const Matrix3 m2 )
{
    // Row 1
    result[0] = m1[0]*m2[0] + m1[1]*m2[1] + m1[2]*m2[2];
    result[1] = m1[0]*m2[3] + m1[1]*m2[4] + m1[2]*m2[5];
    result[2] = m1[0]*m2[6] + m1[1]*m2[7] + m1[2]*m2[8];
    // Row 2
    result[3] = m1[3]*m2[0] + m1[4]*m2[1] + m1[5]*m2[2];
    result[4] = m1[3]*m2[3] + m1[4]*m2[4] + m1[5]*m2[5];
    result[5] = m1[3]*m2[6] + m1[4]*m2[7] + m1[5]*m2[8];
    // Row 3
    result[6] = m1[6]*m2[0] + m1[7]*m2[1] + m1[8]*m2[2];
    result[7] = m1[6]*m2[3] + m1[7]*m2[4] + m1[8]*m2[5];
    result[8] = m1[6]*m2[6] + m1[7]*m2[7] + m1[8]*m2[8];
}

void mul_Mat3x3_Mat3x3_Trans_d(Matrix3d result, const Matrix3d m1, const Matrix3d m2)
{
    // Row 1
    result[0] = m1[0] * m2[0] + m1[1] * m2[1] + m1[2] * m2[2];
    result[1] = m1[0] * m2[3] + m1[1] * m2[4] + m1[2] * m2[5];
    result[2] = m1[0] * m2[6] + m1[1] * m2[7] + m1[2] * m2[8];
    // Row 2
    result[3] = m1[3] * m2[0] + m1[4] * m2[1] + m1[5] * m2[2];
    result[4] = m1[3] * m2[3] + m1[4] * m2[4] + m1[5] * m2[5];
    result[5] = m1[3] * m2[6] + m1[4] * m2[7] + m1[5] * m2[8];
    // Row 3
    result[6] = m1[6] * m2[0] + m1[7] * m2[1] + m1[8] * m2[2];
    result[7] = m1[6] * m2[3] + m1[7] * m2[4] + m1[8] * m2[5];
    result[8] = m1[6] * m2[6] + m1[7] * m2[7] + m1[8] * m2[8];
}

void mul_Mat2x2_Vec2x1( Vector2 result, const Matrix2 m, const Vector2 v )
{
    result[0] = m[0]*v[0] + m[1]*v[1];
    result[1] = m[2]*v[0] + m[3]*v[1];
}

void mul_Mat2x2_Trans_Vec2x1( Vector2 result, const Matrix2 m, const Vector2 v )
{
    result[0] = m[0]*v[0] + m[2]*v[1];
    result[1] = m[1]*v[0] + m[3]*v[1];
}

void mul_Mat3x3_Vec3x1( Vector3 result, const Matrix3 m, const Vector3 v )
{
	result[0] = m[0]*v[0] + m[1]*v[1] + m[2]*v[2];
	result[1] = m[3]*v[0] + m[4]*v[1] + m[5]*v[2];
	result[2] = m[6]*v[0] + m[7]*v[1] + m[8]*v[2];
}

void mul_Mat3x3_Trans_Vec3x1( Vector3 result, const Matrix3 m, const Vector3 v )
{
    result[0] = m[0]*v[0] + m[3]*v[1] + m[6]*v[2];
    result[1] = m[1]*v[0] + m[4]*v[1] + m[7]*v[2];
    result[2] = m[2]*v[0] + m[5]*v[1] + m[8]*v[2];
}

void mul_Mat4x4_Vec4x1( Vector4 result, const Matrix4 m, const Vector4 v )
{
	result[0] =  m[0]*v[0] +  m[1]*v[1] +  m[2]*v[2] +  m[3]*v[3];
	result[1] =  m[4]*v[0] +  m[5]*v[1] +  m[6]*v[2] +  m[7]*v[3];
	result[2] =  m[8]*v[0] +  m[9]*v[1] + m[10]*v[2] + m[11]*v[3];
	result[3] = m[12]*v[0] + m[13]*v[1] + m[14]*v[2] + m[15]*v[3];
}

void mul_Mat4x4_Trans_Vec4x1( Vector4 result, const Matrix4 m, const Vector4 v )
{
    result[0] =  m[0]*v[0] + m[4]*v[1] +  m[8]*v[2] + m[12]*v[3];
    result[1] =  m[1]*v[0] + m[5]*v[1] +  m[9]*v[2] + m[13]*v[3];
    result[2] =  m[2]*v[0] + m[6]*v[1] + m[10]*v[2] + m[14]*v[3];
    result[3] =  m[3]*v[0] + m[7]*v[1] + m[11]*v[2] + m[15]*v[3];
}

void neg_Mat3x3(Matrix3 result, const Matrix3 m)
{
    // Row 1
    result[0] = -m[0];
    result[1] = -m[1];
    result[2] = -m[2];
    // Row 2
    result[3] = -m[3];
    result[4] = -m[4];
    result[5] = -m[5];
    // Row 3
    result[6] = -m[6];
    result[7] = -m[7];
    result[8] = -m[8];
}

void mul_Vec3x1_Vec1x3( Matrix3 result, const Vector3 v1, const  Vector3 v2 )
{
	// Row 1
	result[0] = v1[0]*v2[0];
	result[1] = v1[0]*v2[1];
	result[2] = v1[0]*v2[2];
	// Row 2
	result[3] = v1[1]*v2[0];
	result[4] = v1[1]*v2[1];
	result[5] = v1[1]*v2[2];
	// Row 3
	result[6] = v1[2]*v2[0];
	result[7] = v1[2]*v2[1];
	result[8] = v1[2]*v2[2];
}

void mul_Vec3_Vec3( Vector3 result, const Vector3 v1, const Vector3 v2 )
{
	result[0] = v1[0] * v2[0];
	result[1] = v1[1] * v2[1];
	result[2] = v1[2] * v2[2];
}

void mul_Vec4_Vec4( Vector4 result, const Vector4 v1, const Vector4 v2 )
{
	result[0] = v1[0] * v2[0];
	result[1] = v1[1] * v2[1];
	result[2] = v1[2] * v2[2];
	result[3] = v1[3] * v2[3];
}

void sqrt_Vec3( Vector3 result, const Vector3 v )
{
	result[0] = _SQRT( v[0] );
	result[1] = _SQRT( v[1] );
	result[2] = _SQRT( v[2] );
}

void sqrt_Vec4( Vector4 result, const Vector4 v )
{
	result[0] = _SQRT( v[0] );
	result[1] = _SQRT( v[1] );
	result[2] = _SQRT( v[2] );
	result[3] = _SQRT( v[3] );
}

void abs_Vec2( Vector2 result, const Vector2 v )
{
	result[0] = _FABS( v[0] );
	result[1] = _FABS( v[1] );
}

void abs_Vec2d(Vector2d result, const Vector2d v)
{
	result[0] = fabs(v[0]);
	result[1] = fabs(v[1]);
}

void abs_Vec3( Vector3 result, const Vector3 v )
{
	result[0] = _FABS( v[0] );
	result[1] = _FABS( v[1] );
	result[2] = _FABS( v[2] );	
}

void abs_Vec3d(Vector3d result, const Vector3d v)
{
	result[0] = fabs(v[0]);
	result[1] = fabs(v[1]);
	result[2] = fabs(v[2]);
}

void abs_Vec4( Vector4 result, const Vector4 v )
{
	result[0] = _FABS( v[0] );
	result[1] = _FABS( v[1] );
	result[2] = _FABS( v[2] );
	result[3] = _FABS( v[3] );
}

void abs_Vec4d(Vector4d result, const Vector4d v)
{
	result[0] = fabs(v[0]);
	result[1] = fabs(v[1]);
	result[2] = fabs(v[2]);
	result[3] = fabs(v[3]);
}

f_t dot_Vec2_Vec2(const Vector2 v1, const Vector2 v2 )
{
	return  v1[0] * v2[0] +
			v1[1] * v2[1];
}

f_t dot_Vec3_Vec3(const Vector3 v1, const Vector3 v2 )
{
	return  v1[0] * v2[0] +
	        v1[1] * v2[1] +
	        v1[2] * v2[2];
}

double dot_Vec3d_Vec3d(const Vector3d v1, const Vector3d v2)
{
    return  v1[0] * v2[0] +
            v1[1] * v2[1] +
            v1[2] * v2[2];
}

f_t dot_Vec4_Vec4(const Vector4 v1, const Vector4 v2 )
{
	return  v1[0] * v2[0] +
	        v1[1] * v2[1] +
	        v1[2] * v2[2] +
	        v1[3] * v2[3];
}

//_______________________________________________________________________________________________
//observe that cross product output cannot overwrite cross product input without destroying logic
void cross_Vec3( Vector3 result, const Vector3 v1, const Vector3 v2 )
{
    result[0] = v1[1]*v2[2] - v1[2]*v2[1];
    result[1] = v1[2]*v2[0] - v1[0]*v2[2];
    result[2] = v1[0]*v2[1] - v1[1]*v2[0]; 
}

void crossd_Vec3( Vector3d result, const Vector3 v1, const Vector3 v2 )
{
	result[0] = (double)(v1[1] * v2[2] - v1[2] * v2[1]);
	result[1] = (double)(v1[2] * v2[0] - v1[0] * v2[2]);
	result[2] = (double)(v1[0] * v2[1] - v1[1] * v2[0]);
}

void mul_Vec2_X( Vector2 result, const Vector2 v, const f_t x )
{
    result[0] = v[0]*x;
    result[1] = v[1]*x;
}

void mul_Vec2d_X( Vector2d result, const Vector2d v, const double x )
{
    result[0] = v[0]*x;
    result[1] = v[1]*x;
}

void mul_Vec3_X( Vector3 result, const Vector3 v, const f_t x )
{
	result[0] = v[0]*x;
	result[1] = v[1]*x;
	result[2] = v[2]*x;
}

void mul_Vec3d_X( Vector3d result, const Vector3d v, const double x )
{
	result[0] = v[0]*x;
	result[1] = v[1]*x;
	result[2] = v[2]*x;
}

void mul_Vec4_X( Vector4 result, const Vector4 v, const f_t x )
{
	result[0] = v[0]*x;
	result[1] = v[1]*x;
	result[2] = v[2]*x;
	result[3] = v[3]*x;
}

void mul_Vec4d_X( Vector4d result, const Vector4d v, const double x )
{
	result[0] = v[0] * x;
	result[1] = v[1] * x;
	result[2] = v[2] * x;
	result[3] = v[3] * x;
}

void div_Vec3_X( Vector3 result, const Vector3 v, const f_t x )
{
    f_t d = (f_t)1.0/x;
	result[0] = v[0]*d;
	result[1] = v[1]*d;
	result[2] = v[2]*d;
}

void div_Vec4_X( Vector4 result, const Vector4 v, const f_t x )
{
    f_t d = (f_t)1.0/x;
	result[0] = v[0]*d;
	result[1] = v[1]*d;
	result[2] = v[2]*d;
	result[3] = v[3]*d;
}
void div_Vec4d_X( Vector4d result, const Vector4d v, const double x )
{
	double d = 1.0 / x;
	result[0] = v[0] * d;
	result[1] = v[1] * d;
	result[2] = v[2] * d;
	result[3] = v[3] * d;
}

void add_Vec3_Vec3( Vector3 result, const Vector3 v1, const Vector3 v2 )
{
	result[0] = v1[0] + v2[0];
	result[1] = v1[1] + v2[1];
	result[2] = v1[2] + v2[2];
}

void add_Vec3d_Vec3d( Vector3d result, const Vector3d v1, const Vector3d v2 )
{
    result[0] = v1[0] + v2[0];
    result[1] = v1[1] + v2[1];
    result[2] = v1[2] + v2[2];
}

void add_K1Vec3_K2Vec3(Vector3 result, const Vector3 v1, const Vector3 v2, float k1, float k2)
{
    result[0] = k1 * v1[0] + k2 * v2[0];
    result[1] = k1 * v1[1] + k2 * v2[1];
    result[2] = k1 * v1[2] + k2 * v2[2];
}

void add_Vec4_Vec4( Vector4 result, const Vector4 v1, const Vector4 v2 )
{
	result[0] = v1[0] + v2[0];
	result[1] = v1[1] + v2[1];
	result[2] = v1[2] + v2[2];
	result[3] = v1[3] + v2[3];
}

void add_Vec4d_Vec4d( Vector4d result, const Vector4d v1, const Vector4d v2 )
{
	result[0] = v1[0] + v2[0];
	result[1] = v1[1] + v2[1];
	result[2] = v1[2] + v2[2];
	result[3] = v1[3] + v2[3];
}

void sub_Vec3_Vec3( Vector3 result, const Vector3 v1, const Vector3 v2 )
{
	result[0] = v1[0] - v2[0];
	result[1] = v1[1] - v2[1];
	result[2] = v1[2] - v2[2];
}

void sub_Vec3d_Vec3d( Vector3d result, const Vector3d v1, const Vector3d v2 )
{
    result[0] = v1[0] - v2[0];
    result[1] = v1[1] - v2[1];
    result[2] = v1[2] - v2[2];
}

void sub_Vec4_Vec4( Vector4 result, const Vector4 v1, const Vector4 v2 )
{
	result[0] = v1[0] - v2[0];
	result[1] = v1[1] - v2[1];
	result[2] = v1[2] - v2[2];
	result[3] = v1[3] - v2[3];
}

void div_Vec3_Vec3( Vector3 result, const Vector3 v1, const Vector3 v2 )
{
	result[0] = v1[0] / v2[0];
	result[1] = v1[1] / v2[1];
	result[2] = v1[2] / v2[2];
}

void div_Vec4_Vec4( Vector4 result, const Vector4 v1, const Vector4 v2 )
{
	result[0] = v1[0] / v2[0];
	result[1] = v1[1] / v2[1];
	result[2] = v1[2] / v2[2];
	result[3] = v1[3] / v2[3];
}

void neg_Vec3(Vector3 result, const Vector3 v)
{
    result[0] = -v[0];
    result[1] = -v[1];
    result[2] = -v[2];
}

void cpy_MatRxC_MatMxN( f_t *result, i_t r, i_t c, i_t r_offset, i_t c_offset, f_t *A, i_t m, i_t n )
{
	// Ensure source matrix A fits within result matrix
	if( (m + r_offset) > r || (n + c_offset) > c )
		return;

	// Set result pointer to first location
	result += c*r_offset + c_offset;

	int rowSize = sizeof( f_t )*n;
	for( int mi=0; mi<m; mi++ )
	{
		// Copy row
		memcpy( result, A, rowSize );

		// Update to next row
		A		+= n;
		result	+= c;
	}
}


void transpose_Mat2( Matrix2 result, const Matrix2 m )
{
    // Row 1
    result[0] = m[0];
    result[1] = m[2];
    // Row 2
    result[2] = m[1];
    result[3] = m[3];
}

void transpose_Mat3( Matrix3 result, const Matrix3 m )
{
	// Row 1
	result[0] = m[0];
	result[1] = m[3];
	result[2] = m[6];
	// Row 2
	result[3] = m[1];
	result[4] = m[4];
	result[5] = m[7];
	// Row 3
	result[6] = m[2];
	result[7] = m[5];
	result[8] = m[8];
}

void transpose_Mat4( Matrix4 result, const Matrix4 m )
{
	// Row 1
	result[ 0] = m[ 0];
	result[ 1] = m[ 4];
	result[ 2] = m[ 8];
	result[ 3] = m[12];
	// Row 2
	result[ 4] = m[ 1];
	result[ 5] = m[ 5];
	result[ 6] = m[ 9];
	result[ 7] = m[13];
	// Row 3
	result[ 8] = m[ 2];
	result[ 9] = m[ 6];
	result[10] = m[10];
	result[11] = m[14];
	// Row 4
	result[12] = m[ 3];
	result[13] = m[ 7];
	result[14] = m[11];
	result[15] = m[15];
}

char inv_Mat2(Matrix2 result, Matrix2 m)
{
    f_t invDet, det = m[0] * m[3] - m[1] * m[2];

    if( det!=0.0f )
        invDet = 1.0f/det;
    else
        return -1;

	// Row 1
    result[0] = m[3] * ( invDet);
	result[1] = m[1] * (-invDet);
	// Row 2
	result[2] = m[2] * (-invDet);
	result[3] = m[0] * ( invDet);

    return 1;
}

char inv_Mat3( Matrix3 result, const Matrix3 m )
{
	// 	| m[0] m[1] m[2] |-1             |   m[8]m[4]-m[7]m[5]  -(m[8]m[1]-m[7]m[2])   m[5]m[1]-m[4]m[2]  |
	// 	| m[3] m[4] m[5] |    =  1/det * | -(m[8]m[3]-m[6]m[5])   m[8]m[0]-m[6]m[2]  -(m[5]m[0]-m[3]m[2]) |
	// 	| m[6] m[7] m[8] |               |   m[7]m[3]-m[6]m[4]  -(m[7]m[0]-m[6]m[1])   m[4]m[0]-m[3]m[1]  |
	
	f_t invDet, det = m[0]*(m[8]*m[4]-m[7]*m[5]) - m[3]*(m[8]*m[1]-m[7]*m[2]) + m[6]*(m[5]*m[1]-m[4]*m[2]);

	if( det!=0 )
		invDet = 1/det;
	else
		return -1;
		
	// Row 1
	result[0] = invDet * (m[8]*m[4]-m[7]*m[5]);
	result[1] = invDet * (m[7]*m[2]-m[8]*m[1]);  
	result[2] = invDet * (m[5]*m[1]-m[4]*m[2]);
	// Row 2
	result[3] = invDet * (m[6]*m[5]-m[8]*m[3]);
	result[4] = invDet * (m[8]*m[0]-m[6]*m[2]); 
	result[5] = invDet * (m[3]*m[2]-m[5]*m[0]);
	// Row 3
	result[6] = invDet * (m[7]*m[3]-m[6]*m[4]);
	result[7] = invDet * (m[6]*m[1]-m[7]*m[0]);
	result[8] = invDet * (m[4]*m[0]-m[3]*m[1]);
	
	return 0;
}

char inv_Mat4( Matrix4 result, const Matrix4 m )
{
    f_t inv[16], det;
    int i;

    inv[0] = 
        m[5]  * m[10] * m[15] - 
        m[5]  * m[11] * m[14] - 
        m[9]  * m[6]  * m[15] + 
        m[9]  * m[7]  * m[14] +
        m[13] * m[6]  * m[11] - 
        m[13] * m[7]  * m[10];

    inv[4] = 
       -m[4]  * m[10] * m[15] + 
        m[4]  * m[11] * m[14] + 
        m[8]  * m[6]  * m[15] - 
        m[8]  * m[7]  * m[14] - 
        m[12] * m[6]  * m[11] + 
        m[12] * m[7]  * m[10];

    inv[8] = 
        m[4]  * m[9] * m[15] - 
        m[4]  * m[11] * m[13] - 
        m[8]  * m[5] * m[15] + 
        m[8]  * m[7] * m[13] + 
        m[12] * m[5] * m[11] - 
        m[12] * m[7] * m[9];

    inv[12] = 
       -m[4]  * m[9] * m[14] + 
        m[4]  * m[10] * m[13] +
        m[8]  * m[5] * m[14] - 
        m[8]  * m[6] * m[13] - 
        m[12] * m[5] * m[10] + 
        m[12] * m[6] * m[9];

    inv[1] = 
       -m[1]  * m[10] * m[15] + 
        m[1]  * m[11] * m[14] + 
        m[9]  * m[2] * m[15] - 
        m[9]  * m[3] * m[14] - 
        m[13] * m[2] * m[11] + 
        m[13] * m[3] * m[10];

    inv[5] = 
        m[0]  * m[10] * m[15] - 
        m[0]  * m[11] * m[14] - 
        m[8]  * m[2] * m[15] + 
        m[8]  * m[3] * m[14] + 
        m[12] * m[2] * m[11] - 
        m[12] * m[3] * m[10];

    inv[9] = 
       -m[0]  * m[9] * m[15] + 
        m[0]  * m[11] * m[13] + 
        m[8]  * m[1] * m[15] - 
        m[8]  * m[3] * m[13] - 
        m[12] * m[1] * m[11] + 
        m[12] * m[3] * m[9];

    inv[13] = 
        m[0]  * m[9] * m[14] - 
        m[0]  * m[10] * m[13] - 
        m[8]  * m[1] * m[14] + 
        m[8]  * m[2] * m[13] + 
        m[12] * m[1] * m[10] - 
        m[12] * m[2] * m[9];

    inv[2] = 
        m[1]  * m[6] * m[15] - 
        m[1]  * m[7] * m[14] - 
        m[5]  * m[2] * m[15] + 
        m[5]  * m[3] * m[14] + 
        m[13] * m[2] * m[7] - 
        m[13] * m[3] * m[6];

    inv[6] = 
       -m[0]  * m[6] * m[15] + 
        m[0]  * m[7] * m[14] + 
        m[4]  * m[2] * m[15] - 
        m[4]  * m[3] * m[14] - 
        m[12] * m[2] * m[7] + 
        m[12] * m[3] * m[6];

    inv[10] = 
        m[0]  * m[5] * m[15] - 
        m[0]  * m[7] * m[13] - 
        m[4]  * m[1] * m[15] + 
        m[4]  * m[3] * m[13] + 
        m[12] * m[1] * m[7] - 
        m[12] * m[3] * m[5];

    inv[14] = 
       -m[0]  * m[5] * m[14] + 
        m[0]  * m[6] * m[13] + 
        m[4]  * m[1] * m[14] - 
        m[4]  * m[2] * m[13] - 
        m[12] * m[1] * m[6] + 
        m[12] * m[2] * m[5];

    inv[3] = 
       -m[1] * m[6] * m[11] + 
        m[1] * m[7] * m[10] + 
        m[5] * m[2] * m[11] - 
        m[5] * m[3] * m[10] - 
        m[9] * m[2] * m[7] + 
        m[9] * m[3] * m[6];

    inv[7] = 
        m[0] * m[6] * m[11] - 
        m[0] * m[7] * m[10] - 
        m[4] * m[2] * m[11] + 
        m[4] * m[3] * m[10] + 
        m[8] * m[2] * m[7] - 
        m[8] * m[3] * m[6];

    inv[11] = 
       -m[0] * m[5] * m[11] + 
        m[0] * m[7] * m[9] + 
        m[4] * m[1] * m[11] - 
        m[4] * m[3] * m[9] - 
        m[8] * m[1] * m[7] + 
        m[8] * m[3] * m[5];

    inv[15] = 
        m[0] * m[5] * m[10] - 
        m[0] * m[6] * m[9] - 
        m[4] * m[1] * m[10] + 
        m[4] * m[2] * m[9] + 
        m[8] * m[1] * m[6] - 
        m[8] * m[2] * m[5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if (det == 0)
        return -1;

    det = (f_t)1.0 / det;

    for (i = 0; i < 16; i++)
        result[i] = inv[i] * det;

    return 0;
}

// Initialize Alpha Filter alpha and beta values
void LPFO0_init_Vec3( sLpfO0 *lpf, f_t dt, f_t cornerFreqHz, const Vector3 initVal )
{
    f_t dc;

    memset( lpf, 0, sizeof(sLpfO0) );
    cpy_Vec3_Vec3( lpf->v, initVal );

    dc = dt * cornerFreqHz;
    lpf->alpha  = dc / (1.0f + dc);
    lpf->beta   = 1.0f - lpf->alpha;
}

// Low-Pass Alpha Filter
void LPFO0_Vec3( sLpfO0 *lpf, const Vector3 input )
{
    // v[n+1] = beta*v[n] + alpha*input
	O0_LPF_Vec3( lpf->v, input, lpf->alpha, lpf->beta );
}



