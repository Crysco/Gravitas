#ifndef GRAVITAS_GMATH_H_
#define GRAVITAS_GMATH_H_

#include <math.h>
#include "gPrecision.h"

namespace Gravitas
{
	class gVector3
	{
	public:
		real x;
		real y;
		real z;

	public:
		gVector3() : x(0), y(0), z(0) {}
		gVector3(real x, real y, real z) : x(x), y(y), z(z) {}

		real operator[](unsigned i) const
		{
			if (i == 0) return x;
			if (i == 1) return y;
			return z;
		}

		real& operator[](unsigned i)
		{
			if (i == 0) return x;
			if (i == 1) return y;
			return z;
		}

		//test equality
		bool operator == (const gVector3& vec)
		{
			return (x == vec.x && y == vec.y && z == vec.z);
		}

		//test inequality
		bool operator != (const gVector3& vec)
		{
			return (x != vec.x || y != vec.y || z != vec.z);
		}

		//add two vectors
		gVector3 operator + (const gVector3& vec) const
		{
			return gVector3(x + vec.x, y + vec.y, z + vec.z);
		}

		//update vector with addition
		void operator += (const gVector3& vec)
		{
			x += vec.x;
			y += vec.y;
			z += vec.z;
		}

		//subract two vectors
		gVector3 operator - (const gVector3& vec) const
		{
			return gVector3(x - vec.x, y - vec.y, z - vec.z);
		}

		//update vector with subtraction
		void operator -= (const gVector3& vec)
		{
			x -= vec.x;
			y -= vec.y;
			z -= vec.z;
		}

		//multiply a vector by a scalar
		gVector3 operator * (const real scalar) const
		{
			return gVector3(x * scalar, y * scalar, z * scalar);
		}

		//update vector with multiplied scalar
		void operator *= (const real scalar)
		{
			x *= scalar;
			y *= scalar;
			z *= scalar;
		}

		/**
		* Calculates and returns a component-wise product of this
		* vector with the given vector.
		*/
		gVector3 componentProduct(const gVector3& vec) const
		{
			return gVector3(x * vec.x, y * vec.y, z * vec.z);
		}

		/**
		* Performs a component-wise product with the given vector and
		* sets this vector to its result.
		*/
		void componentProductUpdate(const gVector3& vec)
		{
			x *= vec.x;
			y *= vec.y;
			z *= vec.z;
		}


		gVector3 operator / (const real scalar) const
		{
			return gVector3(x / scalar, y / scalar, z / scalar);
		}

		void operator /= (const real scalar)
		{
			x /= scalar;
			y /= scalar;
			z /= scalar;
		}

		const gVector3 operator -() const
		{
			return gVector3(-x, -y, -z);
		}

		//perform the dot product between two vectors
		real operator * (const gVector3& vec) const
		{
			return x * vec.x + y * vec.y + z * vec.z;
		}

		//another method to perform the dot product 
		real dotProduct(const gVector3& vec_1, const gVector3& vec_2)
		{
			return vec_1.x * vec_2.x + vec_1.y * vec_2.y + vec_1.z * vec_2.z;
		}

		//perform the cross product between two vectors
		gVector3 operator % (const gVector3& vec) const
		{
			return gVector3(y * vec.z - vec.y * z, z * vec.x - vec.z * x, x * vec.y - vec.x * y);
		}

		//another method to perform the cross product
		gVector3 crossProduct(const gVector3& vec_1, const gVector3& vec_2)
		{
			return gVector3(vec_1.y * vec_2.z - vec_2.y * vec_1.z, vec_1.z * vec_2.x - vec_2.z * vec_1.x, vec_1.x * vec_2.y - vec_2.x * vec_1.y);
		}

		//calculate the magnitude of vector
		real magnitude() const
		{
			return sqrtf(x * x + y * y + z * z);
		}

		//calculate the squared magnitude of vector
		real squareMagnitude() const
		{
			return x * x + y * y + z * z;
		}

		//calculate the normalized form of vector
		gVector3 normalize()
		{
			real length = magnitude();
			*this *= 1 / length;
			return *this;
		}

		gVector3 normalized() const
		{
			real length = magnitude();
			gVector3 n_vector = *this;
			n_vector *= 1 / length;
			return n_vector;
		}

		//calculate the angle in radians between two vectors
		real angle_rad(const gVector3& vec_1, const gVector3& vec_2)
		{
			return acosf(dotProduct(vec_1, vec_2) / (vec_1.magnitude() * vec_2.magnitude()));
		}

		//calculate the angle in degrees between two vectors
		real angle_deg(const gVector3& vec_1, const gVector3& vec_2)
		{
			return acosf(dotProduct(vec_1, vec_2) / (vec_1.magnitude() * vec_2.magnitude())) * (180 / 3.1415926);
		}

		void clear()
		{
			x = 0;
			y = 0;
			z = 0;
		}
	};

	class gQuaternion
	{
	public:
		real w;
		real x;
		real y;
		real z;

	public:
		gQuaternion() : w(1), x(0), y(0), z(0) {}
		gQuaternion(real w, real x, real y, real z) : w(w), x(0), y(0), z(0) {}

		//converts three Euler angles to a quaternion
		const gQuaternion convertFromEuler(real ang_x, real ang_y, real ang_z)
		{
			w = cos(ang_x / 2) * cos(ang_y / 2) * cos(ang_z / 2) + sin(ang_x / 2) * sin(ang_y / 2) * sin(ang_z / 2);
			x = sin(ang_x / 2) * cos(ang_y / 2) * cos(ang_z / 2) - cos(ang_x / 2) * sin(ang_y / 2) * sin(ang_z / 2);
			y = cos(ang_x / 2) * sin(ang_y / 2) * cos(ang_z / 2) + sin(ang_x / 2) * cos(ang_y / 2) * sin(ang_z / 2);
			z = cos(ang_x / 2) * cos(ang_y / 2) * sin(ang_z / 2) - sin(ang_x / 2) * sin(ang_y / 2) * cos(ang_z / 2);

			return *this;
		}

		//converts a vector of Euler angles to a quaternion
		const gQuaternion convertFromEuler(const gVector3& angles)
		{
			w = cos(angles.x / 2) * cos(angles.y / 2) * cos(angles.z / 2) + sin(angles.x / 2) * sin(angles.y / 2) * sin(angles.z / 2);
			x = sin(angles.x / 2) * cos(angles.y / 2) * cos(angles.z / 2) - cos(angles.x / 2) * sin(angles.y / 2) * sin(angles.z / 2);
			y = cos(angles.x / 2) * sin(angles.y / 2) * cos(angles.z / 2) + sin(angles.x / 2) * cos(angles.y / 2) * sin(angles.z / 2);
			z = cos(angles.x / 2) * cos(angles.y / 2) * sin(angles.z / 2) - sin(angles.x / 2) * sin(angles.y / 2) * cos(angles.z / 2);

			return *this;
		}

		//test equality
		bool operator == (const gQuaternion& quat)
		{
			return (w == quat.w && x == quat.x && y == quat.y && z == quat.z);
		}

		//test inequality
		bool operator != (const gQuaternion& quat)
		{
			return (w != quat.w || x != quat.x || y != quat.y || z != quat.z);
		}

		//add two quaternions
		const gQuaternion operator + (const gQuaternion& quat) const
		{
			return gQuaternion(w + quat.w, x + quat.x, y + quat.y, z + quat.z);
		}

		//update quaternions with addition
		void operator += (const gQuaternion& quat)
		{
			w += quat.w;
			x += quat.x;
			y += quat.y;
			z += quat.z;
		}

		void operator += (const gVector3& vec)
		{
			gQuaternion q(0, vec.x, vec.y, vec.z);
			q *= *this;
			w += q.w * ((real)0.5);
			x += q.x * ((real)0.5);
			y += q.y * ((real)0.5);
			z += q.z * ((real)0.5);
		}

		//subract two quaternions
		const gQuaternion operator - (const gQuaternion& quat) const
		{
			return gQuaternion(w - quat.w, x - quat.x, y - quat.y, z - quat.z);
		}

		//update quaternion with subtraction
		void operator -= (const gQuaternion& quat)
		{
			w -= quat.w;
			x -= quat.x;
			y -= quat.y;
			z -= quat.z;
		}

		//multiply a quaternion by a scalar
		const gQuaternion operator * (const real scalar) const
		{
			return gQuaternion(w * scalar, x * scalar, y * scalar, z * scalar);
		}

		//update quaternion with multiplied scalar
		void operator *= (const real scalar)
		{
			w *= scalar;
			x *= scalar;
			y *= scalar;
			z *= scalar;
		}

		//divide a quaternion by a scalar
		const gQuaternion operator / (const real scalar) const
		{
			return gQuaternion(w / scalar, x / scalar, y / scalar, z / scalar);
		}

		//update quaternion with divided scalar 
		void operator /= (const real scalar)
		{
			w /= scalar;
			x /= scalar;
			y /= scalar;
			z /= scalar;
		}

		//negates a quaternion
		const gQuaternion operator -() const
		{
			return *this * (real)-1.0;
		}

		//multiply two quaternions
		const gQuaternion operator * (const gQuaternion& quat) const
		{
			return gQuaternion(
				w * quat.w - x * quat.x - y * quat.y - z * quat.z,
				y * quat.z + w * quat.x + x * quat.w - z * quat.y,
				z * quat.x + w * quat.y + y * quat.w - x * quat.z,
				x * quat.y + w * quat.z + z * quat.w - y * quat.x
				);
		}

		void  operator *= (const gQuaternion& quat)
		{
			gQuaternion q = *this;

			w = q.w * quat.w - q.x * quat.x - q.y * quat.y - q.z * quat.z;
			x = q.w * quat.x + q.x * quat.w + q.y * quat.z - q.z * quat.y;
			y = q.w * quat.y + q.y * quat.w + q.z * quat.x - q.x * quat.z;
			z = q.w * quat.z + q.z * quat.w + q.x * quat.y - q.y * quat.x;
		}

		//calculate the "difference" between two quaternions
		gQuaternion difference(const gQuaternion& quat_1, const gQuaternion& quat_2)
		{
			gQuaternion temp = quat_1;
			temp.invert();
			return quat_2 * temp;
		}

		//perform the dot product between two quaternions
		real operator ^ (const gQuaternion& quat)
		{
			return w * quat.w + x * quat.x + y * quat.y + z * quat.z;
		}

		//another method to perform the dot product
		static inline real dotProduct(const gQuaternion& quat_1, const gQuaternion& quat_2)
		{
			return quat_1.w * quat_2.w + quat_1.x * quat_2.x + quat_1.y * quat_2.y + quat_1.z * quat_2.z;
		}

		//calculate the magnitude of quattor
		real magnitude() const
		{
			return (real)sqrtf(w * w + x * x + y * y + z * z);
		}

		//calculate the normalized form of quattor
		void normalize()
		{
			*this /= magnitude();
		}

		//returns the normalized version of this quaternion
		gQuaternion normalized() const
		{
			return  *this / magnitude();
		}

		//negate quaternion
		void negate()
		{
			*this *= (real)-1.0;
		}

		//calculate the conjugate of quaternion
		void conjugate()
		{
			x = -x;
			y = -y;
			z = -z;
		}

		//calculate the inverse of quaternion
		void invert()
		{
			conjugate();
			*this /= magnitude();
		}

		//calculate the logarithm of quaternion
		gQuaternion log() const
		{
			real a = acosf(w);
			real sina = sinf(a);
			gQuaternion ret;

			ret.w = 0;
			if (sina > 0)
			{
				ret.x = a * x / sina;
				ret.y = a * y / sina;
				ret.z = a * z / sina;
			}
			else
			{
				ret.x = ret.y = ret.z = 0;
			}
			return ret;
		}

		//calculate the scalar exponent of quaternion
		gQuaternion exp() const
		{
			real a = (real)sqrt(x * x + y * y + z * z);
			real sina = (real)sin(a);
			real cosa = (real)cos(a);
			gQuaternion ret;

			ret.w = cosa;
			if (a > 0)
			{
				ret.x = sina * x / a;
				ret.y = sina * y / a;
				ret.z = sina * z / a;
			}
			else
			{
				ret.x = ret.y = ret.z = 0;
			}
			return ret;
		}

		//perform linear interpolation between two quaternions
		static gQuaternion lerp(const gQuaternion& quat_1, const gQuaternion& quat_2, real t)
		{
			return (quat_1 * (1 - t) + quat_2 * t).normalized();
		}

		//perform spherical linear interpolation between two quaternions
		static gQuaternion slerp(const gQuaternion& quat_1, const gQuaternion& quat_2, real t)
		{
			gQuaternion quat_3;
			real dot = dotProduct(quat_1, quat_2);

			if (dot < 0)
			{
				dot = -dot;
				quat_3 = quat_2 * (real)-1.0;
			}
			else
			{
				quat_3 = quat_2;
			}

			real omega = acosf(dot);
			real k0 = sinf(1 - t) * omega / sinf(omega);
			real k1 = sinf(t * omega) / sinf(omega);

			return quat_1 * k0 + quat_3 * k1;
		}


	};

	class gMatrix3
	{
	public:
		/*
		* Holds the tensor matrix data in array form.
		*/
		real data[9];

		/**
		* Creates a new matrix.
		*/
		gMatrix3()
		{
			data[0] = data[1] = data[2] = data[3] = data[4] = data[5] = data[6] = data[7] = data[8] = 0;
		}

		/**
		* Creates a new matrix with the given three vectors making up its columns.
		*/
		gMatrix3(const gVector3& comp_1, const gVector3& comp_2, const gVector3& comp_3)
		{
			setComponents(comp_1, comp_2, comp_3);
		}

		/**
		* Creates a new matrix with explicit coefficients.
		*/
		gMatrix3(real x_1, real y_1, real z_1, real x_2, real y_2, real z_2, real x_3, real y_3, real z_3)
		{
			data[0] = x_1; data[1] = y_1; data[2] = z_1;
			data[3] = x_2; data[4] = y_2; data[5] = z_2;
			data[6] = x_3; data[7] = y_3; data[8] = z_3;
		}

		/**
		* Sets the matrix to be a diagonal matrix with the given
		* values along the leading diagonal.
		*/
		void setDiagonal(real a, real b, real c)
		{
			setInertiaTensorCoeffs(a, b, c);
		}

		/**
		* Sets the value of the matrix from inertia tensor values.
		*/
		void setInertiaTensorCoeffs(real ix, real iy, real iz,
			real ixy = 0, real ixz = 0, real iyz = 0)
		{
			data[0] = ix;
			data[1] = data[3] = -ixy;
			data[2] = data[6] = -ixz;
			data[4] = iy;
			data[5] = data[7] = -iyz;
			data[8] = iz;
		}

		/**
		* Sets the value of the matrix as an inertia tensor of
		* a rectangular block aligned with the body's coordinate
		* system with the given axis half-sizes and mass.
		*/
		void setBlockInertiaTensor(const gVector3 &halfSizes, real mass)
		{
			gVector3 squares = halfSizes.componentProduct(halfSizes);
			setInertiaTensorCoeffs(0.3f * mass * (squares.y + squares.z), 0.3f * mass * (squares.x + squares.z), 0.3f * mass * (squares.x + squares.y));
		}

		void setSkewSymmetric(const gVector3 vector)
		{
			data[0] = data[4] = data[8] = 0;
			data[1] = -vector.z;
			data[2] = vector.y;
			data[3] = vector.z;
			data[5] = -vector.x;
			data[6] = -vector.y;
			data[7] = vector.x;
		}

		//multiplies two matricies
		gMatrix3 operator * (const gMatrix3 &mat) const
		{
			return gMatrix3(
				data[0] * mat.data[0] + data[1] * mat.data[3] + data[2] * mat.data[6],
				data[0] * mat.data[1] + data[1] * mat.data[4] + data[2] * mat.data[7],
				data[0] * mat.data[2] + data[1] * mat.data[5] + data[2] * mat.data[8],

				data[3] * mat.data[0] + data[4] * mat.data[3] + data[5] * mat.data[6],
				data[3] * mat.data[1] + data[4] * mat.data[4] + data[5] * mat.data[7],
				data[3] * mat.data[2] + data[4] * mat.data[5] + data[5] * mat.data[8],

				data[6] * mat.data[0] + data[7] * mat.data[3] + data[8] * mat.data[6],
				data[6] * mat.data[1] + data[7] * mat.data[4] + data[8] * mat.data[7],
				data[6] * mat.data[2] + data[7] * mat.data[5] + data[8] * mat.data[8]
				);
		}

		//multiplies this matrix in place by the given other matrix
		void operator *= (const gMatrix3 &mat)
		{
			real t1;
			real t2;
			real t3;

			t1 = data[0] * mat.data[0] + data[1] * mat.data[3] + data[2] * mat.data[6];
			t2 = data[0] * mat.data[1] + data[1] * mat.data[4] + data[2] * mat.data[7];
			t3 = data[0] * mat.data[2] + data[1] * mat.data[5] + data[2] * mat.data[8];
			data[0] = t1;
			data[1] = t2;
			data[2] = t3;

			t1 = data[3] * mat.data[0] + data[4] * mat.data[3] + data[5] * mat.data[6];
			t2 = data[3] * mat.data[1] + data[4] * mat.data[4] + data[5] * mat.data[7];
			t3 = data[3] * mat.data[2] + data[4] * mat.data[5] + data[5] * mat.data[8];
			data[3] = t1;
			data[4] = t2;
			data[5] = t3;

			t1 = data[6] * mat.data[0] + data[7] * mat.data[3] + data[8] * mat.data[6];
			t2 = data[6] * mat.data[1] + data[7] * mat.data[4] + data[8] * mat.data[7];
			t3 = data[6] * mat.data[2] + data[7] * mat.data[5] + data[8] * mat.data[8];
			data[6] = t1;
			data[7] = t2;
			data[8] = t3;
		}

		//multiplies this matrix in place by the given scalar.
		void operator *= (const real scalar)
		{
			data[0] *= scalar; data[1] *= scalar; data[2] *= scalar;
			data[3] *= scalar; data[4] *= scalar; data[5] *= scalar;
			data[6] *= scalar; data[7] *= scalar; data[8] *= scalar;
		}

		//does a component-wise addition of this matrix and the given matrix.
		void operator += (const gMatrix3 &mat)
		{
			data[0] += mat.data[0]; data[1] += mat.data[1]; data[2] += mat.data[2];
			data[3] += mat.data[3]; data[4] += mat.data[4]; data[5] += mat.data[5];
			data[6] += mat.data[6]; data[7] += mat.data[7]; data[8] += mat.data[8];
		}

		//sets the matrix to the transpose of another
		void setTranspose(const gMatrix3& mat)
		{
			data[1] = mat.data[3]; data[2] = mat.data[6]; data[3] = mat.data[1];
			data[5] = mat.data[7]; data[6] = mat.data[3]; data[7] = mat.data[5];
		}

		/** Returns a new matrix containing the transpose of this matrix. */
		gMatrix3 transpose() const
		{
			gMatrix3 result;
			result.setTranspose(*this);
			return result;
		}

		gVector3 transform(const gVector3& vec) const
		{
			return gVector3(
				vec.x * data[0] + vec.y * data[1] + vec.z * data[2],
				vec.x * data[3] + vec.y * data[4] + vec.z * data[5],
				vec.x * data[6] + vec.y * data[7] + vec.z * data[8]
				);
		}

		/** Transform the given vector by the transpose of this matrix. */
		gVector3 transformTranspose(const gVector3& vec) const
		{
			return gVector3(
				vec.x * data[0] + vec.y * data[3] + vec.z * data[6],
				vec.x * data[1] + vec.y * data[4] + vec.z * data[7],
				vec.x * data[2] + vec.y * data[5] + vec.z * data[8]
				);
		}

		/** Sets the matrix to be the inverse of the given matrix. */
		void setInverse(const gMatrix3 &mat)
		{
			real t4 = mat.data[0] * mat.data[4];
			real t6 = mat.data[0] * mat.data[5];
			real t8 = mat.data[1] * mat.data[3];
			real t10 = mat.data[2] * mat.data[3];
			real t12 = mat.data[1] * mat.data[6];
			real t14 = mat.data[2] * mat.data[6];

			// Calculate the determatinant
			real t16 = (t4*mat.data[8] - t6*mat.data[7] - t8*mat.data[8] +
				t10*mat.data[7] + t12*mat.data[5] - t14*mat.data[4]);

			// matake sure the determatinant is non-zero.
			if (t16 == (real)0.0f) return;
			real t17 = 1 / t16;

			data[0] = (mat.data[4] * mat.data[8] - mat.data[5] * mat.data[7])*t17;
			data[1] = -(mat.data[1] * mat.data[8] - mat.data[2] * mat.data[7])*t17;
			data[2] = (mat.data[1] * mat.data[5] - mat.data[2] * mat.data[4])*t17;
			data[3] = -(mat.data[3] * mat.data[8] - mat.data[5] * mat.data[6])*t17;
			data[4] = (mat.data[0] * mat.data[8] - t14)*t17;
			data[5] = -(t6 - t10)*t17;
			data[6] = (mat.data[3] * mat.data[7] - mat.data[4] * mat.data[6])*t17;
			data[7] = -(mat.data[0] * mat.data[7] - t12)*t17;
			data[8] = (t4 - t8)*t17;
		}

		/** Returns a new matrix containing the inverse of this matrix. */
		gMatrix3 inverse() const
		{
			gMatrix3 result;
			result.setInverse(*this);
			return result;
		}

		void setComponents(const gVector3& comp_1, const gVector3& comp_2, const gVector3& comp_3)
		{
			data[0] = comp_1.x;
			data[1] = comp_2.x;
			data[2] = comp_3.x;
			data[3] = comp_1.y;
			data[4] = comp_2.y;
			data[5] = comp_3.y;
			data[6] = comp_1.z;
			data[7] = comp_2.z;
			data[8] = comp_3.z;
		}

		/**
		* Sets this matrix to be the rotation matrix corresponding to
		* the given quaternion.
		*/
		void setOrientation(const gQuaternion &quat)
		{
			data[0] = 1 - (2 * quat.y*quat.y + 2 * quat.z*quat.z);
			data[1] = 2 * quat.x*quat.y + 2 * quat.z*quat.w;
			data[2] = 2 * quat.x*quat.z - 2 * quat.y*quat.w;
			data[3] = 2 * quat.x*quat.y - 2 * quat.z*quat.w;
			data[4] = 1 - (2 * quat.x*quat.x + 2 * quat.z*quat.z);
			data[5] = 2 * quat.y*quat.z + 2 * quat.x*quat.w;
			data[6] = 2 * quat.x*quat.z + 2 * quat.y*quat.w;
			data[7] = 2 * quat.y*quat.z - 2 * quat.x*quat.w;
			data[8] = 1 - (2 * quat.x*quat.x + 2 * quat.y*quat.y);
		}

		/**
		* Interpolates a couple of matrices.
		*/
		static gMatrix3 linearInterpolate(const gMatrix3& mat_1, const gMatrix3& mat_2, real prop)
		{
			gMatrix3 result;
			for (unsigned i = 0; i < 9; i++)
			{
				result.data[i] = mat_1.data[i] * (1 - prop) + mat_2.data[i] * prop;
			}
			return result;
		}
	};

	class gMatrix4
	{
	public:
		real data[12];

		/** 
		* Creates an identity matrix.
		*/
		gMatrix4()
		{
			data[1] = data[2] = data[3] = data[4] = data[6] = data[7] = data[8] = data[9] = data[11] = 0;
			data[0] = data[5] = data[10] = 1;
		}

		/**
		* 
		*/

		/**
		* Sets the matrix to be a diagonal matrix with the given coefficients.
		*/
		void setDiagonal(real a, real b, real c)
		{
			data[0] = a;
			data[5] = b;
			data[10] = c;
		}

		/**
		* Returns a matrix which is this matrix multiplied by the given other matrix.
		*/
		gMatrix4 operator * (const gMatrix4& mat) const
		{
			gMatrix4 result;
			result.data[0] = (mat.data[0] * data[0]) + (mat.data[4] * data[1]) + (mat.data[8] * data[2]);
			result.data[4] = (mat.data[0] * data[4]) + (mat.data[4] * data[5]) + (mat.data[8] * data[6]);
			result.data[8] = (mat.data[0] * data[8]) + (mat.data[4] * data[9]) + (mat.data[8] * data[10]);

			result.data[1] = (mat.data[1] * data[0]) + (mat.data[5] * data[1]) + (mat.data[9] * data[2]);
			result.data[5] = (mat.data[1] * data[4]) + (mat.data[5] * data[5]) + (mat.data[9] * data[6]);
			result.data[9] = (mat.data[1] * data[8]) + (mat.data[5] * data[9]) + (mat.data[9] * data[10]);

			result.data[2] = (mat.data[2] * data[0]) + (mat.data[6] * data[1]) + (mat.data[10] * data[2]);
			result.data[6] = (mat.data[2] * data[4]) + (mat.data[6] * data[5]) + (mat.data[10] * data[6]);
			result.data[10] = (mat.data[2] * data[8]) + (mat.data[6] * data[9]) + (mat.data[10] * data[10]);

			result.data[3] = (mat.data[3] * data[0]) + (mat.data[7] * data[1]) + (mat.data[11] * data[2]) + data[3];
			result.data[7] = (mat.data[3] * data[4]) + (mat.data[7] * data[5]) + (mat.data[11] * data[6]) + data[7];
			result.data[11] = (mat.data[3] * data[8]) + (mat.data[7] * data[9]) + (mat.data[11] * data[10]) + data[11];

			return result;
		}

		/**
		* Transform the given vector by this matrix.
		*
		* @param vec The vector to transform.
		*/
		gVector3 operator * (const gVector3& vec) const
		{
			return gVector3(
				vec.x * data[0] + vec.y * data[1] + vec.z * data[2] + data[3],
				vec.x * data[4] + vec.y * data[5] + vec.z * data[6] + data[7],
				vec.x * data[8] + vec.y * data[9] + vec.z * data[10] + data[11]
				);
		}

		/**
		* Transform the given vector by this matrix.
		*
		* @param vec The vector to transform.
		*/
		gVector3 transform(const gVector3 &vec) const
		{
			return (*this) * vec;
		}

		/**
		* Returns the determinant of the matrix.
		*/
		real getDeterminant() const
		{
			return data[8] * data[5] * data[2] +
				data[4] * data[9] * data[2] +
				data[8] * data[1] * data[6] -
				data[0] * data[9] * data[6] -
				data[4] * data[1] * data[10] +
				data[0] * data[5] * data[10];
		}

		/**
		* Sets the matrix to be the inverse of the given matrix.
		*
		* @param mat The matrix to invert and use to set this. 
		*/
		void setInverse(const gMatrix4& mat)
		{
			// Make sure the determinant is non-zero.
			real det = getDeterminant();
			if (det == 0) return;
			det = (real)1.0 / det;

			data[0] = (-mat.data[9] * mat.data[6] + mat.data[5] * mat.data[10])*det;
			data[4] = (mat.data[8] * mat.data[6] - mat.data[4] * mat.data[10])*det;
			data[8] = (-mat.data[8] * mat.data[5] + mat.data[4] * mat.data[9])*det;

			data[1] = (mat.data[9] * mat.data[2] - mat.data[1] * mat.data[10])*det;
			data[5] = (-mat.data[8] * mat.data[2] + mat.data[0] * mat.data[10])*det;
			data[9] = (mat.data[8] * mat.data[1] - mat.data[0] * mat.data[9])*det;

			data[2] = (-mat.data[5] * mat.data[2] + mat.data[1] * mat.data[6])*det;
			data[6] = (+mat.data[4] * mat.data[2] - mat.data[0] * mat.data[6])*det;
			data[10] = (-mat.data[4] * mat.data[1] + mat.data[0] * mat.data[5])*det;

			data[3] = (mat.data[9] * mat.data[6] * mat.data[3]
				- mat.data[5] * mat.data[10] * mat.data[3]
				- mat.data[9] * mat.data[2] * mat.data[7]
				+ mat.data[1] * mat.data[10] * mat.data[7]
				+ mat.data[5] * mat.data[2] * mat.data[11]
				- mat.data[1] * mat.data[6] * mat.data[11])*det;
			data[7] = (-mat.data[8] * mat.data[6] * mat.data[3]
				+ mat.data[4] * mat.data[10] * mat.data[3]
				+ mat.data[8] * mat.data[2] * mat.data[7]
				- mat.data[0] * mat.data[10] * mat.data[7]
				- mat.data[4] * mat.data[2] * mat.data[11]
				+ mat.data[0] * mat.data[6] * mat.data[11])*det;
			data[11] = (mat.data[8] * mat.data[5] * mat.data[3]
				- mat.data[4] * mat.data[9] * mat.data[3]
				- mat.data[8] * mat.data[1] * mat.data[7]
				+ mat.data[0] * mat.data[9] * mat.data[7]
				+ mat.data[4] * mat.data[1] * mat.data[11]
				- mat.data[0] * mat.data[5] * mat.data[11])*det;
		}

		/**
		* Returns a new matrix containing the inverse of this matrix.
		*/
		gMatrix4 inverse() const
		{
			gMatrix4 result;
			result.setInverse(*this);
			return result;
		}

		/**
		* Inverts the matrix.
		*/
		void invert()
		{
			setInverse(*this);
		}

		/**
		* Transform the given direction vector by this matrix.
		*
		* @note When a direction is converted between frames of reference, there is no translation required.
		*
		* @param vector The vector to transform.
		*/
		gVector3 transformDirection(const gVector3& vec) const
		{
			return gVector3(
				vec.x * data[0] + vec.y * data[1] + vec.z * data[2],
				vec.x * data[4] + vec.y * data[5] + vec.z * data[6],
				vec.x * data[8] + vec.y * data[9] + vec.z * data[10]
			);
		}

		/**
		* Transform the given direction vector by the transformational inverse of this matrix.
		*
		* @note This function relies on the fact that the inverse of a pure rotation matrix is its transpose.
		* It separates the translational and rotation components, transposes the rotation, and multiplies out.
		* If the matrix is not a scale and shear free transform matrix, then this function will not give correct results.
		*
		* @note When a direction  is converted between frames of reference, there is no translation required.
		*
		* @param vec The vector to transform.
		*/
		gVector3 transformInverseDirection(const gVector3& vec) const
		{
			return gVector3(
				vec.x * data[0] + vec.y * data[4] + vec.z * data[8],
				vec.x * data[1] + vec.y * data[5] + vec.z * data[9],
				vec.x * data[2] + vec.y * data[6] + vec.z * data[10]
			);
		}

		/**
		* Transform the given vector by the transformational inverse of this matrix.
		* 
		* @note This function relies on the fact that the inverse of a pure rotational matrix is its transpose.
		* It separates the translational and rotation components, transposes the rotation, and multiplies out.
		* If the matrix is not a scale and shear free transform matrix, then this function will not give correct results.
		*
		* @param vec The vector to transform.
		*/
		gVector3 transformInverse(const gVector3& vec) const
		{
			gVector3 tmp = vec;
			tmp.x -= data[3];
			tmp.y -= data[7];
			tmp.z -= data[11];

			return gVector3(
				tmp.x * data[0] + tmp.y * data[4] + tmp.z * data[8],
				tmp.x * data[1] + tmp.y * data[5] + tmp.z * data[9],
				tmp.x * data[2] + tmp.y * data[6] + tmp.z * data[10]
			);
		}

		/**
		* Gets a vector representing one axis (i.e. one column) in the matrix.
		*
		* @param i the row to return. Row 3 corresponds to the position of the transform matrix.
		*
		* @return The vector.
		*/
		gVector3 getAxisVector(int i) const
		{
			return gVector3(data[i], data[i + 4], data[i + 8]);
		}

		/**
		* Sets this matrix to be the rotation matrix corresponding to the given quaternion.
		*/
		void setOrientationAndPos(const gQuaternion& quat, const gVector3& pos)
		{
			data[0] = 1 - (2 * quat.y * quat.y + 2 * quat.z * quat.z);
			data[1] = 2 * quat.x * quat.y + 2 * quat.z * quat.w;
			data[2] = 2 * quat.x*quat.z - 2 * quat.y*quat.w;
			data[3] = pos.x;

			data[4] = 2 * quat.x * quat.y - 2 * quat.z * quat.w;
			data[5] = 1 - (2 * quat.x * quat.x + 2 * quat.z * quat.z);
			data[6] = 2 * quat.y * quat.z + 2 * quat.x * quat.w;
			data[7] = pos.y;

			data[8] = 2 * quat.x * quat.z + 2 * quat.y * quat.w;
			data[9] = 2 * quat.y * quat.z - 2 * quat.x * quat.w;
			data[10] = 1 - (2 * quat.x * quat.x + 2 * quat.y * quat.y);
			data[11] = pos.z;
		}

		/**
		* Fills the given array with this transform matrix, so it is usable as an open-gl transform matrix.
		* OpenGL uses a column major format, so that the values are transposed as they are written.
		*/
		void fillGLArray(real array[16]) const
		{
			array[0] = (real)data[0];
			array[1] = (real)data[4];
			array[2] = (real)data[8];
			array[3] = (real)0;

			array[4] = (real)data[1];
			array[5] = (real)data[5];
			array[6] = (real)data[9];
			array[7] = (real)0;

			array[8] = (real)data[2];
			array[9] = (real)data[6];
			array[10] = (real)data[10];
			array[11] = (real)0;

			array[12] = (real)data[3];
			array[13] = (real)data[7];
			array[14] = (real)data[11];
			array[15] = (real)1;
		}


	};
}

#endif
