#ifndef __DualQuaternion_H__
#define __DualQuaternion_H__

#include "OgrePrerequisites.h"
#include "OgreMath.h"
#include "OgreQuaternion.h"

namespace Ogre {

	/** Implementation of a dual Quaternion, the 4-Sphere complex vector
	*/
	class _OgreExport DualQuaternion
	{
	public: Quaternion Ordinary;  
			Quaternion Dual;       
	public:
		/// <summary>
		/// Converts a rotation and translation into a DualQuaternion.
		/// </summary>
		/// <param name="q0">Unit rotation quaternion.</param>
		/// <param name="t">Translation vector</param>
		/// <returns>A special dual quaternion that can be linearly blended with other dual quaternions with minimal error.</returns>
		static DualQuaternion QuatTrans2UDQ(Quaternion q0, Vector3 t)
		{
			DualQuaternion dq = new DualQuaternion();
			// non-dual part (just copy q0):
			dq.Ordinary = q0;
			// dual part:            
			dq.Dual.W = -0.5f * (t.X * q0.X + t.Y * q0.Y + t.Z * q0.Z);
			dq.Dual.X = 0.5f * (t.X * q0.W + t.Y * q0.Z - t.Z * q0.Y);
			dq.Dual.Y = 0.5f * (-t.X * q0.Z + t.Y * q0.W + t.Z * q0.X);
			dq.Dual.Z = 0.5f * (t.X * q0.Y - t.Y * q0.X + t.Z * q0.W);
			return dq;
		}

		/// <summary>
		/// Converts a DualQuaternion created using QuatTrans2UDQ back into a matrix. This code is currently not being used by the program.
		/// </summary>
		/// <param name="dq">A DualQuaternion created with QuatTrans2UDQ()</param>
		/// <returns>The matrix represented by dq.</returns>
		static Matrix UDQToMatrix(DualQuaternion dq)
		{
			Matrix M;
			float len2 = Quaternion.Dot(dq.Ordinary, dq.Ordinary);
			float w = dq.Ordinary.W, x = dq.Ordinary.X, y = dq.Ordinary.Y, z = dq.Ordinary.Z;
			float t0 = dq.Dual.W, t1 = dq.Dual.X, t2 = dq.Dual.Y, t3 = dq.Dual.Z;

			M.M11 = w*w + x*x - y*y - z*z;
			M.M21 = 2 * x * y - 2 * w * z;
			M.M31 = 2 * x * z + 2 * w * y;
			M.M12 = 2 * x * y + 2 * w * z;
			M.M22 = w * w + y * y - x * x - z * z;
			M.M32 = 2 * y * z - 2 * w * x;
			M.M13 = 2 * x * z - 2 * w * y;
			M.M23 = 2 * y * z + 2 * w * x;
			M.M33 = w * w + z * z - x * x - y * y;

			M.M41 = -2 * t0 * x + 2 * w * t1 - 2 * t2 * z + 2 * y * t3;
			M.M42 = -2 * t0 * y + 2 * t1 * z - 2 * x * t3 + 2 * w * t2;
			M.M43 = -2 * t0 * z + 2 * x * t2 + 2 * w * t3 - 2 * t1 * y;

			M.M14 = 0;
			M.M24 = 0;
			M.M34 = 0;
			M.M44 = len2;

			M /= len2;

			return M;	
		}
	
	
		inline DualQuaternion& operator= (const DualQuaternion& rkQ)
		{
			Ordinary = rkQ.Ordinary;
			Dual = rkQ.Dual;
			return *this;
		}
		DualQuaternion operator+ (const DualQuaternion& rkQ) const;
		DualQuaternion operator- (const DualQuaternion& rkQ) const;
		DualQuaternion operator* (const DualQuaternion& rkQ) const;
		DualQuaternion operator* (Real fScalar) const;
		_OgreExport friend Quaternion operator* (Real fScalar,
			const Quaternion& rkQ);
		Quaternion operator- () const;
		inline bool operator== (const DualQuaternion& rhs) const
		{
			return  (rhs.Ordinary == Ordinary) && (rhs.Dual == Dual);
		}
		inline bool operator!= (const DualQuaternion& rhs) const
		{
			return !operator==(rhs);
		}
		// functions of a quaternion
		Real Dot (const DualQuaternion& rkQ) const;  // dot product
		Real Norm () const;  // squared-length
		/// Normalises this quaternion, and returns the previous length
		Real normalise(void); 
		DualQuaternion Inverse () const;  // apply to non-zero quaternion
		DualQuaternion UnitInverse () const;  // apply to unit-length quaternion
		DualQuaternion Exp () const;
		DualQuaternion Log () const;

		// rotation of a vector by a quaternion
		Vector3 operator* (const Vector3& rkVector) const;

		/** Calculate the local roll element of this quaternion.
		@param reprojectAxis By default the method returns the 'intuitive' result
		that is, if you projected the local Y of the quaternion onto the X and
		Y axes, the angle between them is returned. If set to false though, the
		result is the actual yaw that will be used to implement the quaternion,
		which is the shortest possible path to get to the same orientation and 
		may involve less axial rotation. 
		*/
		Radian getRoll(bool reprojectAxis = true) const;
		/** Calculate the local pitch element of this quaternion
		@param reprojectAxis By default the method returns the 'intuitive' result
		that is, if you projected the local Z of the quaternion onto the X and
		Y axes, the angle between them is returned. If set to true though, the
		result is the actual yaw that will be used to implement the quaternion,
		which is the shortest possible path to get to the same orientation and 
		may involve less axial rotation. 
		*/
		Radian getPitch(bool reprojectAxis = true) const;
		/** Calculate the local yaw element of this quaternion
		@param reprojectAxis By default the method returns the 'intuitive' result
		that is, if you projected the local Z of the quaternion onto the X and
		Z axes, the angle between them is returned. If set to true though, the
		result is the actual yaw that will be used to implement the quaternion,
		which is the shortest possible path to get to the same orientation and 
		may involve less axial rotation. 
		*/
		Radian getYaw(bool reprojectAxis = true) const;		
		/// Equality with tolerance (tolerance is max angle difference)
		bool equals(const Quaternion& rhs, const Radian& tolerance) const;

		// spherical linear interpolation
		static DualQuaternion Slerp (Real fT, const Quaternion& rkP,
			const Quaternion& rkQ, bool shortestPath = false);

		static Quaternion SlerpExtraSpins (Real fT,
			const Quaternion& rkP, const Quaternion& rkQ,
			int iExtraSpins);

		// setup for spherical quadratic interpolation
		static void Intermediate (const Quaternion& rkQ0,
			const Quaternion& rkQ1, const Quaternion& rkQ2,
			Quaternion& rka, Quaternion& rkB);

		// spherical quadratic interpolation
		static DualQuaternion Squad (Real fT, const Quaternion& rkP,
			const Quaternion& rkA, const Quaternion& rkB,
			const Quaternion& rkQ, bool shortestPath = false);

		// normalised linear interpolation - faster but less accurate (non-constant rotation velocity)
		static DualQuaternion nlerp(Real fT, const Quaternion& rkP, 
			const Quaternion& rkQ, bool shortestPath = false);
		
		/** Screw linear interpolation 
		@param 
		*/
		static DualQuaternion ScLerp(Real fT, const DualQuaternion& rkP,
			const DualQuaternion& rkQ);




		/** DualQuaternion Linear Blending
		@param 
		*/
		static DualQuaternion DLB(Real fT, const DualQuaternion& rkP, 
			const DualQuaternion& rkQ);


		/** Function for writing to a stream. Outputs "DualQuaternion(Ordinary:w, x, y, z;Dual:w,x,y,z)" with respective w,x,y,z
		being the member values of ordinary and dual quaternion.
		*/
		inline _OgreExport friend std::ostream& operator <<
			( std::ostream& o, const DualQuaternion& dq )
		{
			o << "DualQuaternion(Ordinary:" << Ordinary.w << ", " << Ordinary.x << ", " << Ordinary.y << ", " << Ordinary.z << ";";
			o << "Dual:"<< Dual.w << "," << Dual.x << "," << Dual.y << "," << Dual.z << ")";
			return o;
		}

	};

}




#endif 
