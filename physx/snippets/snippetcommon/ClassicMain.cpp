//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include <iostream>
#include "PxPhysicsAPI.h"

extern int snippetMain(int, const char*const*);

template <class T>
constexpr T PI_REAL = T(3.1415926535897932385);
constexpr float PI = PI_REAL<float>;

template <int RANGE>
constexpr float Range2Rad(float fValue)
{
	return fValue * (2 * PI) / RANGE;
}

template <int RANGE>
constexpr float Rad2Range(float fRad)
{
	return fRad * RANGE / (2 * PI);
}

constexpr auto Dir2Rad = Range2Rad<65536>;
constexpr auto Rad2Dir = Rad2Range<65536>;

class A3DTransform
{
	physx::PxVec3 _pos;
	physx::PxVec3 _axis;
	float _angle = .0f;
public:

	void Set(const physx::PxVec3& pos, const physx::PxVec3& axis, float angle)
	{
		_pos = pos;
		_axis = axis;
		_axis.normalize();
		_angle = angle;
	}

	void SetWithDir(const physx::PxVec3& pos, unsigned short dir)
	{
		_pos = pos;
		_axis = physx::PxVec3(0, 1, 0);
		_angle = Dir2Rad(dir);
	}

	void GetPhyTransform(physx::PxTransform& output) const
	{
		output.p = _pos;
		output.q = physx::PxQuat(_angle, _axis).getConjugate();
	}

	physx::PxVec3 ToParent(const physx::PxVec3& pos_child) const
	{
		physx::PxTransform transform;
		GetPhyTransform(transform);

		auto position_parent = transform.transform(pos_child);
		return position_parent;
	}

	physx::PxVec3 ToChild(const physx::PxVec3& pos_parent) const
	{
		physx::PxTransform transform;
		GetPhyTransform(transform);

		auto position_child = transform.transformInv(pos_parent);
		return position_child;
	}
};

int main(int argc, char** argv)
{
	if (true)
	{
		A3DTransform transform;
		transform.Set(physx::PxVec3(0, 0, 0), physx::PxVec3(0, 1, 1), physx::PxHalfPi);
		auto pos_global = transform.ToParent(physx::PxVec3(0, 0, 2));
		std::cout << "completed" << std::endl;
		return 0;
	}
	if (false)
	{
		A3DTransform transform;
		transform.SetWithDir(physx::PxVec3(0, 0, 0), 65536 / 4);
		auto pos_global = transform.ToParent(physx::PxVec3(2, 2, 2));
		auto pos_local = transform.ToChild(pos_global);
		std::cout << "completed" << std::endl;
		return 0;
	}
	if (false)
	{
		unsigned short dir = 65536 / 4;
		std::cout << Dir2Rad(dir) << std::endl;
		return 0;
	}
	return snippetMain(argc, argv);
}
