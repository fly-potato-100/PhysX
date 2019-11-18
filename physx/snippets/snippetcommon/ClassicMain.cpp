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

struct A3DTransform
{
	physx::PxVec3 pos;
	unsigned short dir = 0;

	void GetPhyTransform(physx::PxTransform& output) const
	{
		float rad_dir = Dir2Rad(dir);
		physx::PxVec3 axis(.0f, -1.f, .0f);
		output = physx::PxTransform(pos, physx::PxQuat(rad_dir, axis));
	}

	physx::PxVec3 LocalPos2GlobalPos(const physx::PxVec3& pos_local) const
	{
		physx::PxTransform transform;
		GetPhyTransform(transform);

		auto position_global = transform.transform(pos_local);
		return position_global;
	}

	physx::PxVec3 GlobalPos2LocalPos(const physx::PxVec3& pos_global) const
	{
		physx::PxTransform transform;
		GetPhyTransform(transform);

		auto position_local = transform.transformInv(pos_global);
		return position_local;
	}
};

int main(int argc, char** argv)
{
	if (false)
	{
		A3DTransform transform{ physx::PxVec3(1, 2, 3), 65536 / 4 };
		auto pos_global = transform.LocalPos2GlobalPos(physx::PxVec3(2, 2, 2));
		auto pos_local = transform.GlobalPos2LocalPos(pos_global);
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
