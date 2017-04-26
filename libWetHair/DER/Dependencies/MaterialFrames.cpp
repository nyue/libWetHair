//
// This file is part of the libWetHair open source project
//
// Copyright 2017 Yun (Raymond) Fei, Henrique Teles Maia, Christopher Batty,
// Changxi Zheng, and Eitan Grinspun
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include "MaterialFrames.h"

template<>
const char* MaterialFrames<1>::name() const
{
    return "MaterialFrames1";
}

template<>
const char* MaterialFrames<2>::name() const
{
    return "MaterialFrames2";
}

// GCC did not like this... split into two
//template<int FrameN>
//void MaterialFrames<FrameN>::compute()

template<>
inline Vec3 MaterialFrames<1>::linearMix( const Vec3& u, const Vec3& v, scalar s, scalar c )
{
    return c * u + s * v;
}

template<>
inline Vec3 MaterialFrames<2>::linearMix( const Vec3& u, const Vec3& v, scalar s, scalar c )
{
    return -s * u + c * v;
}

template<>
void MaterialFrames<1>::compute()
{
    m_value.resize( m_size );
    const Vec3Array& referenceFrames1 = m_referenceFrames1.get();
    const Vec3Array& referenceFrames2 = m_referenceFrames2.get();
    const VecX& sinThetas = m_trigThetas.getSines();
    const VecX& cosThetas = m_trigThetas.getCosines();

	for (IndexType vtx = 0; vtx < m_firstValidIndex; ++vtx)
	{
		m_value[vtx].setZero();
	}

    for( IndexType vtx = m_firstValidIndex; vtx < size(); ++vtx )
    {
        const Vec3& u = referenceFrames1[vtx];
        const Vec3& v = referenceFrames2[vtx];
        const scalar s = sinThetas[vtx];
        const scalar c = cosThetas[vtx];

        m_value[vtx] = linearMix( u, v, s, c );
    }

    setDependentsDirty();
}

template<>
void MaterialFrames<2>::compute()
{
    m_value.resize( m_size );
    const Vec3Array& referenceFrames1 = m_referenceFrames1.get();
    const Vec3Array& referenceFrames2 = m_referenceFrames2.get();
    const VecX& sinThetas = m_trigThetas.getSines();
    const VecX& cosThetas = m_trigThetas.getCosines();

	for (IndexType vtx = 0; vtx < m_firstValidIndex; ++vtx)
	{
		m_value[vtx].setZero();
	}

    for( IndexType vtx = m_firstValidIndex; vtx < size(); ++vtx )
    {
        const Vec3& u = referenceFrames1[vtx];
        const Vec3& v = referenceFrames2[vtx];
        const scalar s = sinThetas[vtx];
        const scalar c = cosThetas[vtx];

        m_value[vtx] = linearMix( u, v, s, c );
    }

    setDependentsDirty();
}
