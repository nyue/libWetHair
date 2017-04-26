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

#ifndef MATERIALFRAMES_H_
#define MATERIALFRAMES_H_

#include "ReferenceFrames.h"

/**
 * Unit: no dimension
 */
template<int FrameN>
class MaterialFrames: public DependencyNode<Vec3Array>
{
public:
    MaterialFrames( TrigThetas& trigThetas, ReferenceFrames1& referenceFrames1,
            ReferenceFrames2& referenceFrames2 ) :
            DependencyNode<Vec3Array>( 0, referenceFrames1.size() ), m_trigThetas( trigThetas ), m_referenceFrames1(
                    referenceFrames1 ), m_referenceFrames2( referenceFrames2 )
    {
        m_trigThetas.addDependent( this );
        m_referenceFrames1.addDependent( this );
        m_referenceFrames2.addDependent( this );
    }

    virtual const char* name() const;

protected:
    virtual void compute();
    Vec3 linearMix( const Vec3& u, const Vec3& v, scalar s, scalar c );

    TrigThetas& m_trigThetas;
    ReferenceFrames1& m_referenceFrames1;
    ReferenceFrames2& m_referenceFrames2;
};

#endif
