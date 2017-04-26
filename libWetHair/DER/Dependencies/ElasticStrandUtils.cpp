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


#include "ElasticStrandUtils.h"

Vec3 parallelTransport( const Vec3& u, const Vec3& t0, const Vec3& t1 )
{
    // Compute rotation axis (if any)
    Vec3 b = t0.cross( t1 );
    const scalar bNorm = b.norm();
    if( isSmall( bNorm ) ) // vectors are nearly collinear
        return u;
    b /= bNorm;

    const Vec3& n0 = t0.cross( b ).normalized();
    const Vec3& n1 = t1.cross( b ).normalized();

    return u.dot( t0.normalized() ) * t1.normalized() + u.dot( n0 ) * n1 + u.dot( b ) * b;
}

Vec3 normalParallelTransport( const Vec3& u, const Vec3& t0, const Vec3& t1 )
{
    // This should be called only to transport an orthogonal vector
    assert( isSmall(u.dot(t0)) );

    // Compute rotation axis (if any)
    Vec3 b = t0.cross( t1 );
    const scalar bNorm = b.norm();
    if( isSmall( bNorm ) ) // vectors are nearly collinear
        return u;
    b /= bNorm;

    const Vec3& n0 = t0.cross( b ).normalized();
    const Vec3& n1 = t1.cross( b ).normalized();

    return u.dot( n0 ) * n1 + u.dot( b ) * b;
}

Vec3 orthonormalParallelTransport( const Vec3& u, const Vec3& t0, const Vec3& t1 )
{
    // This should be called only to transport an orthogonal vector
    assert( isSmall(u.dot(t0)) );

    Vec3 b = t0.cross( t1 );
    const scalar bNorm = b.norm();
    if( isSmall( bNorm ) ) // vectors are nearly collinear
        return u;
    b /= bNorm;

    const Vec3& n0 = t0.cross( b );
    const Vec3& n1 = t1.cross( b );

    return u.dot( n0 ) * n1 + u.dot( b ) * b;
}

bool containsNans( const VecX &dofs )
{
    bool panic = false;
    for( int i = 0; i < dofs.size(); ++i )
    {
        if( std::isnan( dofs[i] ) )
        {
            panic = true;
            break;
        }
    }

    return panic;
}
