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


#include "TwistingForce.h"
#include "ViscousOrNotViscous.h"

template<typename ViscousT>
scalar TwistingForce<ViscousT>::localEnergy( const StrandForce& strand, const IndexType vtx )
{
    const scalar kt = ViscousT::kt( strand, vtx );
    const scalar undefTwist = ViscousT::thetaBar( strand, vtx );
    const scalar ilen = strand.m_invVoronoiLengths[vtx];
    const scalar twist = strand.m_strandState->m_twists[vtx];

    return 0.5 * kt * square( twist - undefTwist ) * ilen;
}

template<typename ViscousT>
void TwistingForce<ViscousT>::computeLocal(Eigen::Matrix<scalar, 11, 1>& localF,
        const StrandForce& strand, const IndexType vtx )
{
    const scalar kt = ViscousT::kt( strand, vtx );
    const scalar undefTwist = ViscousT::thetaBar( strand, vtx );
    const scalar ilen = strand.m_invVoronoiLengths[vtx];
    const scalar twist = strand.m_strandState->m_twists[vtx];

    localF = -kt * ilen * ( twist - undefTwist ) * strand.m_strandState->m_gradTwists[vtx];
}

template<typename ViscousT>
void TwistingForce<ViscousT>::computeLocal( Eigen::Matrix<scalar, 11, 11>& localJ,
        const StrandForce& strand, const IndexType vtx )
{
    const scalar kt = ViscousT::kt( strand, vtx );
    const scalar ilen = strand.m_invVoronoiLengths[vtx];
    const Mat11& gradTwistSquared = strand.m_strandState->m_gradTwistsSquared[vtx];

    localJ = -kt * ilen * gradTwistSquared;
    if( strand.m_requiresExactForceJacobian )
    {
        const scalar undeformedTwist = ViscousT::thetaBar( strand, vtx );
        const scalar twist = strand.m_strandState->m_twists[vtx];
        const Mat11& hessTwist = strand.m_strandState->m_hessTwists[vtx];
        localJ += -kt * ilen * ( twist - undeformedTwist ) * hessTwist;
    }
}

template<typename ViscousT>
void TwistingForce<ViscousT>::addInPosition( VecX& globalForce, const IndexType vtx,
        const LocalForceType& localForce )
{
    globalForce.segment<11>( 4 * ( vtx - 1 ) ) += localForce;
}

template<typename ViscousT>
void TwistingForce<ViscousT>::accumulateCurrentE( scalar& energy, StrandForce& strand )
{
    for( IndexType vtx = s_first; vtx < strand.getNumVertices() - s_last; ++vtx )
    {
        energy += localEnergy( strand, vtx );
    }
}

template<typename ViscousT>
void TwistingForce<ViscousT>::accumulateCurrentF( VecX& force, StrandForce& strand )
{
    for( IndexType vtx = s_first; vtx < strand.getNumVertices() - s_last; ++vtx )
    {
        LocalForceType localF;
        computeLocal( localF, strand, vtx );
        addInPosition( force, vtx, localF );
    }
}

template<typename ViscousT>
void TwistingForce<ViscousT>::accumulateIntegrationVars( 
    const unsigned& pos_start, const unsigned& j_start, const unsigned& tildek_start, const unsigned& global_start_dof, 
    StrandForce& strand, VectorXs& lambda, TripletXs& J, TripletXs& tildeK, TripletXs& stiffness, VectorXs& Phi, const int& lambda_start )
{
    for( IndexType vtx = s_first; vtx < strand.getNumVertices() - s_last; ++vtx )
    {
        unsigned dfirst = global_start_dof + 4 * (vtx-1);
        unsigned dsecond = global_start_dof + 4 * vtx;
        unsigned dthird = global_start_dof + 4 * (vtx+1);

        const scalar twist = strand.m_strandState->m_twists[vtx];
        const scalar undeformedTwist = ViscousT::thetaBar( strand, vtx );
        const scalar kt = ViscousT::kt( strand, vtx );
        const scalar ilen = strand.m_invVoronoiLengths[vtx];

        unsigned idx_pos = pos_start + vtx - 1;
        Phi[ idx_pos ] = twist - undeformedTwist;
        stiffness[ idx_pos ] = Triplets( idx_pos, idx_pos, kt * ilen );

        const Vec11& gt = strand.m_strandState->m_gradTwists[vtx];
        unsigned idx_j = j_start + 11 * (vtx - 1);
        for( int r = 0; r < 4; ++r ){
            J[ idx_j + r ] = Triplets( idx_pos, dfirst + r, gt(r));
            J[ idx_j + 4 + r ] = Triplets( idx_pos, dsecond + r, gt(r + 4));
            if( r < 3 ) J[ idx_j + 8 + r ] = Triplets( idx_pos, dthird + r, gt(r + 8));
        }

        if( std::is_same< ViscousT, NonViscous >::value ){
            scalar weight = -lambda[ lambda_start + (vtx-1) ];
            const Mat11& hessTwist = strand.m_strandState->m_hessTwists[vtx];
            unsigned idx_tildek = tildek_start + 121 * (vtx - 1);
            for(int r = 0; r < 11; ++r) {
                for(int s = 0; s < 11; ++s){
                    tildeK[ idx_tildek + r * 11 + s] = Triplets( dfirst + r, dfirst + s, hessTwist(r, s) * weight);
                }
            }
        }
    }
}

template class TwistingForce<NonViscous>;
template class TwistingForce<Viscous>;
