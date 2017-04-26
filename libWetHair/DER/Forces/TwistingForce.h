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

#ifndef TWISTINGFORCE_H_
#define TWISTINGFORCE_H_

#include "ViscousOrNotViscous.h"
#include "../StrandForce.h"

template<typename ViscousT = NonViscous>
class TwistingForce
{
public:
    TwistingForce()
    {}
    
    virtual ~TwistingForce()
    {}

public:
    static const IndexType s_first = 1; // The first index on which this force can apply
    static const IndexType s_last = 1; // The last index (counting from the end)

    typedef Eigen::Matrix<scalar, 11, 1> LocalForceType;
    typedef Eigen::Matrix<scalar, 11, 11> LocalJacobianType;

    static std::string getName()
    {
        return ViscousT::getName() + "twisting";
    }

    static scalar localEnergy( const StrandForce& strand, const IndexType vtx );

    static void computeLocal( LocalForceType& localF, const StrandForce& strand,
            const IndexType vtx );

    static void computeLocal( LocalJacobianType& localJ, const StrandForce& strand,
            const IndexType vtx );

    static void addInPosition( VecX& globalForce, const IndexType vtx,
            const LocalForceType& localForce );

    static void accumulateCurrentE( scalar& energy, StrandForce& strand );
    static void accumulateCurrentF( VecX& force, StrandForce& strand );

    static void accumulateIntegrationVars( 
            const unsigned& pos_start, 
            const unsigned& j_start, 
            const unsigned& tildek_start, 
            const unsigned& global_start_dof, 
            StrandForce& strand, 
            VectorXs& lambda, 
            TripletXs& J, 
            TripletXs& tildeK, 
            TripletXs& stiffness, 
            VectorXs& Phi,
            const int& lambda_start );
};

#endif
