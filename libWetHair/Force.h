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

#ifndef __FORCE_H__
#define __FORCE_H__

#include <Eigen/Core>
#include <unordered_set>

#include "MathDefs.h"

class Force
{
protected:
  int m_internal_index_pos;
  int m_internal_index_vel;
  
  int m_internal_index_J;
  int m_internal_index_Jv;
  int m_internal_index_Jxv;
  int m_internal_index_tildeK;
  
public:

  virtual ~Force();
  
  virtual void addEnergyToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, scalar& E ) = 0;
  
  virtual void addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE ) = 0;
  
  virtual void addHessXToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, TripletXs& hessE ) = 0;

  virtual void addHessVToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, TripletXs& hessE ) = 0;

  virtual void addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE, int pidx ) = 0;
  
  virtual void addHessXToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& hessE, int pidx ) = 0;
  
  virtual void addHessVToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& hessE, int pidx ) = 0;
  
  virtual void computeIntegrationVars( const VectorXs& x, const VectorXs& v, const VectorXs& m,
                                VectorXs& lambda, VectorXs& lambda_v,
                                TripletXs& J, TripletXs& Jv, TripletXs& Jxv, TripletXs& tildeK,
                                TripletXs& stiffness, TripletXs& damping, VectorXs& Phi, VectorXs& Phiv, const scalar& dt) = 0;
  
  virtual int numConstraintPos() = 0;
  
  virtual int numConstraintVel() = 0;
  
  virtual int numJ() = 0;
  
  virtual int numJv() = 0;
  
  virtual int numJxv() = 0;
  
  virtual int numTildeK() = 0;
  
  virtual bool isParallelized() = 0;
  
  virtual bool isPrecomputationParallelized() = 0;
  
  virtual void storeLambda(const VectorXs& lambda, const VectorXs& lambda_v);
  
  virtual void setInternalIndex(
                                int index_pos,
                                int index_vel,
                                int index_J,
                                int index_Jv,
                                int index_Jxv,
                                int index_tildeK);
  
  virtual Force* createNewCopy() = 0;
  
  virtual const char* name() = 0;
  
  virtual void getAffectedVars( int pidx, std::unordered_set<int>& vars ) = 0;
  
  virtual int getAffectedHair( const std::vector<int> particle_to_hairs );
  
  virtual bool isContained( int pidx ) = 0;
  
  virtual bool isExternal();
  
  virtual void preCompute( const VectorXs& x, const VectorXs& v, const VectorXs& m, const scalar& dt );
  
  virtual bool isInterHair() const;
  
  virtual void postStepScene( const scalar& dt );
};

#endif
