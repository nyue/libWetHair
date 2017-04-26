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


#ifndef __LINEAR_SPRING_FORCE_H__
#define __LINEAR_SPRING_FORCE_H__

#include <Eigen/Core>
#include "Force.h"
#include <iostream>

template<int DIM>
class TwoDScene;

template<int DIM>
class LinearSpringForce : public Force
{
public:

  LinearSpringForce( TwoDScene<DIM>* parent, const std::pair<int,int>& endpoints, const scalar& k, const scalar& l0, const scalar& b = 0.0 );

  virtual ~LinearSpringForce();
  
  virtual void addEnergyToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, scalar& E );
  
  virtual void addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE );
  
  virtual void addHessXToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, TripletXs& hessE );
  
  virtual void addHessVToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, TripletXs& hessE );
  
  virtual void addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE, int pidx );
  
  virtual void addHessXToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& hessE, int pidx );
  
  virtual void addHessVToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& hessE, int pidx );
  
  virtual Force* createNewCopy();
  
  virtual void preCompute( const VectorXs& x, const VectorXs& v, const VectorXs& m, const scalar& dt );
  
  virtual void getAffectedVars( int pidx, std::unordered_set<int>& vars );
  
  virtual int getAffectedHair( const std::vector<int> particle_to_hairs );
  
  virtual bool isContained( int pidx );
  
  virtual void computeIntegrationVars( const VectorXs& x, const VectorXs& v, const VectorXs& m,
                                      VectorXs& lambda, VectorXs& lambda_v,
                                      TripletXs& J, TripletXs& Jv, TripletXs& Jxv, TripletXs& tildeK,
                                      TripletXs& stiffness, TripletXs& damping, VectorXs& Phi, VectorXs& Phiv, const scalar& dt);
  
  virtual int numConstraintPos();
  
  virtual int numConstraintVel();
  
  virtual int numJ();
  
  virtual int numJv();
  
  virtual int numJxv();
  
  virtual int numTildeK();
  
  virtual bool isParallelized();
  
  virtual bool isPrecomputationParallelized();
  
  virtual void storeLambda(const VectorXs& lambda, const VectorXs& lambda_v);
  
  virtual const char* name();
  
private:
  std::pair<int,int> m_endpoints;
  scalar m_k;
  scalar m_l0;
  scalar m_b;
  
  Vectors<DIM> m_D;
  Vectors<DIM> m_lambda_v;
  Vectors<DIM> m_lambda;

  TwoDScene<DIM>* m_scene;
};

#endif