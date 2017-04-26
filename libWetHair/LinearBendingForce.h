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


#ifndef __LINEAR_BENDING_FORCE_H__
#define __LINEAR_BENDING_FORCE_H__

#include <Eigen/Core>
#include "Force.h"
#include <iostream>

template<int DIM>
class TwoDScene;

template<int DIM>
class LinearBendingForce : public Force
{
public:
  
  LinearBendingForce( TwoDScene<DIM>* parent, int idx1, int idx2, int idx3, const scalar& alpha, const scalar& beta, const Vectors<DIM-1>& theta0, const scalar& eb1n, const scalar& eb2n );
  
  virtual ~LinearBendingForce();
  
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
  
  virtual const char* name();
  
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

private:
  
  int m_idx1;
  int m_idx2;
  int m_idx3;
  
  Matrixs<DIM> m_R;       // rotation matrix
  
  scalar m_alpha;     // stiffness coefficient
  scalar m_beta;      // damping coefficient
  Vectors<DIM-1> m_theta0;    // rest angle
  scalar m_eb1n;      // norm of e1 bar
  scalar m_eb2n;      // norm of e2 bar
  
  Vectors<DIM> m_x1;
  Vectors<DIM> m_x2;
  Vectors<DIM> m_x3;
  
  Vectors<DIM> m_L0;
  
  Vectors<DIM> m_RL0;
  
  Vectors<DIM> m_lambda_v;
  Vectors<DIM> m_lambda;
  
  scalar m_c1;
  scalar m_c2;

  TwoDScene<DIM>* m_scene;
};

#endif
