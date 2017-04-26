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


#ifndef __STRAND_COMPLIANT_EULER__
#define __STRAND_COMPLIANT_EULER__

#include <iostream>
//#include <ceres/ceres.h>

#include "SceneStepper.h"
#include "MathUtilities.h"
#include "StringUtilities.h"

template<int DIM>
class StrandCompliantManager;

template<int DIM>
class StrandCompliantEuler
{
public:
  StrandCompliantEuler(StrandCompliantManager<DIM>* parent, int hidx);
  
  virtual ~StrandCompliantEuler();
  
  virtual bool stepScene( TwoDScene<DIM> & scene, scalar dt );
  
  virtual bool stepScene( TwoDScene<DIM> & scene, scalar dt, VectorXs& r, const VectorXs& b );
  
  virtual bool PreconditionScene( TwoDScene<DIM> & scene, scalar dt, VectorXs& r, const VectorXs& b );
  
  virtual void preIterate( TwoDScene<DIM>& scene, scalar dt );
  
  virtual void computeRHS( TwoDScene<DIM> & scene, scalar dt, VectorXs& b );
  
  virtual void computeRHSIncremental( TwoDScene<DIM> & scene, scalar dt, VectorXs& b, const VectorXs& vplus );
  
  virtual void computeAp( const VectorXs& p, VectorXs& b );
  
  virtual void updateLambda( TwoDScene<DIM>& scene, const VectorXs& dx, const VectorXs& dv, scalar dt );
  
  virtual void updateNextV( TwoDScene<DIM>& scene, const VectorXs& vplus );
private:
  TripletXs m_A_nz;
  TripletXs m_J_nz;
  TripletXs m_Jv_nz;
  TripletXs m_Jxv_nz;
  TripletXs m_invC_nz;
  TripletXs m_invCv_nz;
  
  TripletXs m_J_inter_nz;
  TripletXs m_Jv_inter_nz;
  TripletXs m_invC_inter_nz;
  TripletXs m_invCv_inter_nz;
  
  SparseXs m_A;
  SparseXs m_J;
  SparseXs m_Jv;
  SparseXs m_Jxv;
  SparseXs m_invC;
  SparseXs m_invCv;
  
  SparseXs m_J_inter;
  SparseXs m_Jv_inter;
  SparseXs m_invC_inter;
  SparseXs m_invCv_inter;
  
  VectorXs m_A_inv_diag;
  
  Eigen::SimplicialLDLT<SparseXs> m_solver;
  
  int m_hidx;

  
  StrandCompliantManager<DIM>* m_parent;
  
  int m_start_global_dof;
  int m_num_global_dof;
};

#endif
