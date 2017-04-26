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

#ifndef __COMPLIANT_IMPLICIT_EULER__
#define __COMPLIANT_IMPLICIT_EULER__

#include <Eigen/Dense>
#include <iostream>

#include "SceneStepper.h"
#include "MathUtilities.h"
#include "StringUtilities.h"

template<int DIM>
class TwoDScene;

template<int DIM>
class CompliantImplicitEuler : public SceneStepper<DIM>
{
public:
  CompliantImplicitEuler(TwoDScene<DIM>* scene, int max_iters, scalar criterion, bool autoUpdateNextInit = true);
  
  virtual ~CompliantImplicitEuler();
  
  virtual bool stepScene( TwoDScene<DIM>& scene, scalar dt, bool updatePreCompute = true );
  
  virtual std::string getName() const;
private:
  void zeroFixedDoFs( const TwoDScene<DIM>& scene, VectorXs& vec );
  void updateNumConstraints(const VectorXs& dx, const VectorXs& dv, scalar dt);
  
  bool m_bAutoUpdateNextInit;
  int m_max_iters;
  scalar m_criterion;
  
  TwoDScene<DIM>* m_scene;
  
  TripletXs m_Kext_nz;
  TripletXs m_A_nz;
  TripletXs m_J_nz;
  TripletXs m_Jv_nz;
  TripletXs m_Jxv_nz;
  TripletXs m_Fix_nz;
  TripletXs m_M_nz;
  TripletXs m_invC_nz;
  TripletXs m_invCv_nz;
  
  SparseXs m_Kext;
  SparseXs m_A;
  SparseXs m_J;
  SparseXs m_JC;
  SparseXs m_Jv;
  SparseXs m_JvC;
  SparseXs m_Jxv;
  SparseXs m_Fix;
  SparseXs m_M;
  SparseXs m_invC;
  SparseXs m_invCv;
  
  VectorXs m_lambda;
  VectorXs m_lambda_v;
  VectorXs m_gradU;
  VectorXs m_Phi;
  VectorXs m_Phi_v;
  VectorXs m_b;
  VectorXs m_vplus;
  
  Vector6i m_interhair_idx;
  Vector6i m_interhair_num;

  Eigen::SimplicialLDLT< SparseXs > m_solver;
  Eigen::ConjugateGradient< SparseXs > m_iterative_solver;
};

#endif
