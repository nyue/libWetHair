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


#include "CompliantImplicitEuler.h"
#include "TwoDScene.h"
#include "TimingUtilities.h"
// #include <Eigen/PardisoSupport>

// TODO: Save space by creating dx, dv, rhs, A only once.

// #define CIE_VERBOSE

template<int DIM>
CompliantImplicitEuler<DIM>::CompliantImplicitEuler(TwoDScene<DIM>* scene, int max_iters, scalar criterion, bool autoUpdateNextInit)
: SceneStepper<DIM>(), m_scene(scene), m_max_iters(max_iters), m_criterion(criterion), m_bAutoUpdateNextInit(autoUpdateNextInit)
{
  SceneStepper<DIM>::m_timing_statistics.resize(4, 0);
  
  const int predicted_nnzs_row = DIM * 4;
  const int ndof = scene->getNumDofs();
  
#ifdef CIE_VERBOSE
  stringutils::print(m_M);
  stringutils::print(m_Fix);
#endif
  
  m_A_nz.reserve(ndof * predicted_nnzs_row);
  m_Kext_nz.reserve(ndof * predicted_nnzs_row);
  m_A.resize(ndof, ndof);
  m_Kext.resize(ndof, ndof);
  m_M_nz.resize(ndof);
  m_M.resize(ndof, ndof);
  m_A.reserve(ndof * predicted_nnzs_row);
  m_Kext.reserve(ndof * predicted_nnzs_row);
  
  m_gradU.resize(ndof);
  m_vplus = scene->getV();
}

template<int DIM>
CompliantImplicitEuler<DIM>::~CompliantImplicitEuler()
{}

template<int DIM>
void CompliantImplicitEuler<DIM>::updateNumConstraints(const VectorXs& dx, const VectorXs& dv, scalar dt)
{
  int num_pos, num_vel, num_J, num_Jv, num_Jxv, num_tildeK;
  m_scene->updateNumConstraints(num_pos, num_vel, num_J, num_Jv, num_Jxv, num_tildeK, m_interhair_idx, m_interhair_num);
  
  m_lambda.resize(num_pos);
  m_lambda_v.resize(num_vel);
  m_A_nz.resize(num_tildeK);
  m_J_nz.resize(num_J);
  m_Jv_nz.resize(num_Jv);
  m_Jxv_nz.resize(num_Jxv);
  m_invC_nz.resize(num_pos);
  m_invCv_nz.resize(num_vel);
  m_Phi.resize(num_pos);
  m_Phi_v.resize(num_vel);
  
  m_scene->postPreprocess(m_lambda, m_lambda_v, m_J_nz, m_Jv_nz, m_Jxv_nz, m_A_nz, m_invC_nz, m_invCv_nz, m_Phi, m_Phi_v, dx, dv, dt);
  
  stringutils::print(MAKE_REF(m_lambda));
  stringutils::print(MAKE_REF(m_lambda_v));
  stringutils::print(MAKE_REF(m_J_nz));
  stringutils::print(MAKE_REF(m_Jv_nz));
  stringutils::print(MAKE_REF(m_Jxv_nz));
  stringutils::print(MAKE_REF(m_A_nz));
  stringutils::print(MAKE_REF(m_invC_nz));
  stringutils::print(MAKE_REF(m_invCv_nz));
  stringutils::print(MAKE_REF(m_Phi));
  stringutils::print(MAKE_REF(m_Phi_v));
}

template<int DIM>
bool CompliantImplicitEuler<DIM>::stepScene( TwoDScene<DIM>& scene, scalar dt, bool updatePreCompute )
{
  std::cout << "[pre-compute]" << std::endl;
  scalar t0 = timingutils::seconds();
  scalar t1;
  
  VectorXs& x = scene.getX();
  VectorXs& v = scene.getV();
  const VectorXs& m = scene.getM();
  assert(x.size() == v.size());
  assert(x.size() == m.size());
  int ndof = x.size();
  int nprts = scene.getNumParticles();
  stringutils::print(x);
  stringutils::print(v);

  VectorXs dv(v.size());
  VectorXs dx(x.size());
  VectorXs dx_scripted(x.size());

  dv.setZero();
  dx = v * dt;
  dx_scripted.setZero();
  
  int np = m_scene->getNumParticles();
  for(int i = 0; i < np; ++i)
  {
    if(scene.isFixed(i)) {
      int numdofs = scene.isMassSpring() || scene.isTip(i) ? DIM : 4;
      dx_scripted.segment( scene.getDof(i), numdofs ) = v.segment( scene.getDof(i), numdofs ) * dt;
    }
  }
  
  if(updatePreCompute) scene.preCompute(dx_scripted, dv, dt);
  
  t1 = timingutils::seconds();
  SceneStepper<DIM>::m_timing_statistics[0] += t1 - t0; // local precomputation
  t0 = t1;
  
  std::cout << "[compute-assist-vars]" << std::endl;
  updateNumConstraints(dx_scripted, dv, dt);
  
  t1 = timingutils::seconds();
  SceneStepper<DIM>::m_timing_statistics[1] += t1 - t0; // Jacobian
  t0 = t1;
  
  const int nconstraint = m_lambda.size();
  const int nconstraint_v = m_lambda_v.size();
  
  m_A_nz.erase(std::remove_if(m_A_nz.begin(), m_A_nz.end(), [&] (const Triplets& t) {
    return scene.isFixed(scene.getVertFromDof( t.row() )) || scene.isFixed( scene.getVertFromDof(t.col()) ) || (t.value() == 0.0);
  }), m_A_nz.end());
  m_A.setFromTriplets(m_A_nz.begin(), m_A_nz.end());
#ifdef CIE_VERBOSE
  stringutils::print(MAKE_REF(m_A));
#endif
  
  if(m_J.rows() != nconstraint) m_J.resize(nconstraint, ndof);
  m_J_nz.erase(std::remove_if(m_J_nz.begin(), m_J_nz.end(), [&] (const Triplets& t) {
    return scene.isFixed(  scene.getVertFromDof(t.col() ) ) || (t.value() == 0.0);
  }), m_J_nz.end());
  m_J.setFromTriplets(m_J_nz.begin(), m_J_nz.end());
  m_J.makeCompressed();
  
#ifdef CIE_VERBOSE
  stringutils::print(MAKE_REF(m_J));
#endif
  
  if(nconstraint_v > 0) {
    if(m_Jv.rows() != nconstraint_v) m_Jv.resize(nconstraint_v, ndof);
    m_Jv_nz.erase(std::remove_if(m_Jv_nz.begin(), m_Jv_nz.end(), [&] (const Triplets& t) {
      return scene.isFixed(  scene.getVertFromDof(t.col() ) ) || (t.value() == 0.0);
    }), m_Jv_nz.end());
    m_Jv.setFromTriplets(m_Jv_nz.begin(), m_Jv_nz.end());
    m_Jv.makeCompressed();
    
    if(m_Jxv.rows() != nconstraint_v) m_Jxv.resize(nconstraint_v, ndof);
    m_Jxv_nz.erase(std::remove_if(m_Jxv_nz.begin(), m_Jxv_nz.end(), [&] (const Triplets& t) {
      return scene.isFixed(  scene.getVertFromDof(t.col() ) ) || (t.value() == 0.0);
    }), m_Jxv_nz.end());
    m_Jxv.setFromTriplets(m_Jxv_nz.begin(), m_Jxv_nz.end());
    m_Jxv.makeCompressed();
  }
  
  
#ifdef CIE_VERBOSE
  stringutils::print(MAKE_REF(m_Jv));
#endif
  
  if(m_invC.rows() != nconstraint) m_invC.resize(nconstraint, nconstraint);
  m_invC.setFromTriplets(m_invC_nz.begin(), m_invC_nz.end());
  
#ifdef CIE_VERBOSE
  stringutils::print(MAKE_REF(m_invC));
#endif
  
  if(m_invCv.rows() != nconstraint_v) m_invCv.resize(nconstraint_v, nconstraint_v);
  m_invCv.setFromTriplets(m_invCv_nz.begin(), m_invCv_nz.end());
  
#ifdef CIE_VERBOSE
  stringutils::print(MAKE_REF(m_invCv));
#endif
  
  m_JC = m_J.transpose() * m_invC;
  
  if(nconstraint_v > 0) m_JvC = m_Jv.transpose() * m_invCv;
  
  if(nconstraint_v > 0)
    m_A += (m_JC * m_J) + (m_JvC * (m_Jv / dt + m_Jxv)) ;
  else
    m_A += m_JC * m_J;
  
  m_A *= dt * dt;
  
#ifdef CIE_VERBOSE
  stringutils::print(MAKE_REF(m_A));
#endif

  for(int i = 0; i < ndof; ++i)
  {
    if(scene.isFixed( scene.getVertFromDof(i) )) {
      m_M_nz[i] = (Triplets(i, i, 1.0));
    } else {
      m_M_nz[i] = (Triplets(i, i, m(i)));
    }
  }
  
  m_M.setFromTriplets(m_M_nz.begin(), m_M_nz.end());
  
  m_A += m_M;
  
  //m_A.makeCompressed();
#ifdef CIE_VERBOSE
  stringutils::print(MAKE_REF(m_A));
#endif
  
  m_gradU.setZero();
  scene.accumulateExternalGradU(m_gradU);
  
#ifdef CIE_VERBOSE
  stringutils::print(MAKE_REF(m_gradU));
#endif
  
  zeroFixedDoFs(scene, m_gradU);
  
#ifdef CIE_VERBOSE
  stringutils::print(MAKE_REF(m_gradU));
#endif

  if(nconstraint_v > 0)
    m_b = v.cwiseProduct(m) - dt * (m_gradU + (m_JC * m_Phi + m_JvC * (m_Phi_v - m_Jv * v)));
  else
    m_b = v.cwiseProduct(m) - dt * (m_gradU + (m_JC * m_Phi));
  
#ifdef CIE_VERBOSE
  stringutils::print(MAKE_REF(m_b));
#endif

  m_A.makeCompressed();
  
  t1 = timingutils::seconds();
  SceneStepper<DIM>::m_timing_statistics[2] += t1 - t0; t0 = t1; // matrix composition

  std::cout << "[solve-equations]" << std::endl;
  if(m_max_iters > 0) {
    scalar nmb = m_b.norm();
    
    m_iterative_solver.compute(m_A);
    m_iterative_solver.setTolerance(m_criterion / nmb);
    m_iterative_solver.setMaxIterations(m_max_iters);
    m_vplus = m_iterative_solver.solveWithGuess(m_b, v);
    std::cout << "[cg total iter: " << m_iterative_solver.iterations() << ", res: " << (m_iterative_solver.error() * nmb) << "]" << std::endl;
  } else {
    m_solver.compute(m_A);
    m_vplus = m_solver.solve(m_b);
  }
#ifdef CIE_VERBOSE
  stringutils::print(MAKE_REF(m_vplus));
#endif
  
  m_lambda = -m_invC * (m_J * m_vplus * dt + m_Phi);
  m_lambda_v = -m_invCv * (m_Jxv * m_vplus * dt + m_Jv * (m_vplus - v) + m_Phi_v);
  
  t1 = timingutils::seconds();
  SceneStepper<DIM>::m_timing_statistics[3] += t1 - t0; t0 = t1; // solve equation
  
  for(int i = 0; i < nprts; ++i){
    if(!scene.isFixed(i)){
      int numdofs = scene.isMassSpring() || scene.isTip(i) ? DIM : 4;
      v.segment(scene.getDof(i), numdofs) = m_vplus.segment(scene.getDof(i), numdofs);
    }
  }
  
#ifdef CIE_VERBOSE
  stringutils::print(MAKE_REF(m_lambda));
  stringutils::print(MAKE_REF(m_lambda_v));
#endif
  
  m_scene->storeLambda(m_lambda, m_lambda_v);
#ifdef CIE_VERBOSE
  stringutils::print(v);
#endif
  SceneStepper<DIM>::m_next_x = x + dt * v;

  return true; 
}

template<int DIM>
std::string CompliantImplicitEuler<DIM>::getName() const
{
  return "Linear Compliant Implicit Euler";
}

template<int DIM>
void CompliantImplicitEuler<DIM>::zeroFixedDoFs( const TwoDScene<DIM>& scene, VectorXs& vec )
{
  int nprts = scene.getNumParticles();
  for( int i = 0; i < nprts; ++i ){
    if( scene.isFixed(i) ){
      int numdofs = scene.isMassSpring() || scene.isTip(i) ? DIM : 4;
      vec.segment( scene.getDof(i), numdofs ).setZero();
    }
  }
}

// explicit instantiations at bottom
template class CompliantImplicitEuler<2>;
template class CompliantImplicitEuler<3>;
