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


#include "LinearSpringForce.h"
#include "TwoDScene.h"

template<int DIM>
LinearSpringForce<DIM>::LinearSpringForce( TwoDScene<DIM>* parent, const std::pair<int,int>& endpoints, const scalar& k, const scalar& l0, const scalar& b )
: Force()
, m_endpoints(endpoints)
, m_k(k)
, m_l0(l0)
, m_b(b)
, m_scene( parent )
{
  m_lambda.setZero();
  m_lambda_v.setZero();
  
  assert( m_endpoints.first >= 0 );
  assert( m_endpoints.second >= 0 );
  assert( m_endpoints.first != m_endpoints.second );
  assert( m_k >= 0.0 );
  assert( m_l0 >= 0.0 );
  assert( m_b >= 0.0 );
}

template<int DIM>
LinearSpringForce<DIM>::~LinearSpringForce()
{}

template<int DIM>
void LinearSpringForce<DIM>::preCompute( const VectorXs& x, const VectorXs& v, const VectorXs& m, const scalar& dt )
{
  m_D = (x.segment<DIM>( m_scene->getDof(m_endpoints.first) ) - x.segment<DIM>( m_scene->getDof(m_endpoints.second) ) ).normalized() * m_l0;
}

template<int DIM>
void LinearSpringForce<DIM>::addEnergyToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, scalar& E )
{
  assert( x.size() == v.size() );
  assert( m_endpoints.first >= 0 );
  assert( m_endpoints.second >= 0 );

  VectorXs dx = x.segment<DIM>( m_scene->getDof(m_endpoints.first) ) - x.segment<DIM>( m_scene->getDof(m_endpoints.second) ) - m_D;
  VectorXs dv = v.segment<DIM>( m_scene->getDof(m_endpoints.first) ) - v.segment<DIM>( m_scene->getDof(m_endpoints.second) );
  E += 0.5 * m_k * dx.squaredNorm() + 0.5 * m_b * dv.squaredNorm();
}

template<int DIM>
void LinearSpringForce<DIM>::addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE )
{
  assert( x.size() == v.size() );
  assert( x.size() == gradE.size() );
  assert( m_endpoints.first >= 0 );
  assert( m_endpoints.second >= 0 );

  // Compute the elastic component
  VectorXs dx = x.segment<DIM>( m_scene->getDof(m_endpoints.first) ) - x.segment<DIM>( m_scene->getDof(m_endpoints.second) ) - m_D;
  gradE.segment<DIM>( m_scene->getDof(m_endpoints.first) )  += m_k * dx;
  gradE.segment<DIM>( m_scene->getDof(m_endpoints.second) ) += -m_k * dx;

  // Compute the internal damping
  // Remember we are computing minus the force here
  VectorXs dv = v.segment<DIM>( m_scene->getDof(m_endpoints.first) ) - v.segment<DIM>( m_scene->getDof(m_endpoints.second) );
  gradE.segment<DIM>( m_scene->getDof(m_endpoints.first) )  += m_b * dv;
  gradE.segment<DIM>( m_scene->getDof(m_endpoints.second) ) -= m_b * dv;
}

template<int DIM>
void LinearSpringForce<DIM>::addHessXToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, TripletXs& hessE )
{
  assert( x.size() == v.size() );
  assert( x.size() == m.size() );
  assert( m_endpoints.first >= 0 );
  assert( m_endpoints.second >= 0 );
  
  Matrixs<DIM> hess = m_k * Matrixs<DIM>::Identity();
  
  for(int r = 0; r < DIM; ++r) for(int s = 0; s < DIM; ++s) hessE.push_back(Triplets( m_scene->getDof(m_endpoints.first) + r, m_scene->getDof(m_endpoints.first) + s, hess(r, s)));
  for(int r = 0; r < DIM; ++r) for(int s = 0; s < DIM; ++s) hessE.push_back(Triplets( m_scene->getDof(m_endpoints.second) + r, m_scene->getDof(m_endpoints.second) + s, hess(r, s)));
  for(int r = 0; r < DIM; ++r) for(int s = 0; s < DIM; ++s) hessE.push_back(Triplets( m_scene->getDof(m_endpoints.first) + r, m_scene->getDof(m_endpoints.second) + s, -hess(r, s)));
  for(int r = 0; r < DIM; ++r) for(int s = 0; s < DIM; ++s) hessE.push_back(Triplets( m_scene->getDof(m_endpoints.second) + r, m_scene->getDof(m_endpoints.first) + s, -hess(r, s)));
}

template<int DIM>
void LinearSpringForce<DIM>::addHessVToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, TripletXs& hessE )
{
  assert( x.size() == v.size() );
  assert( x.size() == m.size() );
  assert( m_endpoints.first >= 0 );
  assert( m_endpoints.second >= 0 );

  Matrixs<DIM> hess = m_b * Matrixs<DIM>::Identity();
  
  for(int r = 0; r < DIM; ++r) for(int s = 0; s < DIM; ++s) hessE.push_back(Triplets( m_scene->getDof(m_endpoints.first) + r, m_scene->getDof(m_endpoints.first) + s, hess(r, s)));
  for(int r = 0; r < DIM; ++r) for(int s = 0; s < DIM; ++s) hessE.push_back(Triplets( m_scene->getDof(m_endpoints.second) + r, m_scene->getDof(m_endpoints.second) + s, hess(r, s)));
  for(int r = 0; r < DIM; ++r) for(int s = 0; s < DIM; ++s) hessE.push_back(Triplets( m_scene->getDof(m_endpoints.first) + r, m_scene->getDof(m_endpoints.second) + s, -hess(r, s)));
  for(int r = 0; r < DIM; ++r) for(int s = 0; s < DIM; ++s) hessE.push_back(Triplets( m_scene->getDof(m_endpoints.second) + r, m_scene->getDof(m_endpoints.first) + s, -hess(r, s)));
}

template<int DIM>
void LinearSpringForce<DIM>::addGradEToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& gradE, int pidx )
{
  assert( x.size() == v.size() );
  assert( x.size() == gradE.size() );
  assert( m_endpoints.first >= 0 );
  assert( m_endpoints.second >= 0 );
  
  int idir = m_scene->getComponent( pidx );
  if( idir == DIM ) return;
  int ip = m_scene->getVertFromDof( pidx );
  
  if(ip == m_endpoints.first) {
    VectorXs fx = (x.segment<DIM>( m_scene->getDof(m_endpoints.first) ) - x.segment<DIM>( m_scene->getDof(m_endpoints.second) ) - m_D);
    
    gradE(pidx) += m_k * fx(idir);
    
    if(m_b > 0.0)
    {
      Vectors<DIM> fv = m_b * (v.segment<DIM>( m_scene->getDof(m_endpoints.first) ) - v.segment<DIM>( m_scene->getDof(m_endpoints.second) ));
      gradE(pidx) += fv(idir);
    }
  } else if(ip == m_endpoints.second) {
    VectorXs fx = (x.segment<DIM>( m_scene->getDof(m_endpoints.first) ) - x.segment<DIM>( m_scene->getDof(m_endpoints.second) ) - m_D);
    
    gradE(pidx) += -m_k * fx(idir);
    
    if(m_b > 0.0)
    {
      Vectors<DIM> fv = m_b * (v.segment<DIM>( m_scene->getDof(m_endpoints.first) ) - v.segment<DIM>( m_scene->getDof(m_endpoints.second) ));
      gradE(pidx) += -fv(idir);
    }
  }
}

template<int DIM>
void LinearSpringForce<DIM>::addHessXToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& hessE, int pidx )
{
  assert( x.size() == v.size() );
  assert( x.size() == m.size() );
  assert( x.size() == hessE.rows() );
  assert( m_endpoints.first >= 0 );
  assert( m_endpoints.second >= 0 );
  
  int idir = m_scene->getComponent( pidx );
  if( idir == DIM ) return;
  int ip = m_scene->getVertFromDof( pidx );

  if(ip == m_endpoints.first) {
    hessE.segment<DIM>( m_scene->getDof(m_endpoints.first) ) += m_k * Matrixs<DIM>::Identity().row(idir);
    hessE.segment<DIM>( m_scene->getDof(m_endpoints.second) ) -= m_k * Matrixs<DIM>::Identity().row(idir);
  } else if(ip == m_endpoints.second) {
    hessE.segment<DIM>( m_scene->getDof(m_endpoints.second) ) += m_k * Matrixs<DIM>::Identity().row(idir);
    hessE.segment<DIM>( m_scene->getDof(m_endpoints.first) ) -= m_k * Matrixs<DIM>::Identity().row(idir);
  }
}

template<int DIM>
void LinearSpringForce<DIM>::addHessVToTotal( const VectorXs& x, const VectorXs& v, const VectorXs& m, VectorXs& hessE, int pidx )
{
  assert( x.size() == v.size() );
  assert( x.size() == m.size() );
  assert( x.size() == hessE.rows() );
  assert( m_endpoints.first >= 0 );
  assert( m_endpoints.second >= 0 );
  
  // Contribution from damping  
  int idir = m_scene->getComponent( pidx );
  if( idir == DIM ) return;
  int ip = m_scene->getVertFromDof( pidx );

  if(ip == m_endpoints.first) {
    hessE.segment<DIM>( m_scene->getDof(m_endpoints.first) ) += m_b * Matrixs<DIM>::Identity().row(idir);
    hessE.segment<DIM>( m_scene->getDof(m_endpoints.second) ) -= m_b * Matrixs<DIM>::Identity().row(idir);
  } else if(ip == m_endpoints.second) {
    hessE.segment<DIM>( m_scene->getDof(m_endpoints.second) ) += m_b * Matrixs<DIM>::Identity().row(idir);
    hessE.segment<DIM>( m_scene->getDof(m_endpoints.first) ) -= m_b * Matrixs<DIM>::Identity().row(idir);
  }
}

template<int DIM>
void LinearSpringForce<DIM>::computeIntegrationVars( const VectorXs& x, const VectorXs& v, const VectorXs& m,
                                                    VectorXs& lambda, VectorXs& lambda_v,
                                                    TripletXs& J, TripletXs& Jv, TripletXs& Jxv, TripletXs& tildeK,
                                                    TripletXs& stiffness, TripletXs& damping, VectorXs& Phi, VectorXs& Phiv, const scalar& dt)
{
  Phi.segment<DIM>(m_internal_index_pos) = x.segment<DIM>( m_scene->getDof(m_endpoints.first) ) - x.segment<DIM>( m_scene->getDof(m_endpoints.second) ) - m_D;
  lambda.segment<DIM>(m_internal_index_pos) = m_lambda;
  
  if(m_b > 0.0) {
    Phiv.segment<DIM>(m_internal_index_vel) = v.segment<DIM>( m_scene->getDof(m_endpoints.first) ) - v.segment<DIM>( m_scene->getDof(m_endpoints.second) );
    lambda_v.segment<DIM>(m_internal_index_vel) = m_lambda_v;
  }
  
  for(int r = 0; r < DIM; ++r)
  {
    stiffness[m_internal_index_pos + r] = Triplets(m_internal_index_pos + r, m_internal_index_pos + r, m_k);
    
    J[m_internal_index_J + r] = Triplets(m_internal_index_pos + r, m_scene->getDof(m_endpoints.first) + r, 1.0);
    J[m_internal_index_J + DIM + r] = Triplets(m_internal_index_pos + r, m_scene->getDof(m_endpoints.second) + r, -1.0);
    
    if(m_b > 0.0) {
      damping[m_internal_index_vel + r] = Triplets(m_internal_index_vel + r, m_internal_index_vel + r, m_b);
      
      Jv[m_internal_index_Jv + r] = Triplets(m_internal_index_vel + r, m_scene->getDof(m_endpoints.first) + r, 1.0);
      Jv[m_internal_index_Jv + DIM + r] = Triplets(m_internal_index_vel + r, m_scene->getDof(m_endpoints.second) + r, -1.0);
    }
  }
}

template<int DIM>
int LinearSpringForce<DIM>::numJ()
{
  return DIM * 2;
}

template<int DIM>
int LinearSpringForce<DIM>::numJv()
{
  return m_b > 0.0 ? DIM * 2 : 0;
}

template<int DIM>
int LinearSpringForce<DIM>::numJxv()
{
  return 0;
}

template<int DIM>
int LinearSpringForce<DIM>::numTildeK()
{
  return 0;
}

template<int DIM>
bool LinearSpringForce<DIM>::isParallelized()
{
  return false;
}

template<int DIM>
bool LinearSpringForce<DIM>::isPrecomputationParallelized()
{
  return false;
}

template<int DIM>
int LinearSpringForce<DIM>::numConstraintPos()
{
  return DIM;
}

template<int DIM>
int LinearSpringForce<DIM>::numConstraintVel()
{
  return m_b > 0.0 ? DIM : 0;
}

template<int DIM>
Force* LinearSpringForce<DIM>::createNewCopy()
{
  return new LinearSpringForce(*this);
}

template<int DIM>
const char* LinearSpringForce<DIM>::name()
{
  return "linearspringforce";
}

template<int DIM>
void LinearSpringForce<DIM>::getAffectedVars( int colidx, std::unordered_set<int>& vars )
{
  int idir = m_scene->getComponent( colidx );
  if( idir == DIM ) return;
  int ip = m_scene->getVertFromDof( colidx );

  if(ip == m_endpoints.first || ip == m_endpoints.second) {
    for(int r = 0; r < DIM; ++r) {
      vars.insert( m_scene->getDof(m_endpoints.first) + r);
      vars.insert( m_scene->getDof(m_endpoints.second) + r);
    }
  }
}

template<int DIM>
int LinearSpringForce<DIM>::getAffectedHair( const std::vector<int> particle_to_hairs )
{
  return particle_to_hairs[m_endpoints.first];
}

template<int DIM>
bool LinearSpringForce<DIM>::isContained( int colidx )
{
  int idir = m_scene->getComponent( colidx );
  if( idir == DIM ) return false;
  int ip = m_scene->getVertFromDof( colidx );

  if(ip == m_endpoints.first || ip == m_endpoints.second) return true;
  else return false;
}


template<int DIM>
void LinearSpringForce<DIM>::storeLambda(const VectorXs& lambda, const VectorXs& lambda_v)
{
  m_lambda = lambda.segment<DIM>(m_internal_index_pos);
  if(m_b > 0.0) m_lambda_v = lambda_v.segment<DIM>(m_internal_index_vel);
}

// explicit instantiations at bottom
template class LinearSpringForce<2>;
template class LinearSpringForce<3>;
