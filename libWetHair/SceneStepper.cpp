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


#include "SceneStepper.h"

template<int DIM>
SceneStepper<DIM>::~SceneStepper()
{}

template<int DIM>
const VectorXs& SceneStepper<DIM>::getAcceleration() const
{
  return m_a;
}

template<int DIM>
void SceneStepper<DIM>::accept( TwoDScene<DIM> & scene, scalar dt )
{
  VectorXs& x = scene.getX();
  VectorXs& v = scene.getV();
  
  v = (m_next_x - x) / dt;
  x = m_next_x;
  if( scene.getParameter().no_fictitious ) {
    SceneStepper<DIM>::m_a.setZero();
  } else {
    SceneStepper<DIM>::m_a = (v - m_old_v) / dt;
  }
  m_old_v = v;
  
  stringutils::print(MAKE_REF(SceneStepper<DIM>::m_a));
  
  mathutils::check_isnan("SIM: v", v);
  mathutils::check_isnan("SIM: x", x);
}

template<int DIM>
void SceneStepper<DIM>::setNextX( const VectorXs& nextx )
{
  m_next_x = nextx;
}

template<int DIM>
const VectorXs& SceneStepper<DIM>::getNextX() const
{
  return m_next_x;
}

template<int DIM>
void SceneStepper<DIM>::PostStepScene( TwoDScene<DIM> & scene, scalar dt )
{
  scene.postCompute(dt);
}

template<int DIM>
void SceneStepper<DIM>::init( TwoDScene<DIM> & scene )
{
  m_old_v = scene.getV();
  m_a.resize(m_old_v.size());
  m_a.setZero();
}

template<int DIM>
const std::vector<scalar>& SceneStepper<DIM>::getTimingStatistics() const
{
  return m_timing_statistics;
}

template<int DIM>
void SceneStepper<DIM>::write(std::vector<scalar>&) const {};

template<int DIM>
void SceneStepper<DIM>::read(const scalar* data) {};

template<int DIM>
size_t SceneStepper<DIM>::size() {return 0;};

// explicit instantiations at bottom
template class SceneStepper<2>;
template class SceneStepper<3>;
