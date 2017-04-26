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


#include "Force.h"

Force::~Force()
{}

void Force::preCompute( const VectorXs& x, const VectorXs& v, const VectorXs& m, const scalar& dt )
{}

bool Force::isInterHair() const
{
  return false;
}

void Force::storeLambda(const VectorXs& lambda, const VectorXs& lambda_v)
{}

void Force::postStepScene(const scalar& dt )
{}

int Force::getAffectedHair( const std::vector<int> particle_to_hairs )
{
  return -1;
}

bool Force::isExternal()
{
  return false;
}

void Force::setInternalIndex(int index_pos,
                             int index_vel,
                             int index_J,
                             int index_Jv,
                             int index_Jxv,
                             int index_tildeK)
{
  m_internal_index_pos = index_pos;
  m_internal_index_vel = index_vel;
  m_internal_index_J = index_J;
  m_internal_index_Jv = index_Jv;
  m_internal_index_Jxv = index_Jxv;
  m_internal_index_tildeK = index_tildeK;
}
