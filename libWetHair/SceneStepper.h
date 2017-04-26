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


#ifndef __SCENE_STEPPER__
#define __SCENE_STEPPER__

#include "TwoDScene.h"

#include "MathDefs.h"

template<int DIM>
class SceneStepper
{
protected:
  std::vector<scalar> m_timing_statistics;
  
  VectorXs m_old_v;
  VectorXs m_a;
  VectorXs m_next_x;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  virtual ~SceneStepper();
  
  virtual bool stepScene( TwoDScene<DIM> & scene, scalar dt, bool updatePreCompute = true ) = 0;
  
  virtual void setNextX( const VectorXs& nextx );
  
  virtual const VectorXs& getNextX() const;
  
  virtual void accept( TwoDScene<DIM> & scene, scalar dt );
  
  virtual std::string getName() const = 0;
  
  virtual const VectorXs& getAcceleration() const;
  
  virtual void PostStepScene( TwoDScene<DIM> & scene, scalar dt );
  
  virtual const std::vector<scalar>& getTimingStatistics() const;
  
  virtual void write(std::vector<scalar>&) const;
  
  virtual void read(const scalar* data);
  
  virtual size_t size();
  
  virtual void init( TwoDScene<DIM> & scene );
};

#endif
