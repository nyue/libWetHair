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

#ifndef __WET_HAIR_CORE_H__
#define __WET_HAIR_CORE_H__

#include "ExecutableSimulation.h"

#include "TwoDScene.h"
#include "TwoDSceneRenderer.h"
#include "SceneStepper.h"
#include "TwoDSceneSerializer.h"
#include "TwoDimensionalDisplayController.h"

template<int DIM>
class WetHairCore
{
public:
  
  WetHairCore( TwoDScene<DIM>* scene, SceneStepper<DIM>* scene_stepper, std::function<bool(double)>&& script_callback);
  ~WetHairCore();
  /////////////////////////////////////////////////////////////////////////////
  // Simulation Control Functions
  
  virtual void stepSystem();

  /////////////////////////////////////////////////////////////////////////////
  // Status Functions
  
  virtual void getBoundingBox(Vectors<DIM>& bb_min, Vectors<DIM>& bb_max);

  virtual const std::vector<scalar>& getTimingStatistics() const;
  virtual const std::vector<scalar>& getStepperTimingStatistics() const;
  
  virtual TwoDScene<DIM>* getScene() const;
  
  virtual const scalar& getCurrentTime() const;
  
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
  TwoDScene<DIM>* m_scene;
  
  SceneStepper<DIM>* m_scene_stepper;
  
  std::vector<scalar> m_timing_statistics;
  std::function<bool(double)> m_script_callback;
  
  scalar m_current_time;
};

#endif
