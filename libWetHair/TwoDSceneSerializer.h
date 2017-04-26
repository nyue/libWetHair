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


#ifndef __TWO_D_SCENE_SERIALIZER_H__
#define __TWO_D_SCENE_SERIALIZER_H__

#include <fstream>
#include <iostream>

#include "StringUtilities.h"

template<int DIM>
class TwoDScene;

template<int DIM>
class SceneStepper;

template<int DIM>
class TwoDSceneRenderer;

template<int DIM>
class TwoDSceneSerializer
{
public:
  void serializeFluidReadable( TwoDScene<DIM>& scene, std::vector< std::ostringstream >& outputstream ) const;
  
  void serializeHairReadable( TwoDScene<DIM>& scene, std::ostream& outputstream ) const;
  
  void serializeShallowFlowReadable( const TwoDSceneRenderer<DIM>* renderer, TwoDScene<DIM>& scene, std::ostream& outputstream ) const;
  
  void serializeBoundariesReadable( const TwoDSceneRenderer<DIM>* renderer, TwoDScene<DIM>& scene, std::ostream& os_boundary_single, std::ostream& os_boundary_double);
  
  void serializePolygonalCohesionReadable( const TwoDSceneRenderer<DIM>* renderer, TwoDScene<DIM>& scene, std::ostream& os_pe, std::ostream& os_poe, std::ostream& os_ppp ) const;
  
  bool deSerializeFluidReadable( TwoDScene<DIM>& scene, const std::vector< std::string > &filename_fluids );

  bool deSerializeHairReadable( TwoDScene<DIM>& scene, const std::string &filename_hairs );

  bool deSerializeShallowFlowReadable( const TwoDSceneRenderer<DIM>* renderer, TwoDScene<DIM>& scene, const std::string &filename_flows );
  
  void serializeScene( TwoDScene<DIM>& scene, SceneStepper<DIM>* stepper, std::ostream& outputstream ) const;

  void loadScene( TwoDScene<DIM>& scene, SceneStepper<DIM>* stepper, std::istream& inputstream ) const;
};

#endif
