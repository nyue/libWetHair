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

#ifndef CAPSULE_H__
#define CAPSULE_H__

/* Capsule generation algorithm.
 * Adapted from Paul Bourke's C implementation found here:
 * http://paulbourke.net/geometry/capsule/
 */

#include "MathDefs.h"
#include <vector>
#include <unordered_map>

class CapsuleCreator
{
public:
  std::vector<Vector3s> vertices;
  std::vector<Vector3i> indices;
  
  CapsuleCreator();
  void Create(int N, const scalar& radius, const scalar& halfheight);
};

#endif
