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


#ifndef ICOSPHERE_H__
#define ICOSPHERE_H__

/* Icosphere generation algorithm.
 * Adapted from Andreas Kahler's C# implementation found here:
 * http://blog.andreaskahler.com/2009/06/creating-icosphere-mesh-in-code.html
 */

#include "MathDefs.h"
#include <vector>
#include <unordered_map>

class IcosphereCreator
{
public:
  std::vector<Vector3s> vertices;
  std::vector<Vector3i> indices;
  std::unordered_map<uint64, int> middlePointIndexCache;
  int index;
  
  int addVertex(const Vector3s& p);
  int getMiddlePoint(int p1, int p2);

  IcosphereCreator();
  void Create(int recursionLevel);
};

#endif
