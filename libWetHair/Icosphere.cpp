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


#include "Icosphere.h"

IcosphereCreator::IcosphereCreator()
{
  index = 0;
}

int IcosphereCreator::addVertex(const Vector3s& p)
{
  vertices.push_back(p.normalized());
  return index++;
}

int IcosphereCreator::getMiddlePoint(int p1, int p2)
{
  int smallerIndex = std::min(p1, p2);
  int greaterIndex = std::max(p1, p2);
  
  uint64 key = ((uint64)(smallerIndex) << 32UL) | (uint64) greaterIndex;
  auto itr = middlePointIndexCache.find(key);
  if(itr != middlePointIndexCache.end())
  {
    return itr->second;
  }
  
  Vector3s middle = (vertices[p1] + vertices[p2]) * 0.5;
  int i = addVertex(middle);
  
  middlePointIndexCache[key] = i;
  return i;
}

void IcosphereCreator::Create(int recursionLevel)
{
  middlePointIndexCache.clear();
  vertices.clear();
  indices.clear();
  index = 0;
  
  const scalar t = (1.0 + sqrt(5.0)) / 2.0;
  
  addVertex(Vector3s(-1,  t,  0));
  addVertex(Vector3s( 1,  t,  0));
  addVertex(Vector3s(-1, -t,  0));
  addVertex(Vector3s( 1, -t,  0));
  
  addVertex(Vector3s( 0, -1,  t));
  addVertex(Vector3s( 0,  1,  t));
  addVertex(Vector3s( 0, -1, -t));
  addVertex(Vector3s( 0,  1, -t));
  
  addVertex(Vector3s( t,  0, -1));
  addVertex(Vector3s( t,  0,  1));
  addVertex(Vector3s(-t,  0, -1));
  addVertex(Vector3s(-t,  0,  1));
  
  // 5 faces around point 0
  indices.push_back(Vector3i(0, 11, 5));
  indices.push_back(Vector3i(0, 5, 1));
  indices.push_back(Vector3i(0, 1, 7));
  indices.push_back(Vector3i(0, 7, 10));
  indices.push_back(Vector3i(0, 10, 11));
  
  indices.push_back(Vector3i(1, 5, 9));
  indices.push_back(Vector3i(5, 11, 4));
  indices.push_back(Vector3i(11, 10, 2));
  indices.push_back(Vector3i(10, 7, 6));
  indices.push_back(Vector3i(7, 1, 8));
  
  indices.push_back(Vector3i(3, 9, 4));
  indices.push_back(Vector3i(3, 4, 2));
  indices.push_back(Vector3i(3, 2, 6));
  indices.push_back(Vector3i(3, 6, 8));
  indices.push_back(Vector3i(3, 8, 9));
  
  indices.push_back(Vector3i(4, 9, 5));
  indices.push_back(Vector3i(2, 4, 11));
  indices.push_back(Vector3i(6, 2, 10));
  indices.push_back(Vector3i(8, 6, 7));
  indices.push_back(Vector3i(9, 8, 1));
  
  // refine triangles
  for (int i = 0; i < recursionLevel; i++)
  {
    std::vector<Vector3i> indices2;
    for(const Vector3i& tri : indices)
    {
      // replace triangle by 4 triangles
      int a = getMiddlePoint(tri(0), tri(1));
      int b = getMiddlePoint(tri(1), tri(2));
      int c = getMiddlePoint(tri(2), tri(0));
      
      indices2.push_back(Vector3i(tri(0), a, c));
      indices2.push_back(Vector3i(tri(1), b, a));
      indices2.push_back(Vector3i(tri(2), c, b));
      indices2.push_back(Vector3i(a, b, c));
    }
    indices = indices2;
  }
}
