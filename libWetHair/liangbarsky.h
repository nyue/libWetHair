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


#ifndef LIANG_BARSKY_H__
#define LIANG_BARSKY_H__

/* Liang-Barsky clipping algorithm.
 */

#include "MathDefs.h"

namespace liangbarsky {
  /* clip_line()
   * modifies parameters in place to clip the line,
   * returns 0 if line is totally outside clip window
   * returns 1 if line is not totally outside clip window
   */
  int clip_line(const Vector4s& c, Vector2s& q1, Vector2s& q2, scalar& alpha0, scalar& alpha1);
  int clip_line(const Vector6s& c, Vector3s& q1, Vector3s& q2, scalar& t0, scalar& t1);
};

#endif
