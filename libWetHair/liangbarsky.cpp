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


#include "liangbarsky.h"
#include "MathDefs.h"

namespace liangbarsky
{
  int clip_line(const Vector4s& c, Vector2s& q1, Vector2s& q2, scalar& t0, scalar& t1)
  {
    t0 = 0.0;
    t1 = 1.0;
    double xdelta = q2(0)-q1(0);
    double ydelta = q2(1)-q1(1);
    double p=1.0,q=0.0,r;
    
    for(int edge=0; edge<4; edge++) {   // Traverse through left, right, bottom, top edges.
      if (edge==0) {  p = -xdelta;    q = -(c(0)-q1(0));  }
      if (edge==1) {  p = xdelta;     q =  (c(2)-q1(0)); }
      if (edge==2) {  p = -ydelta;    q = -(c(1)-q1(1));}
      if (edge==3) {  p = ydelta;     q =  (c(3)-q1(1));   }
      r = q/p;
      if(p==0 && q<0) return false;   // Don't draw line at all. (parallel line outside)
      
      if(p<0) {
        if(r>t1) return false;         // Don't draw line at all.
        else if(r>t0) t0=r;            // Line is clipped!
      } else if(p>0) {
        if(r<t0) return false;      // Don't draw line at all.
        else if(r<t1) t1=r;         // Line is clipped!
      }
    }
    
    Vector2s tq0 = q1;
    
    q1 = tq0 + t0 * Vector2s(xdelta, ydelta);
    q2 = tq0 + t1 * Vector2s(xdelta, ydelta);

    return true;        // (clipped) line is drawn
  }
  
  int clip_line(const Vector6s& c, Vector3s& q1, Vector3s& q2, scalar& t0, scalar& t1)
  {
    t0 = 0.0;
    t1 = 1.0;
    double xdelta = q2(0)-q1(0);
    double ydelta = q2(1)-q1(1);
    double zdelta = q2(2)-q1(2);
    double p=1.0,q=0.0,r;
    
    for(int edge=0; edge<6; edge++) {   // Traverse through left, right, bottom, top edges.
      if (edge==0) {  p = -xdelta;    q = -(c(0)-q1(0));  }
      if (edge==1) {  p = xdelta;     q =  (c(3)-q1(0)); }
      if (edge==2) {  p = -ydelta;    q = -(c(1)-q1(1));}
      if (edge==3) {  p = ydelta;     q =  (c(4)-q1(1));   }
      if (edge==4) {  p = -zdelta;    q = -(c(2)-q1(2));}
      if (edge==5) {  p = zdelta;     q =  (c(5)-q1(2));   }
      
      r = q/p;
      if(p==0 && q<0) return false;   // Don't draw line at all. (parallel line outside)
      
      if(p<0) {
        if(r>t1) return false;         // Don't draw line at all.
        else if(r>t0) t0=r;            // Line is clipped!
      } else if(p>0) {
        if(r<t0) return false;      // Don't draw line at all.
        else if(r<t1) t1=r;         // Line is clipped!
      }
    }
    
    Vector3s tq0 = q1;
    
    q1 = tq0 + t0 * Vector3s(xdelta, ydelta, zdelta);
    q2 = tq0 + t1 * Vector3s(xdelta, ydelta, zdelta);
    
    return true;        // (clipped) line is drawn
  }
}

