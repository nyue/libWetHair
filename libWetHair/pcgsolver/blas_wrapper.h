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


#ifndef BLAS_WRAPPER_H
#define BLAS_WRAPPER_H

// Simple placeholder code for BLAS calls - replace with calls to a real BLAS library

#include <vector>

namespace robertbridson {

namespace BLAS{
// dot products ==============================================================

inline double dot(const std::vector<double> &x, const std::vector<double> &y)
{
   double sum = 0;
   size_t n = x.size() < y.size() ? x.size() : y.size();
   for(size_t i = 0; i < n; ++i)
      sum += x[i]*y[i];
   return sum;
}

// inf-norm (maximum absolute value: index of max returned) ==================

inline int index_abs_max(const std::vector<double> &x)
{ 
   int maxind = 0;
   double maxvalue = 0;
   for(int i = 0; i < x.size(); ++i) {
      if(fabs(x[i]) > maxvalue) {
         maxvalue = fabs(x[i]);
         maxind = i;
      }
   }
   return maxind;
}

// inf-norm (maximum absolute value) =========================================
// technically not part of BLAS, but useful

inline double abs_max(const std::vector<double> &x)
{ return std::fabs(x[index_abs_max(x)]); }

// saxpy (y=alpha*x+y) =======================================================

inline void add_scaled(double alpha, const std::vector<double> &x, std::vector<double> &y)
{ 
   for(int i = 0; i < y.size(); ++i)
      y[i] += alpha*x[i];
}
}
}
#endif
