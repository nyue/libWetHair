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

#include "sorter.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <tbb/tbb.h>

using namespace std;

Sorter::Sorter( int ni_, int nj_, int nk_ )
: ni(ni_), nj(nj_), nk(nk_)
{
  resize(ni, nj, nk);
}

Sorter::~Sorter() {
}

void Sorter::resize( int ni_, int nj_, int nk_ )
{
  array_sup.resize(ni_ * nj_ * nk_);
  ni = ni_; nj = nj_; nk = nk_;
}
