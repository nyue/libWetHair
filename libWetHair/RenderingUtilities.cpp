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


#include "RenderingUtilities.h"

namespace renderingutils
{

bool checkGLErrors()
{
  /*
  GLenum errCode;
  const GLubyte *errString;
  
  if ((errCode = glGetError()) != GL_NO_ERROR) 
  {
    errString = gluErrorString(errCode);
    std::cout << outputmod::startred << "OpenGL Error:" << outputmod::endred << std::flush;
    fprintf(stderr, " %s\n", errString);
    return false;
  }*/
  return true;
}    
  
Color::Color( const Vector3s& c )
:r(c(0)), g(c(1)), b(c(2))
{}

Color::Color()
: r(0.0), g(0.0), b(0.0)
{}

Color::Color( double r, double g, double b )
: r(r), g(g), b(b)
{
  assert( r >= 0.0 ); assert( r <= 1.0 );
  assert( g >= 0.0 ); assert( g <= 1.0 );
  assert( b >= 0.0 ); assert( b <= 1.0 );
}

Vector3s Color::toVector() const
{
  return Vector3s(r, g, b);
}

}
