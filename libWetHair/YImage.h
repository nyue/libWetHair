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


#ifndef __YImage_h__
#define __YImage_h__

// file loading/saving automatically picks up changes to this struct.
// The possibilities are: ARGB, ABGR, RGBA, BGRA.

struct YPixel
{
  unsigned char r;
  unsigned char g;
  unsigned char b;
  unsigned char a;
};

class YImage
{
public:
  YImage();

  YImage(const YImage&);

  virtual ~YImage();

  YImage& operator=(const YImage&);

  bool save(const char* fname) const;

  bool load(const char* fname);

  YPixel* data();

  const YPixel* data() const;

  YPixel& at(int i, int j);

  const YPixel& at(int i, int j) const;

  int width() const;

  int height() const;

  void resize(int width, int height);

  // flip vertically
  void flip();

  // flip horizontally
  void mirror();

  // average rgb
  void greyscale();

protected:
  int m_width;
  int m_height;
  YPixel* m_data; // raw image data
};

#endif /* __YImage_h__ */

