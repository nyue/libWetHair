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


#ifndef OPENGL_UTILS_H
#define OPENGL_UTILS_H

#include <vector>
#include "MathDefs.h"
#include "array2.h"

void draw_circle2d(const Vector2s& centre, scalar rad, int segs);
void draw_grid2d(const Vector2s& origin, scalar dx, int nx, int ny);
void draw_grid3d(const Vector3s& origin, scalar dx, int nx, int ny, int nz);
void draw_box2d(const Vector2s& origin, scalar width, scalar height);
void draw_segmentset2d(const std::vector<Vector2s>& vertices, const std::vector<Vector2i>& edges);
void draw_points2d(const std::vector<Vector2s>& points);
void draw_polygon2d(const std::vector<Vector2s>& vertices);
void draw_polygon2d(const std::vector<Vector2s>& vertices, const std::vector<int>& order);
void draw_segment2d(const Vector2s& start, const Vector2s& end);
void draw_arrow2d(const Vector2s& start, const Vector2s& end, scalar arrow_head_len);
void draw_arrow3d(const Vector3s& start, const Vector3s& end, const Vector3s& view_dir, scalar arrow_head_len);
void draw_grid_data2d(Array2s& data, Vector2s origin, scalar dx, bool color = false);
void draw_trimesh2d(const std::vector<Vector2s>& vertices, const std::vector<Vector3i>& tris);    
   
void draw_trimesh3d(const std::vector<Vector3s>& vertices, const std::vector<Vector3i>& tris);
void draw_trimesh3d(const std::vector<Vector3s>& vertices, const std::vector<Vector3i>& tris, const Vector3s& center, const Vector3s& scaling, const Eigen::Quaternion<scalar>& rot);
void write_trimesh3d(std::ostream& o, const std::vector<Vector3s>& vertices, const std::vector<Vector3i>& tris, const Vector3s& center, const Vector3s& scaling, const Eigen::Quaternion<scalar>& rot, bool double_sided);
void draw_trimesh3d(const std::vector<Vector3s>& vertices, const std::vector<Vector3i>& tris, const std::vector<Vector3s>& normals, const Eigen::Quaternion<scalar>& rot);
void draw_box3d(const Vector3s& dimensions);
void draw_box3d(const Vector3s& dimensions, const Vector3s& center, const Eigen::Quaternion<scalar>& rot);
void write_box3d(std::ostream& o, const Vector3s& dimensions, const Vector3s& center, const Eigen::Quaternion<scalar>& rot, bool double_sided);
void write_box3d(std::vector<Vector3s>& vertices, std::vector<Vector3i>& tris);

#endif
