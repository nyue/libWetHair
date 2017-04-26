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

#ifndef CAMERA_H
#define CAMERA_H
#include <Eigen/Dense>
#include "MathDefs.h"

class Camera
{
public:
  Eigen::Quaterniond rotation_;	// rotation
  Eigen::Vector3d center_;	// center point
  double	 dist_;			// point of view to center
  double   radius_;		// bounding sphere
  double   fov_;			// angle
  Camera ( const Camera& that );
  void operator = ( const Camera& that );

  explicit Camera ( const double fov = 40 );
  explicit Camera ( Eigen::Quaterniond& rot, Eigen::Vector3d& center, const double& dist, const double& radius, const double& fov );
  void init( const Eigen::Vector3d& bmin, const Eigen::Vector3d& bmax );
  void clone ( const Camera& that );
  void getLookAt( Eigen::Vector3d& eye, Eigen::Vector3d& center, Eigen::Vector3d& up ) const;
  void getEye( Eigen::Vector3d& eye ) const;
  void getPerspective( double& fov, double& zNear, double& zFar ) const;
  void rotate( const double  oldx, const double oldy, const double newx, const double newy );
  void zoom( const double  oldx, const double oldy, const double newx, const double newy );
  void pan( const double  oldx, const double oldy, const double newx, const double newy );
  void rotate( const Vector3s& axis, const scalar& angle, bool global );
  void project_to_sphere( const double& radius, Eigen::Vector3d& p ) const;
  friend std::ostream &operator<<( std::ostream &output, const Camera &cam );
};
#endif//CAMERA_H
