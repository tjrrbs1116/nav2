/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000  Brian Gerkey   &  Kasper Stoy
 *                      gerkey@usc.edu    kaspers@robotics.usc.edu
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef PIOT_LOCALIZATION__MOTION_MODEL__DIFFERENTIAL_MOTION_MODEL_HPP_
#define PIOT_LOCALIZATION__MOTION_MODEL__DIFFERENTIAL_MOTION_MODEL_HPP_

#include <sys/types.h>
#include <math.h>
#include <algorithm>
#include "piot_localization/motion_model/motion_model.hpp"
#include "piot_localization/angleutils.hpp"


namespace piot_localization
{

class DifferentialMotionModel : public piot_localization::MotionModel
{
public:
  virtual void initialize(
    double alpha1, double alpha2, double alpha3, double alpha4,
    double alpha5);
  virtual void odometryUpdate(pf_t * pf, const pf_vector_t & pose, const pf_vector_t & delta);

private:
  double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
};
}  // namespace piot_localization
#endif  // PIOT_LOCALIZATION__MOTION_MODEL__DIFFERENTIAL_MOTION_MODEL_HPP_
