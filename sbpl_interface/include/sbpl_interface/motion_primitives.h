/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Michael Ferguson
 *  Copyright (c) 2008, Maxim Likhachev
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of University of Pennsylvania nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef _MOTION_PRIMITIVES_H_
#define _MOTION_PRIMITIVES_H_

namespace sbpl_interface
{

enum motion_primitive_types
{
  STATIC,
  STATIC_LOW_RES,
  SNAP_TO_XYZRPY,
  SNAP_TO_RPY
};

/** \brief Base class for a motion primitive */
class MotionPrimitive
{
public:
  MotionPrimitive(int type) : type_(type)
  {
  }

  virtual ~MotionPrimitive()
  {
  }

  virtual bool getSuccessorState(const std::vector<double>& start,
                                 std::vector<double>& end)
  {
    end = start;  // this is truly a stupid, worthless motion primitive
    return true;
  }

  int type()
  {
    return type_;
  }

protected:
  int type_;
};

class StaticMotionPrimitive : public MotionPrimitive
{
public:
  StaticMotionPrimitive(const std::vector<double>& action, 
                        bool low_res = false) :
    MotionPrimitive(STATIC),
    action_(action)
  {
    if (low_res)
      type_ = STATIC_LOW_RES;    
  }

  virtual bool getSuccessorState(const std::vector<double>& start,
                                 std::vector<double>& end)
  {
    end = start;
    for (size_t i = 0; i < action_.size(); ++i)
      end[i] += action_[i];
    return true;
  }

private:
  std::vector<double> action_;
};

// TODO: orientation solver primitive
// TODO: xyzrpy solver primitive

}  // namespace sbpl_interface

#endif  // _MOTION_PRIMITIVES_H_
