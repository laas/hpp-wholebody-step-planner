// Copyright (C) 2009 by Sebastien Dalibard.
//
// This file is part of the hpp-wholebody-step-planner.
//
// hpp-wholebody-step-planner is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// hpp-wholebody-step-planner is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with hpp-wholebody-step-planner.  If not, see <http://www.gnu.org/licenses/>.

/// \mainpage
///
/// This package implements algorithms to plan walking and whole-body motions
/// satisfying a goal task for a humanoid robot.
/// The main algorithm implemented by hpp::wholeBodyStepPlanner::Planner and
/// described in <a href="http://hal.archives-ouvertes.fr/hal-00654175/PDF/paper.pdf">this paper</a> proceeds in two steps:
/// \li first a sliding motion is computed for the humanoid robot. Along this
/// motion, the feet are constrained to remain on the ground with a constant
/// relative position and the center of mass is constrained to project
/// vertically at a point rigidly fixed to the feet.
/// \li in a second step, the motion is approximated by dynamic walking motions.
/// While a part of dynamic walking motion is in collision, the length and the
/// duration of the steps are shorten.

