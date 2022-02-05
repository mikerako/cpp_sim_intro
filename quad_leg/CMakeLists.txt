# -*- mode: cmake -*-
# vi: set ft=cmake :

# Copyright (c) 2017, Massachusetts Institute of Technology.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Send over the SDF file so the particle example can be run "in place."
# configure_file(models/urdf/leg.urdf "${CMAKE_CURRENT_BINARY_DIR}" COPYONLY)

add_library(quad_leg quad_leg_plant.cc quad_leg_plant.h quad_leg_geometry.cc quad_leg_geometry.h)
target_link_libraries(quad_leg drake::drake)

set(GFLAGS_USE_TARGET_NAMESPACE OFF)
find_package(gflags CONFIG REQUIRED)

FIND_PACKAGE( Boost 1.40 COMPONENTS program_options REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

# executables: run_quadrotor_dynamics.cc run_quadrotor_lqr.cc

# add_executable(run_quad_leg_openloop run_quad_leg_openloop.cpp)
# target_link_libraries(run_quad_leg_openloop quad_leg drake::drake gflags)

add_executable(run_quad_leg_openloop run_quad_leg_openloop.cpp)
target_link_libraries(run_quad_leg_openloop drake::drake gflags)

# add_executable(run_quad_leg_passive run_quad_leg_passive.cpp)
# target_link_libraries(run_quad_leg_passive drake::drake gflags -lboost_system)



# add_executable(particle_test particle_test.cc)
# target_include_directories(particle_test PRIVATE
#   "${CMAKE_CURRENT_SOURCE_DIR}"
# )
# target_link_libraries(particle_test
#   particles drake::drake gtest
# )
# add_test(NAME particle_test COMMAND particle_test)