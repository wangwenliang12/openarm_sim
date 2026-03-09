# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/wwl/openarm_sim_cpp/build/_deps/mujoco_download-src"
  "/home/wwl/openarm_sim_cpp/build/_deps/mujoco_download-build"
  "/home/wwl/openarm_sim_cpp/build/_deps/mujoco_download-subbuild/mujoco_download-populate-prefix"
  "/home/wwl/openarm_sim_cpp/build/_deps/mujoco_download-subbuild/mujoco_download-populate-prefix/tmp"
  "/home/wwl/openarm_sim_cpp/build/_deps/mujoco_download-subbuild/mujoco_download-populate-prefix/src/mujoco_download-populate-stamp"
  "/home/wwl/openarm_sim_cpp/build/_deps/mujoco_download-subbuild/mujoco_download-populate-prefix/src"
  "/home/wwl/openarm_sim_cpp/build/_deps/mujoco_download-subbuild/mujoco_download-populate-prefix/src/mujoco_download-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/wwl/openarm_sim_cpp/build/_deps/mujoco_download-subbuild/mujoco_download-populate-prefix/src/mujoco_download-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/wwl/openarm_sim_cpp/build/_deps/mujoco_download-subbuild/mujoco_download-populate-prefix/src/mujoco_download-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
