# The script detects Intel(R) Inference Engine installation
#
# Cache variables:
# INF_ENGINE_RELEASE - a number reflecting IE source interface (linked with OpenVINO release)
#
# Detect parameters:
# 1. Native cmake IE package:
#    - environment variable InferenceEngine_DIR is set to location of cmake module
# 2. Custom location:
#    - INF_ENGINE_INCLUDE_DIRS - headers search location
#    - INF_ENGINE_LIB_DIRS     - library search location
# 3. OpenVINO location:
#    - environment variable INTEL_OPENVINO_DIR is set to location of OpenVINO installation dir
#    - INF_ENGINE_PLATFORM - part of name of library directory representing its platform
#
# Result:
# INF_ENGINE_TARGET - set to name of imported library target representing InferenceEngine
#

if(NOT HAVE_CXX11)
    message(WARNING "DL Inference engine requires C++11. You can turn it on via ENABLE_CXX11=ON CMake flag.")
    return()
endif()

# =======================

macro(ocv_ie_find_extra_libraries find_prefix find_suffix)
  file(GLOB libraries "${INF_ENGINE_LIB_DIRS}/${find_prefix}inference_engine*${find_suffix}")
  foreach(full_path IN LISTS libraries)
    get_filename_component(library "${full_path}" NAME_WE)
    string(REPLACE "${find_prefix}" "" library "${library}")
    if(library STREQUAL "inference_engine" OR library STREQUAL "inference_engined")
      # skip
    else()
      add_library(${library} UNKNOWN IMPORTED)
      set_target_properties(${library} PROPERTIES
          IMPORTED_LOCATION "${full_path}")
      list(APPEND custom_libraries ${library})
    endif()
  endforeach()
endmacro()

function(add_custom_ie_build _inc _lib _lib_rel _lib_dbg _msg)
  if(NOT _inc OR NOT (_lib OR _lib_rel OR _lib_dbg))
    return()
  endif()
  if(NOT _lib)
    if(_lib_rel)
      set(_lib "${_lib_rel}")
    else()
      set(_lib "${_lib_dbg}")
    endif()
  endif()
  add_library(inference_engine UNKNOWN IMPORTED)
  set_target_properties(inference_engine PROPERTIES
    IMPORTED_LOCATION "${_lib}"
    IMPORTED_IMPLIB_RELEASE "${_lib_rel}"
    IMPORTED_IMPLIB_DEBUG "${_lib_dbg}"
    INTERFACE_INCLUDE_DIRECTORIES "${_inc}"
  )

  set(custom_libraries "")
  set(__prefixes "${CMAKE_FIND_LIBRARY_PREFIXES}")
  if(NOT __prefixes)
    set(__prefixes "_empty_")
  endif()
  foreach(find_prefix ${__prefixes})
    if(find_prefix STREQUAL "_empty_")  # foreach doesn't iterate over empty elements
      set(find_prefix "")
    endif()
    if(NOT DEFINED INFERENCE_ENGINE_FIND_LIBRARY_SUFFIXES)  # allow custom override
      set(INFERENCE_ENGINE_FIND_LIBRARY_SUFFIXES ${CMAKE_FIND_LIBRARY_SUFFIXES})
      if(APPLE)
        ocv_list_filterout(INFERENCE_ENGINE_FIND_LIBRARY_SUFFIXES "^.so$")  # skip plugins (can't be linked)
      endif()
    endif()
    foreach(find_suffix ${INFERENCE_ENGINE_FIND_LIBRARY_SUFFIXES})
      ocv_ie_find_extra_libraries("${find_prefix}" "${find_suffix}")
    endforeach()
    if(NOT CMAKE_FIND_LIBRARY_SUFFIXES)
      ocv_ie_find_extra_libraries("${find_prefix}" "")
    endif()
  endforeach()

  if(NOT INF_ENGINE_RELEASE VERSION_GREATER "2018050000")
    find_library(INF_ENGINE_OMP_LIBRARY iomp5 PATHS "${INF_ENGINE_OMP_DIR}" NO_DEFAULT_PATH)
    if(NOT INF_ENGINE_OMP_LIBRARY)
      message(WARNING "OpenMP for IE have not been found. Set INF_ENGINE_OMP_DIR variable if you experience build errors.")
    endif()
  endif()
  if(EXISTS "${INF_ENGINE_OMP_LIBRARY}")
    set_target_properties(inference_engine PROPERTIES IMPORTED_LINK_INTERFACE_LIBRARIES "${INF_ENGINE_OMP_LIBRARY}")
  endif()
  set(INF_ENGINE_VERSION "Unknown" CACHE STRING "")
  set(INF_ENGINE_TARGET "inference_engine;${custom_libraries}" PARENT_SCOPE)
  message(STATUS "Detected InferenceEngine: ${_msg}")
endfunction()

# ======================

find_package(InferenceEngine QUIET)
if(InferenceEngine_FOUND)
  set(INF_ENGINE_TARGET ${InferenceEngine_LIBRARIES})
  set(INF_ENGINE_VERSION "${InferenceEngine_VERSION}" CACHE STRING "")
  message(STATUS "Detected InferenceEngine: cmake package (${InferenceEngine_VERSION})")
endif()

if(DEFINED InferenceEngine_VERSION)
  message(STATUS "InferenceEngine: ${InferenceEngine_VERSION}")
  if(NOT INF_ENGINE_RELEASE AND NOT (InferenceEngine_VERSION VERSION_LESS "2021.4"))
    math(EXPR INF_ENGINE_RELEASE_INIT "${InferenceEngine_VERSION_MAJOR} * 1000000 + ${InferenceEngine_VERSION_MINOR} * 10000 + ${InferenceEngine_VERSION_PATCH} * 100")
  endif()
endif()
if(NOT INF_ENGINE_RELEASE AND NOT INF_ENGINE_RELEASE_INIT)
  message(STATUS "WARNING: InferenceEngine version has not been set, 2021.4.2 will be used by default. Set INF_ENGINE_RELEASE variable if you experience build errors.")
  set(INF_ENGINE_RELEASE_INIT "2021040200")
elseif(DEFINED INF_ENGINE_RELEASE)
  set(INF_ENGINE_RELEASE_INIT "${INF_ENGINE_RELEASE}")
endif()
set(INF_ENGINE_RELEASE "${INF_ENGINE_RELEASE_INIT}" CACHE STRING "Force IE version, should be in form YYYYAABBCC (e.g. 2020.1.0.2 -> 2020010002)")

if(NOT INF_ENGINE_TARGET AND INF_ENGINE_LIB_DIRS AND INF_ENGINE_INCLUDE_DIRS)
  find_path(ie_custom_inc "inference_engine.hpp" PATHS "${INF_ENGINE_INCLUDE_DIRS}" NO_DEFAULT_PATH)
  if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    find_library(ie_custom_lib_dbg "inference_engined" PATHS "${INF_ENGINE_LIB_DIRS}" NO_DEFAULT_PATH)  # Win32 and MacOSX
  endif()
  find_library(ie_custom_lib "inference_engine" PATHS "${INF_ENGINE_LIB_DIRS}" NO_DEFAULT_PATH)
  find_library(ie_custom_lib_rel "inference_engine" PATHS "${INF_ENGINE_LIB_DIRS}/Release" NO_DEFAULT_PATH)
  find_library(ie_custom_lib_dbg "inference_engine" PATHS "${INF_ENGINE_LIB_DIRS}/Debug" NO_DEFAULT_PATH)
  add_custom_ie_build("${ie_custom_inc}" "${ie_custom_lib}" "${ie_custom_lib_rel}" "${ie_custom_lib_dbg}" "INF_ENGINE_{INCLUDE,LIB}_DIRS")
endif()

set(_loc "$ENV{INTEL_OPENVINO_DIR}")
if(NOT _loc AND DEFINED ENV{INTEL_CVSDK_DIR})
  set(_loc "$ENV{INTEL_CVSDK_DIR}")  # OpenVINO 2018.x
endif()
if(NOT INF_ENGINE_TARGET AND _loc)
  if(NOT INF_ENGINE_RELEASE VERSION_GREATER "2018050000")
    set(INF_ENGINE_PLATFORM_DEFAULT "ubuntu_16.04")
  else()
    set(INF_ENGINE_PLATFORM_DEFAULT "")
  endif()
  set(INF_ENGINE_PLATFORM "${INF_ENGINE_PLATFORM_DEFAULT}" CACHE STRING "InferenceEngine platform (library dir)")
  find_path(ie_custom_env_inc "inference_engine.hpp" PATHS "${_loc}/deployment_tools/inference_engine/include" NO_DEFAULT_PATH)
  if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    find_library(ie_custom_env_lib_dbg "inference_engined" PATHS "${_loc}/deployment_tools/inference_engine/lib/${INF_ENGINE_PLATFORM}/intel64" NO_DEFAULT_PATH)
  endif()
  find_library(ie_custom_env_lib "inference_engine" PATHS "${_loc}/deployment_tools/inference_engine/lib/${INF_ENGINE_PLATFORM}/intel64" NO_DEFAULT_PATH)
  find_library(ie_custom_env_lib_rel "inference_engine" PATHS "${_loc}/deployment_tools/inference_engine/lib/intel64/Release" NO_DEFAULT_PATH)
  find_library(ie_custom_env_lib_dbg "inference_engine" PATHS "${_loc}/deployment_tools/inference_engine/lib/intel64/Debug" NO_DEFAULT_PATH)
  add_custom_ie_build("${ie_custom_env_inc}" "${ie_custom_env_lib}" "${ie_custom_env_lib_rel}" "${ie_custom_env_lib_dbg}" "OpenVINO (${_loc})")
endif()

# Add more features to the target

if(INF_ENGINE_TARGET)
  set_target_properties(${INF_ENGINE_TARGET} PROPERTIES
      INTERFACE_COMPILE_DEFINITIONS "HAVE_INF_ENGINE=1;INF_ENGINE_RELEASE=${INF_ENGINE_RELEASE}"
  )
endif()

if(WITH_NGRAPH)
  find_package(ngraph QUIET)
  if(ngraph_FOUND)
    ocv_assert(TARGET ngraph::ngraph)
    if(INF_ENGINE_RELEASE VERSION_LESS "2019039999")
      message(WARNING "nGraph is not tested with current InferenceEngine version: INF_ENGINE_RELEASE=${INF_ENGINE_RELEASE}")
    endif()
    message(STATUS "Detected ngraph: cmake package (${ngraph_VERSION})")
    set(HAVE_NGRAPH ON)
  endif()
endif()
