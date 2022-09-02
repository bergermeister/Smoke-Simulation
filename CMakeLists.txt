cmake_minimum_required( VERSION 3.15 )
project( ODF VERSION 0.0.1 )

set( CMAKE_CXX_STANDARD 14 )
set( CMAKE_CXX_STANDARD_REQUIRED ON )

#if( BUILD_TESTING )
   # -pg    enable profiling
   #add_compile_options( -fprofile-arcs -ftest-coverage --coverage -pg --no-pie -no-pie -fno-pie )
   #add_link_options( -fprofile-arcs --coverage -pg --no-pie -no-pie -fno-pie )
   #set( CMAKE_CXX_ARCHIVE_CREATE "<CMAKE_AR> qc -fprofile-arcs --coverage <TARGET> <OBJECTS>" )
#endif( )

add_library( OpenGLFramework STATIC )
target_include_directories( OpenGLFramework
   PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
      
   PUBLIC
      # where top-level project will look for the library's public headers
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
      # where external projects will look for the library's public headers
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_executable( OpenGLApplication )
target_include_directories( OpenGLApplication
   PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src

   PUBLIC
      # where top-level project will look for the library's public headers
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
      # where external projects will look for the library's public headers
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

add_subdirectory( src )

target_link_libraries(
   OpenGLApplication
   OpenGLFramework
)

if( BUILD_TESTING )
   target_link_libraries(
      OpenGLApplication
      gcov
   )

   include(FetchContent)
   FetchContent_Declare(
      googletest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG release-1.12.1
   )
   # For Windows: Prevent overriding the parent project's compiler/linker settings
   set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
   FetchContent_MakeAvailable( googletest )

   enable_testing( )

   add_executable( GTestOpenGLFramework )
   add_subdirectory( tst )
   target_include_directories( GTestOpenGLFramework
      PRIVATE
         ${CMAKE_CURRENT_SOURCE_DIR}

      PUBLIC
         # where top-level project will look for the library's public headers
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
         # where external projects will look for the library's public headers
         $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
   )
   target_link_libraries(
      GTestOpenGLFramework
      GTest::gtest_main
      OpenGLFramework
      gcov
   )

   include( GoogleTest )
   gtest_discover_tests( GTestOpenGLFramework )
endif( )

