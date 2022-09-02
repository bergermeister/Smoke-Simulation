/**
 * @file Main.cpp
 * @author Eisenberger, Edward (Edward.Eisenberger@live.com)
 * @brief 
 * @version 0.1
 * @date 2022-08-24
 * 
 * @copyright Copyright (c) 2022
 * 
 */
// StdLib Includes
#include <iostream>

// ODF Includes
#include <ODF/Math/Vector.h>
//#include <ODF/Primitive/Camera.h>

int main( int argc, char** argv )
{
   int status = 0;
   ODF::Math::Vector< 4 > vec4( { } );
   ODF::Math::Vector< 3 > vec3( { 1.0, 2.0, 3.0 } );

   std::cout << "OpenGL Sandbox Application" << std::endl;
   std::cout << vec3;
   vec3 += vec3;
   std::cout << vec3;
   vec3 *= 2.0;
   std::cout << vec3;
   vec3 /= 2.0;
   std::cout << vec3;
   vec3 -= ODF::Math::Vector< 3 >( { 1.0, 2.0, 3.0 } );
   std::cout << vec3;

   return( status );
}

