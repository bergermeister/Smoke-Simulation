/**
 * @file Matrix.cpp
 * @author Eisenberger, Edward (Edward.Eisenberger@live.com)
 * @brief 
 * @version 0.1
 * @date 2022-08-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */
// Google Test Includes
#include <gtest/gtest.h>

// ODF Includes
#include <ODF/Math/Vector.h>

TEST( Math, Vector3DOperations )
{
   const ODF::Math::Vector< 3 > expected0( { 1.0, 2.0, 3.0 } );
   const ODF::Math::Vector< 3 > expected1( { 2.0, 4.0, 6.0 } );
   const ODF::Math::Vector< 3 > expected2( { 4.0, 8.0, 12.0 } );
   

   ODF::Math::Vector< 3 > vec3 = expected0;
   vec3 += vec3;
   EXPECT_EQ( expected1, vec3 );
   vec3 *= 2.0;
   EXPECT_EQ( expected2, vec3 );
   vec3 /= 2.0;
   EXPECT_EQ( expected1, vec3 );
   vec3 -= ODF::Math::Vector< 3 >( { 1.0, 2.0, 3.0 } );
   EXPECT_EQ( expected0, vec3 );
}
