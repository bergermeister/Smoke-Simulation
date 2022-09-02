// Google Test Includes
#include <gtest/gtest.h>

// ODF Includes
#include <ODF/Math/Matrix.h>
#include <ODF/Math/SquareMatrix.h>
#include <ODF/Math/Matrix4x4.h>

TEST( Math, Matrix4x4 )
{
   using namespace ODF::Math;

   Matrix< Matrix4x4::Size, Matrix4x4::Size > mat0;
   SquareMatrix< Matrix4x4::Size > mat1;
   Matrix4x4 mat2;

   mat0 = SquareMatrix< Matrix4x4::Size >::Identity( );
   mat1 = SquareMatrix< Matrix4x4::Size >::Identity( );
   mat2 = SquareMatrix< Matrix4x4::Size >::Identity( );
   
}
