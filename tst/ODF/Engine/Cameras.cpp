// Google Test Includes
#include <gtest/gtest.h>

// ODF Includes
#include <ODF/Math/Vector.h>
#include <ODF/Engine/Camera.h>
#include <ODF/Engine/OrthographicCamera.h>

// StdLib Includes
#include <string>
#include <sstream>

TEST( Engine, OrthgraphicCameraStreams )
{
   static const std::string expectedInitial = 
      "\"OrthographicCamera\":{"
      "\"position\":[0,0,1],"
      "\"pointOfInterest\":[0,0,0],"
      "\"up\":[0,1,0],"
      "\"size\":100}\n";
   static const std::string expectedUpdate = 
      "\"OrthographicCamera\":{"
      "\"position\":[5.2,6.7,3.2],"
      "\"pointOfInterest\":[1.2,2.3,3.4],"
      "\"up\":[0,0,1],"
      "\"size\":200}\n";
   ODF::Engine::Camera* camera = new ODF::Engine::OrthographicCamera( );

   std::string str;
   std::stringstream streamIn( expectedUpdate );
   std::stringstream streamOut;
   streamOut << *camera;
   str = streamOut.str( );
   EXPECT_EQ( expectedInitial, str );
   streamIn >> *camera;
   streamOut.str( std::string( ) );
   streamOut << *camera;
   str = streamOut.str( );
   EXPECT_EQ( expectedUpdate, str );
}
