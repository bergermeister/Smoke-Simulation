// ODF Includes
#include <ODF/Engine/OrthographicCamera.h>

// StdLib Includes
#include <sstream>

namespace ODF
{
   namespace Engine
   {
      OrthographicCamera::OrthographicCamera( const Math::Vector< 3 >& Position,
                                              const Math::Vector< 3 >& POI,
                                              const Math::Vector< 3 >& Up,
                                              double Size )
         : Camera( Position, POI, Up )
      {
         this->size = Size;
      }

      void OrthographicCamera::GLInit( int Width, int Height ) 
      {
         this->width = Width;
         this->height = Height;
         glMatrixMode( GL_PROJECTION );
         glLoadIdentity( );
         double horiz = size / 2.0;
         double vert = size / 2.0;
         double aspect = double( Width ) / double( Height );
         if( aspect > 1 )
         { 
            vert /= aspect;
         }
         else 
         {
            horiz *= aspect;
         }
         double distToPOI = ( this->poi - this->pos ).Length( );
         glOrtho( -horiz, horiz, -vert, vert, distToPOI * 0.1, distToPOI * 10.0 );
      }

      void OrthographicCamera::Zoom( double Factor )
      {
         this->size *= pow( 1.005, Factor );
         this->GLInit( this->width, this->height );
      }

      Ray OrthographicCamera::GenerateRay( double X, double Y )
      {
         Math::Vector< 3 > screenCenter = this->pos;
         Math::Vector< 3 > xAxis = this->Horizontal( ) * size; 
         Math::Vector< 3 > yAxis = this->ScreenUp( ) * size; 
         Math::Vector< 3 > lowerLeft = screenCenter - ( xAxis * 0.5 ) - ( yAxis * 0.5 );
         Math::Vector< 3 > screenPoint = lowerLeft + ( xAxis * X ) + ( yAxis * Y );
         return( Ray( screenPoint,Direction( ) ) );
      }

      std::ostream& operator<<( std::ostream& OutStream, const OrthographicCamera& Cam )
      {
         OutStream << "\"OrthographicCamera\":{";
         OutStream << "\"position\":" << Cam.pos << ",";
         OutStream << "\"pointOfInterest\":" << Cam.poi << ",";
         OutStream << "\"up\":" << Cam.up << ",";
         OutStream << "\"size\":" << Cam.size << "}" << std::endl;
         return( OutStream );
      }

      std::istream& operator>>( std::istream& InStream, OrthographicCamera& Cam )
      {
         size_t index = 0;
         size_t position;
         std::string str;
         std::stringstream stream;

         /// @par
         /// -# Read the OrthographicCamera JSON object into the string object
         InStream >> str;
         position = str.find( '{', index );
         assert( str.substr( index, position - index + 1 ) == "\"OrthoGraphicCamera\":{" );
         index = position + 1;
         position = str.find( ':', index );
         assert( str.substr( index, position - index + 1 ) == "\"position\":" );
         index = position + 1;
         position = str.find( ',', index );
         stream.str( str.substr( index, position - index + 1 ) );
         stream >> Cam.pos;
         /*
         index = position;
         InStream >> token; assert( token == "pointOfInterest" );
         InStream >> token; assert( token == ":" );
         InStream >> Cam.poi;
         InStream >> token; assert( token == "," );
         InStream >> token; assert( token == "up" );
         InStream >> token; assert( token == ":" );
         InStream >> Cam.up; 
         InStream >> token; assert( token == "," );
         InStream >> token; assert( token == "size" );
         InStream >> token; assert( token == ":" );
         InStream >> Cam.size; 
         InStream >> token; assert( token == "}" );
         */
         return( InStream );
      }    
   }
}

