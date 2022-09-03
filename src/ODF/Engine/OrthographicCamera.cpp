// ODF Includes
#include <ODF/Engine/OrthographicCamera.h>

// StdLib Includes
#include <sstream>

namespace ODF
{
   namespace Engine
   {
         const std::string OrthographicCamera::objectNameStr = "\"OrthographicCamera\":{";
         const std::string OrthographicCamera::positionStr = "\"position\":";
         const std::string OrthographicCamera::pointOfInterestStr = "\"pointOfInterest\":";
         const std::string OrthographicCamera::upStr = "\"up\":";
         const std::string OrthographicCamera::sizeStr = "\"size\":";

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
         OutStream << OrthographicCamera::objectNameStr;
         OutStream << OrthographicCamera::positionStr << Cam.pos << ",";
         OutStream << OrthographicCamera::pointOfInterestStr << Cam.poi << ",";
         OutStream << OrthographicCamera::upStr << Cam.up << ",";
         OutStream << OrthographicCamera::sizeStr << Cam.size << "}" << std::endl;
         return( OutStream );
      }

      /**
       * @brief 
       * 
       * @param InStream 
       * @param Cam 
       * @return std::istream& 
       */
      std::istream& operator>>( std::istream& InStream, OrthographicCamera& Cam )
      {
         size_t index = 0;
         size_t position;
         uint32_t attributeIndex;
         std::string str;
         std::string token;
         std::stringstream stream;

         /// @par
         /// -# Verify the object's name is OrthographicCamera
         InStream >> str;
         position = str.find( '{', index );
         token = str.substr( index, position - index + 1 );
         assert( token.compare( OrthographicCamera::objectNameStr ) == 0 );

         /// -# For each attribute
         ///   -# Obtain the attribute's name string
         ///   -# Parse the attribute corresponding to the name
         for( attributeIndex = 0; attributeIndex < OrthographicCamera::attributeCount; attributeIndex++ )
         {
            index = position + 1;
            position = str.find( ':', index );
            token = str.substr( index, position - index + 1 );
            index = position + 1;
            if( attributeIndex < ( OrthographicCamera::attributeCount - 1 ) )
            {
               position = str.find( ']', index );
            }
            else
            {
               position = str.find( '}', index );
            }
            stream.clear( );
            stream.str( str.substr( index, position - index + 2 ) );

            if( token.compare( OrthographicCamera::positionStr ) == 0 )
            {
               stream >> Cam.pos;
            }
            else if( token.compare( OrthographicCamera::pointOfInterestStr ) == 0 )
            {
               stream >> Cam.poi;
            }
            else if( token.compare( OrthographicCamera::upStr ) == 0 )
            {
               stream >> Cam.up;
            }
            else if( token.compare( OrthographicCamera::sizeStr ) == 0 )
            {
               stream >> Cam.size;
            }
            position++; // Move past ] to ,
         }
         
         return( InStream );
      }    
   }
}

