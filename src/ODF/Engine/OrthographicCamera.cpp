// ODF Includes
#include <ODF/Engine/OrthographicCamera.h>

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
   }
}

