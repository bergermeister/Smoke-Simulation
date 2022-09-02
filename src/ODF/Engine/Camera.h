/**
 * @file Camera.h
 * @author Eisenberger, Edward (Edward.Eisenberger@live.com)
 * @brief 
 * @version 0.1
 * @date 2022-08-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ODF_Primitive_Camera_h
#define ODF_Primitive_Camera_h

// ODF Includes
#include <ODF/Environment.h>
#include <ODF/Engine/Ray.h>
#include <ODF/Math/Vector.h>

namespace ODF
{
   namespace Engine
   {
      class Camera
      {
         protected:  // Protected Attributes
            Math::Vector< 3 > pos;   ///< Camera Position
            Math::Vector< 3 > poi;   ///< Point of Interest
            Math::Vector< 3 > up;
            int width;
            int height;

         public:     // Public Methods
            Camera( const Math::Vector< 3 >& Position, const Math::Vector< 3 >& POI, const Math::Vector< 3 >& Up );
            virtual ~Camera( void ) = default;
            
            virtual Ray GenerateRay( double X, double Y ) = 0;

            /// @name GL NAVIGATION
            /// @{
            virtual void GLInit( int Width, int Height ) = 0;
            virtual void Zoom( double Distance ) = 0;
            void GLPlace( void );
            void Dolly( double Distance );            
            void Truck( double Dx, double Dy );
            void Rotate( double Rx, double Ry );
            /// @}

            friend std::ostream& operator<<( std::ostream& OutStream, const Camera& Camera );

         protected:  // Protected Methods
            Camera( void ) = delete;

            // HELPER FUNCTIONS
            Math::Vector< 3 > Horizontal( void ) const
            {
               Math::Vector< 3 > result = Math::Vector< 3 >::Cross( this->Direction( ), this->up );
               result.Normalize( );
               return( result );
            }

            inline Math::Vector< 3 > ScreenUp( void ) const 
            {
               return( Math::Vector< 3 >::Cross( this->Horizontal( ), this->Direction( ) ) );
            }

            Math::Vector< 3 > Direction( void ) const 
            {
               Math::Vector< 3 > result = this->poi - this->pos;
               result.Normalize( );
               return( result ); 
            }
      };
   }
}

#endif

