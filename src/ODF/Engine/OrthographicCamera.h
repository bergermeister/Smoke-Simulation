/**
 * @file OrthographicCamera.h
 * @author Eisenberger, Edward (Edward.Eisenberger@live.com)
 * @brief 
 * @version 0.1
 * @date 2022-08-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ODF_Engine_OrthographicCamera_h
#define ODF_Engine_OrthographicCamera_h

// ODF Includes
#include <ODF/Engine/Camera.h>
#include <ODF/Engine/Ray.h>
#include <ODF/Math/Vector.h>

namespace ODF
{
   namespace Engine
   {
      class OrthographicCamera : public Camera
      {
         private:
            double size;

         public:
            OrthographicCamera( const Math::Vector< 3 >& Position = Math::Vector< 3 >( { 0, 0, 1 } ), 
                                const Math::Vector< 3 >& POI = Math::Vector< 3 >( { 0, 0, 0 } ), 
                                const Math::Vector< 3 >& Up = Math::Vector< 3 >( { 0, 1, 0 } ),
                                double Size = 100.0 );
            
            Ray GenerateRay( double X, double Y );

            /// @name GL NAVIGATION
            /// @{
            virtual void GLInit( int Width, int Height ) override;
            virtual void Zoom( double Factor ) override;
            friend std::ostream& operator<<( std::ostream& OutStream, const OrthographicCamera& Cam );
            friend std::istream& operator>>( std::istream& InStream, OrthographicCamera& Cam );
            /// @}

         protected:  // Protected Methods
            virtual std::ostream& stream( std::ostream& OutStream ) const override
            {
               OutStream << *this;
               return( OutStream );
            }

            virtual std::istream& stream( std::istream& InStream ) override
            {
               InStream >> *this;
               return( InStream );
            }
      };
   }
}

#endif
