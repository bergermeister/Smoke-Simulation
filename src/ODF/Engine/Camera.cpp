// ODF Includes
#include <ODF/Engine/Camera.h>
#include <ODF/Math/Matrix3x3.h>

namespace ODF
{
   namespace Engine
   {
      /**
       * @brief Construct a new Camera:: Camera object
       * 
       * @param[in] Position  Origin position coordinates 
       * @param[in] POI 
       * @param[in] Up 
       */
      Camera::Camera( const Math::Vector< 3 >& Position, const Math::Vector< 3 >& POI, const Math::Vector< 3 >& Up )
         : pos( Position ), poi( POI ), up( Up )
      {
         up.Normalize();
      }

      /**
       * @brief GL Place Camera
       * 
       * Place a camera within an OpenGL scene
       * 
       * @param None 
       * @return
       */
      void Camera::GLPlace( void )
      {
         gluLookAt( this->pos[ 0 ], this->pos[ 1 ], this->pos[ 2 ],
                    this->poi[ 0 ], this->poi[ 1 ], this->poi[ 2 ],
                    this->up[ 0 ],  this->up[ 1 ],  this->up[ 2 ] );
      }

      /**
       * @brief Move camera along the direction vector
       *
       * 
       * @param None
       * @return
       */
      void Camera::Dolly( double Distance )
      {
         Math::Vector< 3 > diff = this->pos - this->poi;
         double d = diff.Length();
         diff.Normalize( );
         d *= pow( 1.003, Distance );
         this->pos = this->poi + ( diff * d );
      }
      
      /**
       * @brief Translate camera perpendicular to the direction vector
       * 
       * @param[in] Dx
       * @param[in] Dy
       * @return 
       */
      void Camera::Truck( double Dx, double Dy )
      {
         Math::Vector< 3 > diff = this->pos - this->poi;
         double d = diff.Length();
         Math::Vector< 3 > translate = ( ( this->Horizontal( ) * Dx ) + ( this->ScreenUp( ) * Dy ) ) * ( d * 0.0007 );
         this->pos += translate;
         this->poi += translate;
      }

      /**
       * @brief Rotate around the up and horizontal vectors
       * 
       * @param[in] Rx
       * @param[in] Ry
       * @return 
       */
      void Camera::Rotate( double Rx, double Ry )
      {
         using Math::Matrix3x3;
         using Math::SquareMatrix;
         // Don't let the model flip upside-down 
         // (There is a singularity at the poles when 'up' and 'direction' are aligned)
         double tiltAngle = acos( this->up.Dot( this->Direction( ) ) );
         if( ( tiltAngle - Ry ) > 3.13 )
         {
            Ry = tiltAngle - 3.13;
         }
         else if( ( tiltAngle - Ry ) < 0.01 )
         {
            Ry = tiltAngle - 0.01;
         }
         Matrix3x3 rotationMatrix( Matrix3x3::Identity( ) );
         rotationMatrix *= Matrix3x3::MakeTranslation( this->poi );
         rotationMatrix *= Matrix3x3::MakeAxisRotation( this->up, Rx );
         rotationMatrix *= Matrix3x3::MakeAxisRotation( this->Horizontal( ), Ry );
         rotationMatrix *= Matrix3x3::MakeTranslation( -this->poi );
         this->pos = rotationMatrix.Transform( );
      }
   }
}


