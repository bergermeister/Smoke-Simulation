// ODF Includes
#include <ODF/Engine/Camera.h>
#include <ODF/Math/Matrix4x4.h>

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
         // Don't let the model flip upside-down 
         // (There is a singularity at the poles when 'up' and 'direction' are aligned)
         double tiltAngle = acos( up.Dot( this->Direction( ) ) );
         if( ( tiltAngle - Ry ) > 3.13 )
         {
            Ry = tiltAngle - 3.13;
         }
         else if( ( tiltAngle - Ry ) < 0.01 )
         {
            Ry = tiltAngle - 0.01;
         }
         Math::Matrix4x4 rotMat;
         *dynamic_cast< Math::SquareMatrix< Math::Matrix4x4::Size >* >( &rotMat ) = Math::Matrix4x4::Identity( );
         rotMat *= Math::Matrix4x4::MakeTranslation( this->poi );
         rotMat *= Matrix::MakeAxisRotation(up, rx);
         rotMat *= Matrix::MakeAxisRotation(getHorizontal(), ry);
         rotMat *= Matrix::MakeTranslation(-point_of_interest);
         rotMat.Transform(camera_position);
      }

      // ====================================================================
      // ====================================================================
      // GENERATE RAY

      Ray OrthographicCamera::generateRay(double x, double y) {
      Math::Vector< 3 > screenCenter = camera_position;
      Math::Vector< 3 > xAxis = getHorizontal() * size; 
      Math::Vector< 3 > yAxis = getScreenUp() * size; 
      Math::Vector< 3 > lowerLeft = screenCenter - 0.5*xAxis - 0.5*yAxis;
      Math::Vector< 3 > screenPoint = lowerLeft + x*xAxis + y*yAxis;
      return Ray(screenPoint,getDirection());
      }

      Ray PerspectiveCamera::generateRay(double x, double y) {
      Math::Vector< 3 > screenCenter = camera_position + getDirection();
      double screenHeight = 2 * tan(angle/2.0);
      Math::Vector< 3 > xAxis = getHorizontal() * screenHeight;
      Math::Vector< 3 > yAxis = getScreenUp() * screenHeight;
      Math::Vector< 3 > lowerLeft = screenCenter - 0.5*xAxis - 0.5*yAxis;
      Math::Vector< 3 > screenPoint = lowerLeft + x*xAxis + y*yAxis;
      Math::Vector< 3 > dir = screenPoint - camera_position;
      dir.Normalize();
      return Ray(camera_position,dir); 
      } 
      // ====================================================================
      // ====================================================================

      std::ostream& operator<<(std::ostream &ostr, const Camera &c) {
      const Camera* cp = &c;
      if (dynamic_cast<const OrthographicCamera*>(cp)) {
         const OrthographicCamera* ocp = (const OrthographicCamera*)cp;
         ostr << *ocp;
      } else if (dynamic_cast<const PerspectiveCamera*>(cp)) {
         const PerspectiveCamera* pcp = (const PerspectiveCamera*)cp;
         ostr << *pcp;
      }
      return ostr;
      }

      std::ostream& operator<<(std::ostream &ostr, const OrthographicCamera &c) {
      ostr << "OrthographicCamera {" << std::endl;
      ostr << "    camera_position   " << c.camera_position;
      ostr << "    point_of_interest " << c.point_of_interest;
      ostr << "    up                " << c.up; 
      ostr << "    size              " << c.size << std::endl;
      ostr << "}" << std::endl;
      return ostr;
      }    

      std::ostream& operator<<(std::ostream &ostr, const PerspectiveCamera &c) {
      ostr << "PerspectiveCamera {" << std::endl;
      ostr << "  camera_position    " << c.camera_position;
      ostr << "  point_of_interest  " << c.point_of_interest;
      ostr << "  up                 " << c.up;
      ostr << "  angle              " << c.angle << std::endl;
      ostr << "}" << std::endl;
      return ostr;
      }


      std::istream& operator>>(std::istream &istr, OrthographicCamera &c) {
      std::string token;
      istr >> token; assert (token == "{");
      istr >> token; assert (token == "camera_position");
      istr >> c.camera_position;
      istr >> token; assert (token == "point_of_interest");
      istr >> c.point_of_interest;
      istr >> token; assert (token == "up");
      istr >> c.up; 
      istr >> token; assert (token == "size");
      istr >> c.size; 
      istr >> token; assert (token == "}");
      return istr;
      }    

      std::istream& operator>>(std::istream &istr, PerspectiveCamera &c) {
      std::string token;
      istr >> token; assert (token == "{");
      istr >> token; assert (token == "camera_position");
      istr >> c.camera_position;
      istr >> token; assert (token == "point_of_interest");
      istr >> c.point_of_interest;
      istr >> token; assert (token == "up");
      istr >> c.up; 
      istr >> token; assert (token == "angle");
      istr >> c.angle; 
      istr >> token; assert (token == "}");
      return istr;
      }
   }
}


