// ODF Includes
#include <ODF/Engine/PerspectiveCamera.h>

namespace ODF
{
   namespace Engine
   {
      PerspectiveCamera::PerspectiveCamera
      (const Math::Vector< 3 > &c, const Math::Vector< 3 > &poi, const Math::Vector< 3 > &u, double a) : Camera(c,poi,u) {
      angle = a;
      }

      void PerspectiveCamera::glInit(int w, int h) {
      width = w;
      height = h;
      glMatrixMode(GL_PROJECTION);
      glLoadIdentity();
      double aspect = double(width)/double(height);
      double asp_angle = angle * 180/M_PI;
      if (aspect > 1) asp_angle /= aspect;
      double dist_to_poi = (point_of_interest-camera_position).Length();
      gluPerspective(asp_angle, aspect, dist_to_poi*0.1, dist_to_poi*100.0);
      }

      /**
       * @brief Change the field of view/angle
       * 
       * @param dist 
       */
      void PerspectiveCamera::zoomCamera(double dist) {
      angle *= pow(1.003,dist);
      glInit(width,height);
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

      std::ostream& operator<<(std::ostream &ostr, const PerspectiveCamera &c) {
      ostr << "PerspectiveCamera {" << std::endl;
      ostr << "  camera_position    " << c.camera_position;
      ostr << "  point_of_interest  " << c.point_of_interest;
      ostr << "  up                 " << c.up;
      ostr << "  angle              " << c.angle << std::endl;
      ostr << "}" << std::endl;
      return ostr;
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

