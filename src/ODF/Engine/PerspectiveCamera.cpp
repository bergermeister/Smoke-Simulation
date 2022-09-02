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
   }
}

