/**
 * @file PerspectiveCamera.h
 * @author Eisenberger, Edward (Edward.Eisenberger@live.com)
 * @brief 
 * @version 0.1
 * @date 2022-08-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ODF_Primitive_PerspectieCamera_h
#define ODF_Primitive_PerspectieCamera_h

namespace ODF
{
   namespace Primitive
   {
      class PerspectiveCamera : public Camera {

      public:
      // CONSTRUCTOR & DESTRUCTOR
      PerspectiveCamera(const Vec3f &c = Vec3f(0,0,1), 
               const Vec3f &poi = Vec3f(0,0,0), 
               const Vec3f &u = Vec3f(0,1,0),
               double a = 45);

      
      // RENDERING
      Ray generateRay(double x, double y);


      // GL NAVIGATION
      void glInit(int w, int h);
      void zoomCamera(double dist);
      friend std::ostream& operator<<(std::ostream &ostr, const PerspectiveCamera &c);
      friend std::istream& operator>>(std::istream &istr, PerspectiveCamera &c);
      
      private:

      // REPRESENTATION
      double angle;
      };
   }
}

#endif

