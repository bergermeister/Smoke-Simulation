// ODF Includes
#include <ODF/Primitive/Material.h>
#include <ODF/Engine/Hit.h>

namespace ODF
{
   namespace Primitive
   {
      Material::Material( const Math::Vector< 3 >& DiffuseColor, const Math::Vector< 3 >& ReflectiveColor, 
                          const Math::Vector< 3 >& EmittedColor, double Roughness )
         : diffuseColor( DiffuseColor ), reflectiveColor( ReflectiveColor ), emittedColor( EmittedColor )
      {
         this->roughness = Roughness;
         // need to initialize texture_id after glut has started
      }

      float AlternateFog( float cosVal, float distToFrag )
      {
         float fex = exp( -distToFrag*(0.15+0.15) );
         float ins = (1-fex);
         float brt = 3.0/(16.0*3.14159)*0.15*(1.0+cosVal*cosVal);
         float bmt = 1.0/(4.0*3.14159)*0.15*(1.0-0.95)*(1.0-0.95)/(pow((1.0+0.95*0.95-2*0.95*cosVal),1.5) );
         float lint = (brt+bmt)/(0.15+0.15)*ins;
         return lint;
      }
   }
}
