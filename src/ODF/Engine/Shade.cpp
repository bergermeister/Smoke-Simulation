// ODF Includes
#include <ODF/Engine/Shade.h>

namespace ODF
{
   namespace Engine
   {
      /**
       * @brief Phong Local Illumination
       * 
       * This method should be called to compute the light contributed by a particular light source to the 
       * intersection point.
       * 
       * @note This method does not calculate any global effects (e.g., shadows). 
       * 
       * @param ray 
       * @param hit 
       * @param dirToLight 
       * @param lightColor 
       * @param args 
       * @return Math::Vector< 3 > 
       */
      Math::Vector< 3 > Shade( const Engine::Ray& ray, const Engine::Hit& hit, const Primitive::Material& mat,
                               const Math::Vector< 3 >& DirToLight, 
                               const Math::Vector< 3 >& LightColor )
      {
         Math::Vector< 3 > point = ray.PointAtTime( hit.Time( ) );
         Math::Vector< 3 > n = hit.Normal( );
         Math::Vector< 3 > e = ray.Direction( ) * -1.0f;
         Math::Vector< 3 > l = DirToLight;
         Math::Vector< 3 > result = Math::Vector< 3 >( { 0, 0, 0 } );

         /// -# Add emitted component
         result += mat.EmittedColor( );

         /// -# Add diffuse component
         double dot_nl = n.Dot( l );
         if( dot_nl < 0 ) 
         {
            dot_nl = 0;
         }
         result += LightColor * mat.DiffuseColor( hit.TextureS( ), hit.TextureT( ) ) * dot_nl;

         /// -# Add specular component (Phong)
         ///   - Make up reasonable values for other Phong parameters
         Math::Vector< 3 > specularColor = mat.ReflectiveColor( );
         double exponent = 100;

         /// -# Compute ideal reflection angle
         Math::Vector< 3 > r = ( l * -1.0f ) + n * ( 2 * dot_nl );
         r.Normalize( );
         double dot_er = e.Dot( r );
         if( dot_er < 0 ) 
         {
            dot_er = 0;
         }
         result += LightColor * specularColor * pow( dot_er, exponent ) * dot_nl;

         return( result );
      }
   }
}

