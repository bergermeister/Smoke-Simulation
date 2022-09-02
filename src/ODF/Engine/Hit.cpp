// ODF Includes
#include <ODF/Engine/Hit.h>

// StdLib Includes
#include <float.h>

namespace ODF
{
   namespace Engine
   {
      /**
       * @brief Construct a new Hit object
       * 
       */
      Hit::Hit( void )
         : normal( { 0, 0, 0 } )
      { 
         this->time = FLT_MAX;
         this->textureS = 0;
         this->textureT = 0;
         this->material = nullptr;
      }

      /**
       * @brief Construct a new Hit object copied from an existing Hit object
       * 
       * @param H 
       */
      Hit::Hit( const Hit& H )
         : normal( { 0, 0, 0 } )
      {
         *this = H;
      }

      /**
       * @brief 
       * 
       * @param H 
       * @return Hit& 
       */
      Hit& Hit::operator=( const Hit& H )
      {
         if( this != &H )
         { 
            this->time = H.time; 
            this->material = H.material; 
            this->normal = H.normal; 
            this->textureS = H.textureS;
            this->textureT = H.textureT;
         }
         return( *this );
      }

      void Hit::Set( double Time, Math::Vector< 3 > Normal, Primitive::Material* Mat )
      {
         this->time = Time;
         this->normal = Normal; 
         this->textureS = 0; 
         this->textureT = 0;
         this->material = Mat;
      }

      /**
       * @brief 
       * 
       * @param[in] TextureS 
       * @param[in] TextureT 
       */
      void Hit::TextureCoordinates( double TextureS, double TextureT )
      {
         this->textureS = TextureS; 
         this->textureT = TextureT; 
      }
   }
}

