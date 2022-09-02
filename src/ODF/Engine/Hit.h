/**
 * @file Hit.h
 * @author Eisenberger, Edward (Edward.Eisenberger@live.com)
 * @brief 
 * @version 0.1
 * @date 2022-08-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ODF_Primitive_Hit_h
#define ODF_Primitive_Hit_h

// ODF Includes
#include <ODF/Math/Vector.h>
#include <ODF/Primitive/Material.h>

namespace ODF
{
   namespace Engine
   {
      class Hit 
      {
         private:    // Private Attributes
            double time;
            Math::Vector< 3 > normal;
            double textureS;
            double textureT;
            Primitive::Material* material;

         public:     // Public Methods
            Hit( void );
            Hit( const Hit& H );
            ~Hit( void ) = default;
            Hit& operator=( const Hit& H );
            void Set( double Time, Math::Vector< 3 > Normal, Primitive::Material* Mat );
            void TextureCoordinates( double TextureS, double TextureT );

            /**
             * @brief 
             * 
             * @return double 
             */
            inline double Time( void ) const 
            { 
               return( this->time ); 
            }
            
            /**
             * @brief Get the Material object
             * 
             * @return Material* 
             */
            inline Primitive::Material* getMaterial( ) const 
            { 
               return( this->material );
            }
            
            /**
             * @brief 
             * 
             * @return Math::Vector< 3 > 
             */
            inline Math::Vector< 3 > Normal( void ) const 
            { 
               return( this->normal );
            }

            /**
             * @brief 
             * 
             * @return double 
             */
            inline double TextureS( void ) const 
            { 
               return( this->textureS ); 
            }

            /**
             * @brief 
             * 
             * @return double 
             */
            inline double TextureT( void ) const 
            { 
               return( this->textureT );
            }

            /**
             * @brief 
             * 
             * @param[out] OutStream 
             * @param[in]  H 
             * @return std::ostream& 
             */
            inline friend std::ostream& operator<<( std::ostream& OutStream, const Hit& H )
            {
               OutStream << "Hit <" << H.Time( ) <<", " << H.Normal( ) << ">";
               return( OutStream );
            }
      };
   }
}
#endif
