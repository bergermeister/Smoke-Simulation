/**
 * @file Material.h
 * @author Eisenberger, Edward (Edward.Eisenberger@live.com)
 * @brief 
 * @version 0.1
 * @date 2022-08-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ODF_Primitive_Material_h
#define ODF_Primitive_Material_h

// StdLib Includes
#include <cassert>
#include <cstring>

// ODF Includes
#include <ODF/Math/Vector.h>
#include <ODF/Engine/Ray.h>

namespace ODF
{
   namespace Primitive
   {
      class Hit;

      // A simple Phong-like material 
      class Material 
      {
         protected:  // Protected Attributes
            Math::Vector< 3 > diffuseColor;
            Math::Vector< 3 > reflectiveColor;
            Math::Vector< 3 > emittedColor;
            double roughness;

         public:     // Public Methods
            Material( const Math::Vector< 3 >& DiffuseColor, const Math::Vector< 3 >& ReflectiveColor, 
                      const Math::Vector< 3 >& EmittedColor, double Roughness );
            ~Material( void ) = default;

            /**
             * @brief 
             * 
             * @return const Math::Vector< 3 >& 
             */
            inline const Math::Vector< 3 >& DiffuseColor( ) const 
            { 
               return( this->diffuseColor ); 
            }

            /**
             * @brief Texture Lookup for Diffuse Color
             * 
             * @param[in] s 
             * @param[in] t 
             * @return const Math::Vector< 3 > 
             */
            inline const Math::Vector< 3 > DiffuseColor( double s, double t ) const
            {
               /// @todo
               return( this->diffuseColor );
            }

            /**
             * @brief 
             * 
             * @return const Math::Vector< 3 >& 
             */
            inline const Math::Vector< 3 >& ReflectiveColor( void ) const 
            { 
               return( this->reflectiveColor ); 
            }

            /**
             * @brief 
             * 
             * @return const Math::Vector< 3 >& 
             */
            inline const Math::Vector< 3 >& EmittedColor( void ) const 
            { 
               return( this->emittedColor ); 
            }  

            /**
             * @brief Get the Roughness object
             * 
             * @return double 
             */
            inline double Roughness( void ) const 
            { 
               return( this->roughness ); 
            }            
            
         private:    // Private Methods
            Material( void ) = delete;
            Material( const Material& ) = delete;
            const Material& operator=( const Material& ) = delete;
      };
   }
}

#endif
  
