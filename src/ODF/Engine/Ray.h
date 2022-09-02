/**
 * @file Ray.h
 * @author Eisenberger, Edward (Edward.Eisenberger@live.com)
 * @brief 
 * @version 0.1
 * @date 2022-08-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ODF_Primitive_Ray_h
#define ODF_Primitive_Ray_h

// StdLib Includes
#include <iostream>

// ODF
#include <ODF/Math/Vector.h>

namespace ODF
{
   namespace Engine
   {
      class Ray 
      {
         protected:  // Protected Attributes
            Math::Vector< 3 > origin;
            Math::Vector< 3 > direction;

         public:     // Public Methods
            Ray( const Math::Vector< 3 >& Origin, const Math::Vector< 3 >& Direction )
               : origin( Origin ), direction( Direction )
            {
            }

            const Math::Vector< 3 >& Origin( ) const
            { 
               return( this->origin );
            }

            const Math::Vector< 3 >& Direction( void ) const
            { 
               return( this->direction ); 
            }

            Math::Vector< 3 > PointAtTime( double Time ) const
            {
               return( this->origin + ( this->direction * Time ) ); 
            }

            friend std::ostream& operator<<( std::ostream& OutStream, const Ray& R )
            {
               OutStream << "Ray <" << R.Origin( ) <<", "<< R.Direction( ) << ">";
               return( OutStream );
            }

      private:
         Ray( void ) = delete;
      };
   }
}

#endif
