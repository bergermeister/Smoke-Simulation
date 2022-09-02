/**
 * @file Vertex.h
 * @author Eisenberger, Edward (Edward.Eisenberger@live.com)
 * @brief 
 * @version 0.1
 * @date 2022-08-27
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ODF_Primitive_Vertex_h
#define ODF_Primitive_Vertex_h

// ODF Incldues
#include <ODF/Math/Vector.h>

namespace ODF
{
   namespace Primitive
   {
      /**
       * @brief 
       * 
       * 
       * @note 
       * The vertices don't know anything about adjacency. In some versions of this data structure they have a pointer
       * to one of their incoming edges. However, this data is complicated to maintain during mesh manipulation.
       */
      class Vertex 
      {
         private:    // Private Attributes
            Math::Vector< 3 > position;   ///< Coordinates of Vertex
            double s;                     ///< Texture coordinate S
            double t;                     ///< Texture coordinate T 
            int index;                    ///< Index from the original .obj file.

         public:
            /**
             * @brief Construct a new Vertex object
             * 
             * @param[in] i 
             * @param[in] Position 
             * @return
             */
            Vertex( int Index, const Math::Vector< 3 >& Position ) 
               : position( Position ) 
            { 
               this->index = Index; 
               this->s = 0; 
               this->t = 0; 
            }
            
            /**
             * @brief 
             * 
             * @param None
             * @return int 
             */
            int Index( void ) const 
            { 
               return( this->index ); 
            }

            /**
             * @brief 
             * 
             * @param None
             * @return double 
             */
            double X( void ) const 
            { 
               return( this->position[ 0 ] ); 
            }
            
            /**
             * @brief 
             * 
             * @param None
             * @return double 
             */
            double Y( void ) const 
            { 
               return( this->position[ 1 ] ); 
            }
            
            /**
             * @brief 
             * 
             * @param None
             * @return double 
             */
            double Z( void ) const 
            { 
               return( this->position[ 2 ] );
            }

            /**
             * @brief 
             * 
             * @param None
             * @return const Math::Vector< 3 >& 
             */
            const Math::Vector< 3 >& Coordinates( void ) const 
            { 
               return( this->position );
            }
            
            /**
             * @brief 
             * 
             * @param None
             * @return double 
             */
            double S( void ) const 
            { 
               return( this->s ); 
            }
            
            /**
             * @brief 
             * 
             * @param None
             * @return double 
             */
            double T( void ) const 
            { 
               return( this->t ); 
            }

            /**
             * @brief 
             * 
             * @param[in] S 
             * @param[in] T
             * @return 
             */
            void TextureCoordinates( double S, double T ) 
            { 
               this->s = S; 
               this->t = T; 
            }
      };
   }
}

#endif

