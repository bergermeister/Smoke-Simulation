/**
 * @file Aggregate.h
 * @author Eisenberger, Edward (Edward.Eisenberger@live.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ODF_Engine_Octree_Aggregate_h
#define ODF_Engine_Octree_Aggregate_h

// ODF Includes
#include <ODF/Engine/Octree/Node.h>

// StdLib Includes
#include <cstdint>

namespace ODF
{
   namespace Engine
   {
      namespace Octree
      {
         template< class T, uint32_t Size > class Aggregate : public Node
         {
            public:     // Public Attributes
               static constexpr uint32_t MaxIndex = Size * Size * Size;

            private:    // Private attributes
               T value[ Size ][ Size ][ Size ];

            public:
               Aggregate( const T& Val ) : Node( Node::Type::Aggregate )
               {
                  for( uint32_t i = 0; i < Size; i++ ) 
                  {
                     for( uint32_t j = 0; j < Size; j++ ) 
                     {
                        for( uint32_t k = 0; k < Size; k++ ) 
                        {
                           value_[ i ][ j ][ k ] = Val;
                        }
                     }
                  }
               }

               inline const T& Value( uint32_t X, uint32_t Y, uint32_t Z ) const
               {
                  assert( X < Size );
                  assert( Y < Size );
                  assert( Z < Size );
                  return( this->value[ Z ][ Y ][ X ] );
               }

               inline T& Value( uint32_t X, uint32_t Y, uint32_t Z )
               {
                  assert( X < Size );
                  assert( Y < Size );
                  assert( Z < Size );
                  return( this->value[ Z ][ Y ][ X ] );
               }

               inline void Value( uint32_t X, uint32_t Y, uint32_t Z, const T& Val )
               {
                  assert( X < Size );
                  assert( Y < Size );
                  assert( Z < Size );
                  this->value[ Z ][ Y ][ X ] = Val;
               }
               
               inline const T& Value( uint32_t Index ) const
               {
                  assert( Index < MaxIndex );
                  return( ( &this->value[ 0 ][ 0 ][ 0 ] )[ Index ] );
               }

               inline T& Value( uint32_t Index )
               {
                  assert( Index < MaxIndex );
                  return( ( &this->value[ 0 ][ 0 ][ 0 ] )[ Index ] );
               }

               inline void Value( uint32_t Index, const T& Val )
               {
                  assert( Index < MaxIndex );
                  ( &this->value[ 0 ][ 0 ][ 0 ] )[ Index ] = Val;
               }

               //friend void Octree<T,AS>::deleteNode( Node** node );

            protected:  // Protected Methods
               virtual ~Aggregate( void )
               {
               
               };
         };
      }
   }
}

#endif
