/**
 * @file Branch.h
 * @author Eisenberger, Edward (Edward.Eisenberger@live.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ODF_Engine_Octree_Branch_h
#define ODF_Engine_Octree_Branch_h

// ODF Includes
#include <ODF/Engine/Octree/Node.h>
#include <ODF/Engine/Octree/Aggregate.h>
#include <ODF/Engine/Octree/Leaf.h>

// StdLib Includes
#include <cstdint>
#include <cstring>
#include <cassert>

namespace ODF
{
   namespace Engine
   {
      namespace Octree
      {
         template< class T, uint32_t AggregateSize > class Branch : public Node
         {
            public:     // Public Attributes
               static constexpr uint32_t TotalNodes = 8;
               static constexpr uint32_t NodesPerDimension = 2;

            protected:  // Protected Attributes
               Node* children[ NodesPerDimension ][ NodesPerDimension ][ NodesPerDimension ];

            public:     // Public Methods
               Branch( void ) : Node( Node::Type::Branch )
               {
                  std::memset( reinterpret_cast< void* >( this->children ), 0, sizeof( this->children ) );
               }

               Branch( const Branch& B ) : Node( Node::Type::Branch )
               {
                  for ( uint32_t i = 0; i < TotalNodes; ++i ) 
                  {
                     if( B.Child( i ) )
                     {
                        switch( B.Child( i )->GetType( ) )
                        {
                           case Node::Type::Branch:
                           {
                              this->Child( i ) = new Branch( *reinterpret_cast< const Branch* >( B.Child( i ) ) );
                              break;
                           }
                           case Node::Type::Leaf:
                           {
                              this->Child( i ) = 
                                 new Leaf< T >( *reinterpret_cast< const Leaf< T >* >( B.Child( i ) ) );
                              break;
                           }
                           case Node::Type::Aggregate:
                           {
                              this->Child( i ) = 
                                 new Aggregate< AggregateSize >( 
                                    *reinterpret_cast< const Aggregate< AggregateSize >* >( B.Child( i ) ) );
                              break;
                           }
                        }
                     }
                     else
                     {
                        this->child( i ) = 0;
                     }
                  }
               }

               virtual ~Branch( void )
               {
                  for( uint32_t i = 0; i < NodesPerDimension; i++ )
                  {
                     for( uint32_t j = 0; j < 2; j++ )
                     {
                        for( uint32_t k = 0; k < 2; k++ )
                        {
                           assert( this->children[ i ][ j ][ k ] != this );
                           deleteNode( &this->children[ i ][ j ][ k ] );
                        }
                     }
                  }
               }


               inline const Node* Child( uint32_t X, uint32_t Y, uint32_t Z ) const
               {
                  assert( X < NodesPerDimension );
                  assert( Y < NodesPerDimension );
                  assert( Z < NodesPerDimension );
                  return( this->children[ Z ][ Y ][ X ] );
               }

               inline Node*& Child( uint32_t X, uint32_t Y, uint32_t Z )
               {
                  assert( X < NodesPerDimension );
                  assert( Y < NodesPerDimension );
                  assert( Z < NodesPerDimension );
                  return( this->children[ Z ][ Y ][ X ] );
               }
               
               inline const Node* Child( uint32_t Index ) const
               {
                  assert( Index < TotalNodes );
                  return( ( &this->children[ 0 ][ 0 ][ 0 ] )[ Index ] );
               }

               inline Node*& Child( uint32_t Index )
               {
                  assert( Index < TotalNodes );
                  return( ( &this->children[ 0 ][ 0 ][ 0 ] )[ Index ] );
               }

               //friend void Octree<T,AS>::deleteNode( Node** node );

            private:
               Branch& operator=( Branch B )
               {

               }
         };
      }
   }
}

#endif

