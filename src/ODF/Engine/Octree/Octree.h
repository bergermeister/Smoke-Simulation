/**
 * @file Octree.h
 * @author Eisenberger, Edward (Edward.Eisenberger@live.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ODF_Engine_Octree_h
#define ODF_Engine_Octree_h

// ODF Includes
#include <ODF/Engine/Octree/Node.h>
#include <ODF/Engine/Octree/Branch.h>
#include <ODF/Engine/Octree/Aggregate.h>
#include <ODF/Engine/Octree/Leaf.h>

// StdLib Includes
#include <algorithm>
#include <cstdint>
#include <cassert>
#include <istream>
#include <ostream>

namespace ODF
{
   namespace Engine
   {
      namespace Octree
      {
         template< typename T, uint32_t AggregateSize = 1 > class Octree
         {
            protected:  // Protected Attributes
               Node* root;
               const T emptyValue;
               uint32_t size;

            public:     // Public Methods
               /**
                * @brief Construct a new Octree object
                * 
                * @param[in] Size         
                * Size of octree, in nodes. Should be a power of two. For example, an octree with \a size = 256 will 
                * represent a cube divided into 256x256x256 nodes. <b>Must be a power of two.
                * @param[in] EmptyValue   
                * This is the value that will be returned when regions of the 3-D volume where no node has been 
                * allocated. In other words, instead of following a null node pointer, this value is returned. Since 
                * the octree root is initially a null pointer, the whole volume is initialized to this value.
                */
               Octree( uint32_t Size, const T& EmptyValue = T( 0 ) )
                  : root( nullptr ), emptyValue( EmptyValue ), size( Size )
               {
                  // Make sure size is power of two.
                  assert( ( ( this->size - 1 ) & this->size ) == 0 );
                  assert( ( ( AggregateSize - 1 ) & AggregateSize ) == 0 );
               }

               Octree( const Octree& O )
                  : emptyValue( O.emptyValue ), size( O.size )
               {
                  if( O.root == nullptr )
                  {
                     this->root = nullptr;
                  } 
                  else 
                  {
                     switch( O.root->GetType( ) )
                     {
                        case Node::Type::Branch:
                        {
                           this->root = 
                              new Branch< T, AggregateSize >( 
                                 *reinterpret_cast< Branch< T, AggregateSize >* >( O.root ) );
                           break;
                        }
                        case Node::Type::Leaf:
                        {
                           this->root = new Leaf< T >( *reinterpret_cast< Leaf< T >* >( O.root ) );
                           break;
                        }
                        case Node::Type::Aggregate:
                        {
                           this->root = 
                              new Aggregate< AggregateSize >( 
                                 *reinterpret_cast< Aggregate< AggregateSize* >( O.root ) );
                           break;
                        }
                     }
                  }
               }

               virtual ~Octree( void )
               {
                  if( this->root != nullptr )
                  {
                     this->root->Delete( );
                  }
               }

               // Accessors
               inline uint32_t size( void ) const { return( this->size ) };
               const T& emptyValue( void ) const;

               static unsigned long branchBytes();
               static unsigned long aggregateBytes();
               static unsigned long leafBytes();
               unsigned long bytes() const;

               int nodes() const;
               int nodesAtSize( int size ) const;

               // Mutators
               void setEmptyValue( const T& emptyValue );

               void swap( Octree<T,AS>& o );
               Octree<T,AS>& operator= ( Octree<T,AS> o );

               // Indexing operators
               T& operator() ( int x, int y, int z );
               const T& operator() ( int x, int y, int z ) const;
               const T& at( int x, int y, int z ) const;

               void set( int x, int y, int z, const T& value );
               void erase( int x, int y, int z );

               Array2D<T> zSlice( int z ) const;

               // I/O functions
               void writeBinary( std::ostream& out ) const;
               void readBinary( std::istream& in );

            protected:

               // Octree node types
               class Node;
               class Branch;
               class Aggregate;
               class Leaf;
               

               Node*& root();
               const Node* root() const;

               static void deleteNode( Node** node );

            private:
               // Recursive helper functions
               void eraseRecursive( Node** node, int size, int x, int y, int z );
               static unsigned long bytesRecursive( const Node* node );
               static int nodesRecursive( const Node* node );
               static int nodesAtSizeRecursive( int targetSize, int size, Node* node );
               void zSliceRecursive( Array2D<T> slice, const Node* node, int size,
                        int x, int y, int z, int targetZ ) const;
               static void writeBinaryRecursive( std::ostream& out, const Node* node );
               static void readBinaryRecursive( std::istream& in, Node** node );

            protected:
               

               static const int aggregateSize_ = AS;

            
         };
      }
   }
}

#endif

