/**
 * @file Leaf.h
 * @author Eisenberger, Edward (Edward.Eisenberger@live.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ODF_Engine_Octree_Leaf_h
#define ODF_Engine_Octree_Leaf_h

namespace ODF
{
   namespace Engine
   {
      namespace Octree
      {
         template< class T > class Leaf : public Node
         {
            protected:
               T value;

            public:
               Leaf( const T& Val ) 
                  : Node( Node::Type::Leaf ), value( Val )
               {
                  
               }

               const T& Value( void ) const
               {
                  return( this->value );
               }

               T& Value( void )
               {
                  return( this->value );
               }

               void Value( const T& Val )
               {
                  this->value = Val;
               }

               //friend void Octree<T,AS>::deleteNode( Node** node );

            protected:  // Protected Methods
               virtual ~Leaf( void ) 
               {

               };            
         };
      }
   }
}

#endif