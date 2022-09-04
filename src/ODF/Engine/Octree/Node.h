/**
 * @file Node.h
 * @author Eisenberger, Edward (Edward.Eisenberger@live.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ODF_Engine_Octree_Node_h
#define ODF_Engine_Octree_Node_h

// StdLib Includes
#include <cstdint>

namespace ODF
{
   namespace Engine
   {
      namespace Octree
      {
         class Node
         {
            public:     // Public Attributes
                enum Type : uint8_t { Branch, Aggregate, Leaf };

            protected:  // Protected Attribtues
               Type type : 2;

            public:     // Public Methods
               inline Type GetType( void ) const
               {
                  return( this->type );
               }

               virtual void Delete( void ) = 0;

            protected:  // Protected Methods
               Node( Type NodeType ) : type( NodeType )
               {

               }

               virtual ~Node( void ) 
               {

               };
            };
      }
   }
}
            

#endif
