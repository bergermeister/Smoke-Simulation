/**
 * @file SquareMatrix.h
 * @author Eisenberger, Edward (Edward.Eisenberger@live.com)
 * @brief 
 * @version 0.1
 * @date 2022-08-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ODF_Math_SquareMatrix_h
#define ODF_Math_SquareMatrix_h

// ODF Includes
#include <ODF/Math/Matrix.h>

namespace ODF
{
   namespace Math
   {
      template< size_t Size > class SquareMatrix : public Matrix< Size, Size >
      {
         public:     // Public Methods
            /**
             * @brief 
             * 
             * @param None
             * @return Matrix 
             */
            Matrix< Size, Size > Transpose( void ) const 
            {
               Matrix< Size, Size > matrix;
               for( size_t row = 0; row < Size; row++ )
               {
                  for( size_t col = 0; col < Size; col++ )
                  {
                     matrix( row, col ) = this->operator( )( col, row );
                  }
               }
               return( matrix );
            }

            /**
             * @brief 
             * 
             * @param[in] Vec
             * @return 
             */
            Vector< Size > Transform( void ) const
            {
               Vector< Size > result( { } );
               for( size_t row = 0; row < Size; row++ )
               {
                  for( size_t col = 0; col < Size; col++ )
                  {
                     result( row ) += Matrix< Size, Size >::operator( )( row, col ) * result( col );
                  }
               }
               return( result );
            }

            /**
             * @brief 
             * 
             * @param None
             * @return Matrix 
             */
            static SquareMatrix< Size > Identity( void )
            {
               SquareMatrix< Size > matrix;
               for( size_t row = 0; row < Size; row++ )
               {
                  for( size_t col = 0; col < Size; col++ )
                  {
                     matrix( row, col ) = static_cast< double >( row == col );
                  }
               }
               return( matrix );
            }
      };
   }
}

#endif

