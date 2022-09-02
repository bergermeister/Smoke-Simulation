/**
 * @file Matrix.h
 * @author Eisenberger, Edward (Edward.Eisenberger@live.com)
 * @brief 
 * @version 0.1
 * @date 2022-08-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ODF_Math_Matrix_h
#define ODF_Math_Matrix_h

// StdLib Includes
#include <cstdint>
#include <cstring>
#include <cassert>
#include <iostream>
#include <iomanip>

// ODF
#include <ODF/Math/Vector.h>

namespace ODF
{
   namespace Math
   {
      template< size_t RowCount, size_t ColumnCount > class Matrix
      {
         public:     // Public Attributes
            static constexpr size_t Rows = RowCount;
            static constexpr size_t Columns = ColumnCount;

         protected:  // Protected Attributes
            double data[ Rows * Columns ];   ///< column-major order

         public:     // Public Methods
            /**
             * @brief Construct a new Matrix object
             * 
             * @param None
             * @return This metod returns nothing.
             */
            Matrix( void )
            {
               this->Clear( );
            }

            /**
             * @brief Construct a new Matrix object
             * 
             * @param[in] M
             * @return This method returns nothing. 
             */
            Matrix( Matrix& M )
            {
               *this = M;
            }

            /**
             * @brief Move constructor
             * 
             * @param M 
             */
            Matrix( Matrix&& M )
            {
               /// @todo std::move decays data to a pointer, must copy each element individually
               //this->data = std::move( M.data );
               *this = M;
            }

            /**
             * @brief Destroy the Matrix object
             * 
             * @param None
             * @return
             */
            ~Matrix( void ) = default;

            /**
             * @brief 
             * 
             * @param[in] M 
             * @return This method returns a reference to this object.
             */
            Matrix& operator=( const Matrix& M )
            {
               if( this != &M )
               {
                  std::memcpy( reinterpret_cast< void* >( this ), 
                               reinterpret_cast< const void* >( &M ),
                               sizeof( Matrix ) );
               }
               return( *this );
            }

            /**
             * @brief 
             * 
             * @param B 
             * @return Matrix 
             */
            Matrix operator*=( const Matrix& B )
            {
               for( size_t row = 0; row < Rows; row++ )
               {
                  for( size_t col = 0; col < Columns; col++ )
                  {
                     for( size_t i = 0; i < Rows; i++ )
                     {
                        this->operator()( row ,col ) = 
                           this->operator()( row, col ) + ( this->operator()( row, i ) * B( i, col ) );
                     }
                  }
               }
               return( *this );
            }

            /**
             * @brief 
             * 
             * @param[in] Row 
             * @param[in] Column 
             * @return double 
             */
            inline double operator( )( size_t Row, size_t Column ) const
            {
               assert( Row < Rows );
               assert( Column < Columns );
               return( this->data[ Row + ( Column * Rows ) ] );
            }

            /**
             * @brief 
             * 
             * @param[in] Row 
             * @param[in] Column 
             * @return double& 
             */
            inline double& operator( )( size_t Row, size_t Column )
            {
               assert( Row < Rows );
               assert( Column < Columns );
               return( this->data[ Row + ( Column * Rows ) ] );
            }          

            /**
             * @brief 
             * 
             * @param None
             * @return
             */
            inline void Clear( void )
            {
               std::memset( reinterpret_cast< void* >( this ), 0, sizeof( Matrix ) );     
            }

            /**
             * @brief 
             * 
             * @param[in] OutStream 
             * @param[in] M 
             * @return std::ostream& 
             */
            friend std::ostream& operator<<( std::ostream& OutStream, const Matrix& M )
            {
               static constexpr double limit = 0.00001;
               double value;
               for( size_t row = 0; row < Rows; row++ )
               {
                  for( size_t col = 0; col < Columns; col++ )
                  {
                     value = M( row, col );
                     if( fabs( value ) < limit )
                     {
                        value = 0;
                     } 
                     OutStream << std::setw( 12 ) << value << " ";
                  }
                  OutStream << std::endl;
               } 
               return( OutStream );
            }

            /**
             * @brief 
             * 
             * @param[in] InStream 
             * @param[in] M 
             * @return std::istream& 
             */
            friend std::istream& operator>>( std::istream& InStream, Matrix& M )
            {
               double value;
               for( size_t row = 0; row < Rows; row++ )
               {
                  for( size_t col = 0; col < Columns; col++ )
                  {
                     InStream >> value;
                     M( row, col ) = value;
                  }
               } 
               return( InStream );
            }
      };

      /**
       * @brief 
       * 
       * @param A 
       * @param B 
       * @return Matrix 
       */
      template< size_t M, size_t N, size_t P >
      static Matrix< M, P > operator*( const Matrix< M, N >& A, 
                                                        const Matrix< N, P >& B )
      {
         Matrix< M, P >C;
         for( size_t row = 0; row < M; row++ )
         {
            for( size_t col = 0; col < N; col++ )
            {
               for( size_t i = 0; i < P; i++ )
               {
                  C( row ,col ) = C( row, col ) + ( A( row, i ) * B( i, col ) );
               }
            }
         }
         return( C );
      }
   }
}

#endif

