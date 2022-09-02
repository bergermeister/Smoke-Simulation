/**
 * @file Matrix3x3.h
 * @author Eisenberger, Edward (Edward.Eisenberger@live.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ODF_Math_Matrix3x3_h
#define ODF_Math_Matrix3x3_h

// ODF Includes
#include <ODF/Math/SquareMatrix.h>

namespace ODF
{
   namespace Math
   {
      class Matrix3x3 : public SquareMatrix< 3 >
      {
         public:     // Public Attriutes
            static constexpr size_t Size = 3;

         public:     // Public Methods
            Matrix3x3( void );
            Matrix3x3( const Matrix3x3& Mat3x3 );
            Matrix3x3( const SquareMatrix< Size > SquareMat );
            virtual ~Matrix3x3( void ) = default;
            Matrix3x3& operator=( const SquareMatrix< 3 >& Mat );
            Matrix3x3& operator*=( const Matrix3x3& Mat );

            Matrix3x3 AxisRotation( const Vector< 3 >& Vec, double Theta );
            Matrix3x3 Transpose( void ) const;            
            int Inverse( Matrix3x3& M, double Epsilon = 1e-08 ) const;
            
            /**
             * @brief 
             * 
             * @param[in] Epsilon 
             * @return int 
             */
            inline int Inverse( double Epsilon = 1e-08 ) 
            { 
               return Inverse( *this, Epsilon ); 
            }

            static Matrix3x3 MakeTranslation( const Vector< 3 >& Vec );
            static Matrix3x3 MakeScale( const Vector< 3 >&& Vec );
            static Matrix3x3 MakeXRotation( double Theta );
            static Matrix3x3 MakeYRotation( double Theta );
            static Matrix3x3 MakeZRotation( double Theta );
            static Matrix3x3 MakeAxisRotation( const Vector< 3 >& Vec, double Theta );

            /**
             * @brief 
             * 
             * @param[in] Scalar 
             * @return Matrix3x3 
             */
            inline static Matrix3x3 MakeScale( double Scalar ) 
            { 
               Matrix3x3 scale; 
               scale( 0, 0 ) = Scalar;
               scale( 1, 1 ) = Scalar;
               scale( 2, 2 ) = Scalar;
               scale( 3, 3 ) = 1;
               return( scale );
               //return( MakeScale( Vector< 3 >( { Scalar, Scalar, Scalar } ) ) ); 
            }
      };
   }
}

#endif

