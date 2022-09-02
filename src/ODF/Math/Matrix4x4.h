/**
 * @file Matrix4x4.h
 * @author Eisenberger, Edward (Edward.Eisenberger@live.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-02
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ODF_Math_Matrix4x4_h
#define ODF_Math_Matrix4x4_h

// ODF Includes
#include <ODF/Math/SquareMatrix.h>

namespace ODF
{
   namespace Math
   {
      class Matrix4x4 : public SquareMatrix< 4 >
      {
         public:     // Public Attriutes
            static constexpr size_t Size = 4;

         public:     // Public Methods
            Matrix4x4( void );
            Matrix4x4( const Matrix4x4& Mat4x4 );
            virtual ~Matrix4x4( void ) = default;
            Matrix4x4& operator=( const SquareMatrix< 4 >& Mat );
            Matrix4x4& operator*=( const Matrix4x4& Mat );

            Matrix4x4 AxisRotation( const Vector< 3 >& Vec, double Theta );
            Matrix4x4 Transpose( void ) const;            
            int Inverse( Matrix4x4& M, double Epsilon = 1e-08 ) const;
            
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

            static Matrix4x4 MakeTranslation( const Vector< 3 >& Vec );
            static Matrix4x4 MakeScale( const Vector< 3 >&& Vec );
            static Matrix4x4 MakeXRotation( double Theta );
            static Matrix4x4 MakeYRotation( double Theta );
            static Matrix4x4 MakeZRotation( double Theta );
            static Matrix4x4 MakeAxisRotation( const Vector< 3 >& Vec, double Theta );

            /**
             * @brief 
             * 
             * @param[in] Scalar 
             * @return Matrix4x4 
             */
            inline static Matrix4x4 MakeScale( double Scalar ) 
            { 
               Matrix4x4 scale; 
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

