// ODF Includes
#include <ODF/Math/Matrix4x4.h>

namespace ODF
{
   namespace Math
   {
      Matrix4x4& Matrix4x4::operator=( const SquareMatrix< 4 >& Mat )
      {
         ( ( Matrix< Size, Size >* )this )->operator=( *( const Matrix< Size, Size >* )&Mat );
         return( *this );
      }

      Matrix4x4& Matrix4x4::operator*=( const Matrix4x4& Mat )
      {
         ( ( Matrix< Size, Size >* )this )->operator*=( *( const Matrix< Size, Size >* )&Mat );
         return( *this );
      }

      Matrix4x4 Matrix4x4::AxisRotation( const Vector< 3 >& Vec, double Theta )
      {
         Matrix4x4 result;
         double x = Vec[ 0 ];
         double y = Vec[ 1 ];
         double z = Vec[ 2 ];
         double c = static_cast< double >( cosf( static_cast< float >( Theta ) ) );
         double s = static_cast< double >( sinf( static_cast< float >( Theta ) ) );
         double xx = x * x;
         double xy = x * y;
         double xz = x * z;
         double yy = y * y;
         double yz = y * z;
         double zz = z * z;

         *dynamic_cast< SquareMatrix< Size >* >( &result ) = SquareMatrix< Size >::Identity( );
         result( 0, 0 ) = ( ( 1 - c ) * xx ) + c;
         result( 1, 0 ) = ( ( 1 - c ) * xy ) + ( z * s );
         result( 2, 0 ) = ( ( 1 - c ) * xz ) - ( y * s );
         result( 0, 1 ) = ( ( 1 - c ) * xy ) - ( z * s );
         result( 1, 1 ) = ( ( 1 - c ) * yy ) + c;
         result( 2, 1 ) = ( ( 1 - c ) * yz ) + ( x * s );
         result( 0, 2 ) = ( ( 1 - c ) * xz ) + ( y * s );
         result( 1, 2 ) = ( ( 1 - c ) * yz ) - ( x * s );
         result( 2, 2 ) = ( ( 1 - c ) * zz ) + c;

         return( result );
      }

      Matrix4x4 Matrix4x4::MakeTranslation( const Math::Vector< 3 >& Vec) 
      {
         Matrix4x4 transform;
         *dynamic_cast< SquareMatrix< Size >* >( &transform ) = SquareMatrix< Size >::Identity( );
         transform( 0, 3 ) = Vec[ 0 ];
         transform( 1, 3 ) = Vec[ 1 ];
         transform( 2, 3 ) = Vec[ 2 ];
         return( transform );
      }


      Matrix4x4 Matrix4x4::MakeScale( const Math::Vector< 3 >&& Vec )
      {
         Matrix4x4 scale; 
         scale( 0, 0 ) = Vec[ 0 ];
         scale( 1, 1 ) = Vec[ 1 ];
         scale( 2, 2 ) = Vec[ 2 ];
         scale( 3, 3 ) = 1;
         return( scale );
      }


      Matrix4x4 Matrix4x4::MakeXRotation( double Theta ) 
      {
         Matrix4x4 rx;
         *dynamic_cast< SquareMatrix< Size >* >( &rx ) = SquareMatrix< Size >::Identity( );
         rx( 1, 1 ) =  ( double )cos( ( double )Theta );
         rx( 1, 2 ) = -( double )sin( ( double )Theta );
         rx( 2, 1 ) =  ( double )sin( ( double )Theta );
         rx( 2, 2 ) =  ( double )cos( ( double )Theta );
         return( rx );
      }

      Matrix4x4 Matrix4x4::MakeYRotation( double Theta )
      {
         Matrix4x4 ry;
         *dynamic_cast< SquareMatrix< Size >* >( &ry ) = SquareMatrix< Size >::Identity( );
         ry( 0, 0 ) =  ( double )cos( ( double )Theta );
         ry( 0, 2 ) =  ( double )sin( ( double )Theta );
         ry( 2, 0 ) = -( double )sin( ( double )Theta );
         ry( 2, 2 ) =  ( double )cos( ( double )Theta );
         return( ry );
      }

      Matrix4x4 Matrix4x4::MakeZRotation( double Theta )
      {
         Matrix4x4 rz;
         *dynamic_cast< SquareMatrix< Size >* >( &rz ) = SquareMatrix< Size >::Identity( );
         rz( 0, 0 ) =  ( double )cos( ( double )Theta );
         rz( 0, 1 ) = -( double )sin( ( double )Theta );
         rz( 1, 0 ) =  ( double )sin( ( double )Theta );
         rz( 1, 1 ) =  ( double )cos( ( double )Theta );
         return( rz );
      }


      Matrix4x4 Matrix4x4::MakeAxisRotation( const Math::Vector< 3 >& Vec, double Theta )
      {
         Matrix4x4 r;
         *dynamic_cast< SquareMatrix< Size >* >( &r ) = SquareMatrix< Size >::Identity( );

         double x = Vec[ 0 ]; 
         double y = Vec[ 1 ]; 
         double z = Vec[ 2 ];

         double c = ( double )cosf( float( Theta ) );
         double s = ( double )sinf( float( Theta ) );
         double xx = x * x;
         double xy = x * y;
         double xz = x * z;
         double yy = y * y;
         double yz = y * z;
         double zz = z * z;

         r( 0, 0 ) = ( ( 1 - c ) * xx ) + c;
         r( 1, 0 ) = ( ( 1 - c ) * xy ) + ( z * s );
         r( 2, 0 ) = ( ( 1 - c ) * xz ) - ( y * s );

         r( 0, 1 ) = ( ( 1 - c ) * xy ) - ( z * s );
         r( 1, 1 ) = ( ( 1 - c ) * yy ) + c;
         r( 2, 1 ) = ( ( 1 - c ) * yz ) + ( x * s );

         r( 0, 2 ) = ( ( 1 - c ) * xz ) + ( y * s );
         r( 1, 2 ) = ( ( 1 - c ) * yz ) - ( x * s );
         r( 2, 2 ) = ( ( 1 - c ) * zz ) + c;

         return( r );
      }      
   }
}

