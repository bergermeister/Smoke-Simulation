/**
 * @file
 * 
 */
#ifndef ODF_Math_Vector_h
#define ODF_Math_Vector_h

// StdLib Includes
#include <cstdint>
#include <cstring>
#include <cassert>
#include <math.h>
#include <iostream>

namespace ODF
{
   namespace Math
   {
      template< size_t Dimensions > class Vector
      {
         public:     // Public Attriutes
            static constexpr size_t Dimension = Dimensions;

         protected:  // Protected Attributes
            double direction[ Dimension ];

         public:     // Public Methods
            /**
             * @brief Construct a new Vector object
             * 
             * @param[in] Direction Initializer list of [0:Dimension] directions
             * @return This method returns nothing.
             */
            Vector( std::initializer_list< double > Direction )
            {
               assert( Direction.size( ) <= Dimension );
               uint32_t index = 0;
               if( Direction.size( ) < Dimension )
               {
                  std::memset( reinterpret_cast< void* >( this->direction ), 0, Dimension * sizeof( double ) );
               }
               for( auto iter = Direction.begin( ); iter != Direction.end( ); iter++ )
               {
                  direction[ index++ ] = *iter;
               }
            }

            /**
             * @brief Construct a new Vector object
             * 
             * @param[in] V
             * @return This method returns nothing. 
             */
            Vector( const Vector& V )
            {
               *this = V;
            }

            /**
             * @brief 
             * 
             * @param[in] V 
             * @return This method returns a reference to this object.
             */
            Vector& operator=( const Vector& V )
            {
               if( this != &V )
               {
                  std::memcpy( reinterpret_cast< void* >( this->direction ),
                               reinterpret_cast< const void* >( V.direction ), 
                               sizeof( double ) * Dimension );
               }
               return( *this );
            }

            /**
             * @brief 
             * 
             * @param[in] Index 
             * @return double 
             */
            double operator[ ]( const size_t Index ) const
            { 
               assert( Index < Dimension );
               return( this->direction[ Index ] );
            }

            /**
             * @brief 
             * 
             * @param[in] Index 
             * @return double& 
             */
            double& operator[ ]( const size_t Index )
            {
               assert( Index < Dimension );
               return( this->direction[ Index ] );
            }

            /**
             * @brief 
             * 
             * @param[in] V 
             * @return This method returns true if this Vector equals the given Vector.
             */
            bool operator==( const Vector& V ) const
            {
               bool equal = true;
               for( auto index = 0; index < Dimension; index++ )
               {
                  if( this->direction[ index ] != V.direction[ index ] )
                  {
                     equal = false;
                     break;
                  }
               }
               return( equal );
            }

            Vector operator+( const Vector& Vec ) const
            {
               Vector result( { } );
               for( size_t index = 0; index < Dimension; index++ )
               {
                  result.direction[ index ] = this->direction[ index ] + Vec.direction[ index ];
               }
               return( result );
            }

            Vector operator-( const Vector& Vec ) const
            {
               Vector result( { } );
               for( size_t index = 0; index < Dimension; index++ )
               {
                  result.direction[ index ] = this->direction[ index ] - Vec.direction[ index ];
               }
               return( result );
            }

            Vector operator*( const Vector& V2 ) const
            {
               Vector v3( *this );
               for( size_t index = 0; index < Dimension; index++ )
               {
                  v3.direction[ index ] *= V2.direction[ index ];
               } 
               return( v3 );
            }

            Vector operator*( const double Scalar ) const
            {
               Vector result( { } );
               for( size_t index = 0; index < Dimension; index++ )
               {
                  result.direction[ index ] = this->direction[ index ] * Scalar;
               }
               return( result );
            }

            /**
             * @brief 
             * 
             * @param[in] V 
             * @return This method returns true if this Vector does not equal the given Vector.
             */
            bool operator!=( const Vector& V ) const
            {
               return( !this->operator==( V ) );
            }

            Vector& operator+=( const Vector& V )
            {
               for( size_t index = 0; index < Dimension; index++ )
               {
                  this->direction[ index ] += V.direction[ index ];
               }
               return( *this );
            }

            Vector& operator-=( const Vector& V )
            {
               for( size_t index = 0; index < Dimension; index++ )
               {
                  this->direction[ index ] -= V.direction[ index ];
               }
               return( *this );
            }

            Vector& operator*=( double Scalar )
            {
               for( size_t index = 0; index < Dimension; index++ )
               {
                  this->direction[ index ] *= Scalar;
               }
               return( *this );
            }

            Vector& operator/=( double Scalar )
            {
               for( size_t index = 0; index < Dimension; index++ )
               {
                  this->direction[ index ] /= Scalar;
               }
               return( *this );
            }

            /**
             * @brief 
             * 
             * @param None
             * @return double 
             */
            double Length( void ) const 
            {
               double sum = 0.0;
               for( size_t index = 0; index < Dimension; index++ )
               {
                  sum += this->direction[ index ] * this->direction[ index ];
               }
               return( sqrt( sum ) );
            }
            
            void Normalize( void )
            {
               double length = Length( );
               if( length > 0 )
               {
                  for( size_t index = 0; index < Dimension; index++ )
                  {
                     this->direction[ index ] /= length;
                  }
               }
            }

            void Scale( std::initializer_list< double > Value )
            {
               uint32_t index = 0;
               for( auto iter = Value.begin( ); iter != Value.end( ); iter++ )
               {
                  direction[ index++ ] *= *iter;
               }
            }

            void Negate( void ) 
            { 
               this->Scale *= -1.0;
            }

            double Dot( const Vector& V ) const
            {
               double dotProduct = 0;
               for( auto index = 0; index < Dimension; index++ )
               {
                  dotProduct += this->direction[ index ] * V.direction[ index ];
               }
               return( dotProduct );
            }

            static Vector Cross( const Vector& V1, const Vector& V2 )
            {
               assert( Dimension > 2 );
               double x = ( V1.direction[ 1 ] * V2.direction[ 2 ] ) - ( V1.direction[ 2 ] * V2.direction[ 1 ] );
               double y = ( V1.direction[ 2 ] * V2.direction[ 0 ] ) - ( V1.direction[ 0 ] * V2.direction[ 2 ] );
               double z = ( V1.direction[ 0 ] * V2.direction[ 1 ] ) - ( V1.direction[ 1 ] * V2.direction[ 0 ] );
               return( Vector( { x, y, z } ) );
            }

            friend std::ostream& operator<<( std::ostream& OutStream, const Vector& V )
            {
               size_t index;
               for( index = 0; index < ( Vector::Dimension - 1 ); index++ )
               {
                  OutStream << V.direction[ index ] << " ";
               }
               OutStream << V.direction[ index ] << std::endl;
               return( OutStream );
            }

            friend std::istream& operator>>( std::istream& InStream, Vector& V )
            {
               for( auto index = 0; index < Vector::Dimension; index++ )
               {
                  InStream >>  V.direction[ index ];
               }
               return( InStream );
            }
      };
   }
}

#endif
