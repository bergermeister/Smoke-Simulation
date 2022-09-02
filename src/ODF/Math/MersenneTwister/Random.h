// MersenneTwister.h
// Mersenne Twister random number generator -- a C++ class Random
// Based on code by Makoto Matsumoto, Takuji Nishimura, and Shawn Cokus
// Richard J. Wagner  v1.1  28 September 2009  wagnerr@umich.edu

// The Mersenne Twister is an algorithm for generating random numbers.  It
// was designed with consideration of the flaws in various other generators.
// The period, 2^19937-1, and the order of equidistribution, 623 dimensions,
// are far greater.  The generator is also fast; it avoids multiplication and
// division, and it benefits from caches and pipelines.  For more information
// see the inventors' web page at
// http://www.math.sci.hiroshima-u.ac.jp/~m-mat/MT/emt.html

// Reference
// M. Matsumoto and T. Nishimura, "Mersenne Twister: A 623-Dimensionally
// Equidistributed Uniform Pseudo-Random Number Generator", ACM Transactions on
// Modeling and Computer Simulation, Vol. 8, No. 1, January 1998, pp 3-30.

// Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
// Copyright (C) 2000 - 2009, Richard J. Wagner
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 
//   1. Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//   2. Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//   3. The names of its contributors may not be used to endorse or promote 
//      products derived from this software without specific prior written 
//      permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// The original code included the following notice:
// 
//     When you use this, send an email to: m-mat@math.sci.hiroshima-u.ac.jp
//     with an appropriate reference to your work.
// 
// It would be nice to CC: wagnerr@umich.edu and Cokus@math.washington.edu
// when you write.

#ifndef ODF_Math_MersenneTwister_Random_h
#define ODF_Math_MersenneTwister_Random_h

// StdLib Includes
#include <iostream>
#include <climits>
#include <cstdio>
#include <cstdint>
#include <ctime>
#include <cmath>

namespace ODF
{
   namespace Math
   {
      namespace MersenneTwister
      {
         /**
          * @brief 
          * 
          * @note Not thread safe (unless auto-initialization is avoided and each thread has its own Random object)
          */
         class Random
         {
            public:     // Public Attributes
               enum { N = 624 };       // length of state vector
               enum { SAVE = N + 1 };  // length of array for save()

            protected:  // Proteced Attributes
               enum { M = 397 };  // period parameter
               
               uint32_t state[ N ];    // internal state
               uint32_t* pNext;        // next value to get from state
               int left;               // number of values left before reload needed

            public:     // Public Methods
                 // initialize with a simple uint32_t
               Random( const uint32_t oneSeed )
               {
                  seed(oneSeed);
               }

               // or array
               Random( uint32_t* const bigSeed, uint32_t const seedLength = N )
               {  
                  seed(bigSeed,seedLength); 
               }

               // auto-initialize with /dev/urandom or time() and clock()
               Random( void )
               { 
                  seed();
               }  

               // copy
               Random( const Random& o )
               {
                  register const uint32_t *t = o.state;
                  register uint32_t *s = state;
                  register int i = N;
                  for( ; i--; *s++ = *t++ ) {}
                  left = o.left;
                  pNext = &state[N-left];
               }

               // Do NOT use for CRYPTOGRAPHY without securely hashing several returned
               // values together, otherwise the generator state can be learned after
               // reading 624 consecutive values.
               
               // Access to 32-bit random numbers
               // integer in [0,2^32-1]
               uint32_t randInt()
               {
                  // Pull a 32-bit integer from the generator state
                  // Every other access function simply transforms the numbers extracted here
                  
                  if( left == 0 ) reload();
                  --left;
                  
                  register uint32_t s1;
                  s1 = *pNext++;
                  s1 ^= (s1 >> 11);
                  s1 ^= (s1 <<  7) & 0x9d2c5680UL;
                  s1 ^= (s1 << 15) & 0xefc60000UL;
                  return ( s1 ^ (s1 >> 18) );
               }

               // integer in [0,n] for n < 2^32    
               uint32_t randInt( const uint32_t n )     
               {
                  // Find which bits are used in n
                  // Optimized by Magnus Jonsson (magnus@smartelectronix.com)
                  uint32_t used = n;
                  used |= used >> 1;
                  used |= used >> 2;
                  used |= used >> 4;
                  used |= used >> 8;
                  used |= used >> 16;
                  
                  // Draw numbers until one is found in [0,n]
                  uint32_t i;
                  do
                     i = randInt() & used;  // toss unused bits to shorten search
                  while( i > n );
                  return i;
               }           
               
               // real number in [0,1]
               inline double rand()
               { 
                  return double(randInt()) * (1.0/4294967295.0); 
               }

               // real number in [0,n]
               inline double rand( const double n )
               { 
                  return rand() * n; 
               }

               // real number in [0,1)
               inline double randExc()
               { 
                  return double(randInt()) * (1.0/4294967296.0); 
               }

               // real number in [0,n)
               inline double randExc( const double n )
               { 
                  return randExc() * n; 
               }

               // real number in (0,1)
               inline double randDblExc()
               { 
                  return ( double(randInt()) + 0.5 ) * (1.0/4294967296.0); 
               }

               // real number in (0,n)
               inline double randDblExc( const double n )
               { 
                  return randDblExc() * n; 
               };  
               
               // same as rand()
               inline double operator()()                  
               {
                  return rand();
               }
               
               // Access to 53-bit random numbers (capacity of IEEE double precision)
               // real number in [0,1)
               inline double rand53()
               {
                  uint32_t a = randInt() >> 5, b = randInt() >> 6;
                  return ( a * 67108864.0 + b ) * (1.0/9007199254740992.0);  // by Isaku Wada
               }  
               
               // Access to nonuniform random number distributions
               inline double randNorm( const double mean = 0.0, const double stddev = 1.0 )
               {
                  // Return a real number from a normal (Gaussian) distribution with given
                  // mean and standard deviation by polar form of Box-Muller transformation
                  double x, y, r;
                  do
                  {
                     x = 2.0 * rand() - 1.0;
                     y = 2.0 * rand() - 1.0;
                     r = x * x + y * y;
                  }
                  while ( r >= 1.0 || r == 0.0 );
                  double s = sqrt( -2.0 * log(r) / r );
                  return mean + x * s * stddev;
               }
               
               // Re-seeding functions with same behavior as initializers
               inline void seed( const uint32_t oneSeed )
               {
                  // Seed the generator with a simple uint32_t
                  initialize(oneSeed);
                  reload();
               }

               inline void seed( uint32_t *const bigSeed, const uint32_t seedLength = N )
               {
                  // Seed the generator with an array of uint32_t's
                  // There are 2^19937-1 possible initial states.  This function allows
                  // all of those to be accessed by providing at least 19937 bits (with a
                  // default seed length of N = 624 uint32_t's).  Any bits above the lower 32
                  // in each element are discarded.
                  // Just call seed() if you want to get array from /dev/urandom
                  initialize(19650218UL);
                  register int i = 1;
                  register uint32_t j = 0;
                  register int k = ( N > seedLength ? N : seedLength );
                  for( ; k; --k )
                  {
                     state[i] =
                     state[i] ^ ( (state[i-1] ^ (state[i-1] >> 30)) * 1664525UL );
                     state[i] += ( bigSeed[j] & 0xffffffffUL ) + j;
                     state[i] &= 0xffffffffUL;
                     ++i;  ++j;
                     if( i >= N ) { state[0] = state[N-1];  i = 1; }
                     if( j >= seedLength ) j = 0;
                  }
                  for( k = N - 1; k; --k )
                  {
                     state[i] =
                     state[i] ^ ( (state[i-1] ^ (state[i-1] >> 30)) * 1566083941UL );
                     state[i] -= i;
                     state[i] &= 0xffffffffUL;
                     ++i;
                     if( i >= N ) { state[0] = state[N-1];  i = 1; }
                  }
                  state[0] = 0x80000000UL;  // MSB is 1, assuring non-zero initial array
                  reload();
               }

               inline void seed()
               {
                  // Seed the generator with an array from /dev/urandom if available
                  // Otherwise use a hash of time() and clock() values
                  
                  // First try getting an array from /dev/urandom
               #if _WIN32
                     FILE* urandom;
                     fopen_s(&urandom, "/dev/urandom", "rb" );
               #else
                  FILE* urandom = fopen( "/dev/urandom", "rb" );
               #endif
                  if( urandom )
                  {
                     uint32_t bigSeed[N];
                     register uint32_t *s = bigSeed;
                     register int i = N;
                     register bool success = true;
                     while( success && (bool)((i--)!=0) )
                                 success = (bool)(fread( s++, sizeof(uint32_t), 1, urandom ) != 0);
                     fclose(urandom);
                     if( success ) { seed( bigSeed, N );  return; }
                  }
                  
                  // Was not successful, so use time() and clock() instead
                  seed( hash( time(NULL), clock() ) );
               }
               
               // Saving and loading generator state
               inline void save( uint32_t* saveArray ) const
               {
                  register const uint32_t *s = state;
                  register uint32_t *sa = saveArray;
                  register int i = N;
                  for( ; i--; *sa++ = *s++ ) {}
                  *sa = left;
               }

               inline void load( uint32_t *const loadArray )
               {
                  register uint32_t *s = state;
                  register uint32_t *la = loadArray;
                  register int i = N;
                  for( ; i--; *s++ = *la++ ) {}
                  left = *la;
                  pNext = &state[N-left];
               }

               inline friend std::ostream& operator<<( std::ostream& os, const Random& Random )
               {
                  register const uint32_t *s = Random.state;
                  register int i = Random.N;
                  for( ; i--; os << *s++ << "\t" ) {}
                  return os << Random.left;
               }

               inline friend std::istream& operator>>( std::istream& is, Random& Random )
               {
                  register uint32_t *s = Random.state;
                  register int i = Random.N;
                  for( ; i--; is >> *s++ ) {}
                  is >> Random.left;
                  Random.pNext = &Random.state[Random.N-Random.left];
                  return is;
               }

               inline Random& operator=( const Random& o )
               {
                  if( this == &o ) return (*this);
                  register const uint32_t *t = o.state;
                  register uint32_t *s = state;
                  register int i = N;
                  for( ; i--; *s++ = *t++ ) {}
                  left = o.left;
                  pNext = &state[N-left];
                  return (*this);
               }

            protected:
               inline void initialize( const uint32_t Seed )
               {
                  // Initialize generator state with seed
                  // See Knuth TAOCP Vol 2, 3rd Ed, p.106 for multiplier.
                  // In previous versions, most significant bits (MSBs) of the seed affect
                  // only MSBs of the state array.  Modified 9 Jan 2002 by Makoto Matsumoto.
                  register uint32_t *s = state;
                  register uint32_t *r = state;
                  register int i = 1;
                  *s++ = Seed & 0xffffffffUL;
                  for( ; i < N; ++i )
                  {
                     *s++ = ( 1812433253UL * ( *r ^ (*r >> 30) ) + i ) & 0xffffffffUL;
                     r++;
                  }
               }

               inline void reload( void )
               {
                  // Generate N new values in state
                  // Made clearer and faster by Matthew Bellew (matthew.bellew@home.com)
                  static const int MmN = int(M) - int(N);  // in case enums are unsigned
                  register uint32_t *p = state;
                  register int i;
                  for( i = N - M; i--; ++p )
                     *p = twist( p[M], p[0], p[1] );
                  for( i = M; --i; ++p )
                     *p = twist( p[MmN], p[0], p[1] );
                  *p = twist( p[MmN], p[0], state[0] );
                  
                  left = N, pNext = state;
               }
               
               uint32_t hiBit( const uint32_t u ) const 
               { 
                  return u & 0x80000000UL; 
               }
               uint32_t loBit( const uint32_t u ) const 
               { 
                  return u & 0x00000001UL; 
               }
               uint32_t loBits( const uint32_t u ) const 
               { 
                  return u & 0x7fffffffUL; 
               }
               uint32_t mixBits( const uint32_t u, const uint32_t v ) const
                  { return hiBit(u) | loBits(v); }
               uint32_t magic( const uint32_t u ) const
                  { return loBit(u) ? 0x9908b0dfUL : 0x0UL; }
               uint32_t twist( const uint32_t m, const uint32_t s0, const uint32_t s1 ) const
                  { return m ^ (mixBits(s0,s1)>>1) ^ magic(s1); }
               static inline uint32_t hash( time_t t, clock_t c )
               {
                  // Get a uint32_t from t and c
                  // Better than uint32_t(x) in case x is floating point in [0,1]
                  // Based on code by Lawrence Kirby (fred@genesis.demon.co.uk)
                  static uint32_t differ = 0;  // guarantee time-based seeds will change
                  
                  uint32_t h1 = 0;
                  unsigned char *p = (unsigned char *) &t;
                  for( size_t i = 0; i < sizeof(t); ++i )
                  {
                     h1 *= UCHAR_MAX + 2U;
                     h1 += p[i];
                  }
                  uint32_t h2 = 0;
                  p = (unsigned char *) &c;
                  for( size_t j = 0; j < sizeof(c); ++j )
                  {
                     h2 *= UCHAR_MAX + 2U;
                     h2 += p[j];
                  }
                  return ( h1 + differ++ ) ^ h2;
               }
         };        
      }
   }
}

#endif 

