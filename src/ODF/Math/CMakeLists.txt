target_sources( OpenGLFramework
   PRIVATE
      Matrix3x3.cpp
      Matrix4x4.cpp
)

set( publicHeaders 
   MersenneTwister/Random.h
   Hash.h
   Matrix.h
   Matrix3x3.h
   Matrix4x4.h
   SquareMatrix.h
   Vector.h
)

install( FILES publicHeaders DESTINATION Lib/ODF/Math )

