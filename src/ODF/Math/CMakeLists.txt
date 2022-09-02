target_sources( OpenGLFramework
   PRIVATE
      Matrix4x4.cpp
)

set( publicHeaders 
   MersenneTwister/Random.h
   Hash.h
   Matrix.h
   Matrix4x4.h
   SquareMatrix.h
   Vector.h
)

install( FILES publicHeaders DESTINATION Lib/ODF/Math )

