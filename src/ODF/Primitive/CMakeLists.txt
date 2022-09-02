target_sources( OpenGLFramework
   PRIVATE
      Material.cpp
      Mesh.cpp
      Radiosity.cpp
)

set( publicHeaders 
   Material.h
   Mesh.h
   Radiosity.h
   Vertex.h
)

install( FILES publicHeaders DESTINATION Lib/ODF/Primitive )

