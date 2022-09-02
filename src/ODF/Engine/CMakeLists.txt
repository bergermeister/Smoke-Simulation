target_sources( OpenGLFramework
   PRIVATE
      Camera.cpp
      Hit.cpp
      #OrthographicCamera.cpp
      #PerspectiveCamera.cpp
      #RayTracer.cpp
      Shade.cpp
)

set( publicHeaders 
   Camera.h
   Hit.h
   OrthographicCamera.h
   RespectiveCamera.h
   Ray.h
   RayTracer.h
)

install( FILES publicHeaders DESTINATION Lib/OpenGLFramework/ODF/Engine )

