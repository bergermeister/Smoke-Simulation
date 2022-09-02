/**
 * @file Shade.h
 * @author Eisenberger, Edward (Edward.Eisenberger@live.com)
 * @brief 
 * @version 0.1
 * @date 2022-09-01
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ODF_Engine_Shade_h
#define ODF_Engine_Shade_h

// ODF Includes
#include <ODF/Primitive/Material.h>
#include <ODF/Engine/Hit.h>

namespace ODF
{
   namespace Engine
   {
      Math::Vector< 3 > Shade( const Engine::Ray& ray, const Engine::Hit& hit, const Primitive::Material& mat,
                               const Math::Vector< 3 >& DirToLight, 
                               const Math::Vector< 3 >& LightColor );
   }
}

#endif

