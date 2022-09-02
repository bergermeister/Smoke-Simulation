/**
 * @file RayTracer.h
 * @author Eisenberger, Edward (Edward.Eisenberger@live.com)
 * @brief 
 * @version 0.1
 * @date 2022-08-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ODF_Primitive_RayTracer
#define ODF_Primitive_RayTracer

// StdLib Include
#include <vector>
#include <map>

// ODF Includes
#include <ODF/Primitive/Ray.h>

namespace ODF
{
   namespace Primitive
   {
      class RayTracer 
      {
         public:     // Public Methods
            RayTracer(Smoke *s, ArgParser *a,Mesh *m) {
               smoke = s;
               args = a;
               mesh = m;
               num_form_factor_samples = 10;
            }  

            // casts a single ray through the scene geometry and finds the closest hit
            bool CastRay( const Ray& ray, Hit& h, bool UseSpherePatches ) const;
            bool ParticleInGrid( const Math::Vector< 3 > Position, const BoundingBox* b ) const;
            bool ParticleInCircle( const Math::Vector< 3 > Position, const Math::Vector< 3 > Center, 
                                   float Radius, float Width ) const;

            // does the recursive work
            Math::Vector< 3 > TraceRay( Ray& ray, Hit& hit, int bounceCount = 0 ) const;
            static Radiosity *radiosity;

         private:    // Private Methods

            Vec3f Trace(const Ray &ray, Face *f, Vec3f end) const;
            Vec3f Scattering(const Ray &ray,std::vector<SmokeParticle*>pr,int numParticles,Vec3f from,float radius,float width,float T,Face *f) const;
            float CylTest_CapsFirst( const Vec3f & pt1, const Vec3f & pt2, float lengthsq, float radius_sq, const Vec3f & testpt ) const;
            // REPRESENTATION
            Smoke *smoke;
            ArgParser *args;
            Mesh *mesh;


            //RADIOSITY
               // length n vectors
            double *area;
            Vec3f *undistributed; // energy per unit area
            Vec3f *absorbed;      // energy per unit area
            Vec3f *radiance;      // energy per unit area

            int max_undistributed_patch;  // the patch with the most undistributed energy
            double total_undistributed;    // the total amount of undistributed light
            double total_area;             // the total area of the scene

               // a nxn matrix F_i,j radiant energy leaving i arriving at j
            double *formfactors;
            int num_form_factor_samples;
            void ComputeFormFactors() const;
            void Iterate() const; 

            void setArea(int i, double value);
            void setUndistributed(int i, Vec3f value);
            void findMaxUndistributed();
            void setAbsorbed(int i, Vec3f value);
            void setRadiance(int i, Vec3f value);
      };
   }
}

#endif
