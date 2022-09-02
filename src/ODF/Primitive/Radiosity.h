/**
 * @file Radiosity.h
 * @author Eisenberger, Edward (Edward.Eisenberger@live.com)
 * @brief 
 * @version 0.1
 * @date 2022-08-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ODF_Primitive_Radiosity_h
#define ODF_Primitive_Radiosity_h

// StdLib Includes
#include <vector>

// ODF Includes
#include <ODF/Math/Vector.h>
#include <ODF/Primitive/Mesh.h>
#include <ODF/Engine/RayTracer.h>

namespace ODF
{
   namespace Primitive
   {
      /**
       * @brief 
       * 
       * This class manages the radiosity calculations, including form factors and radiance solution.
       */
      class Radiosity 
      {
         public:
            Radiosity( Mesh *m );
            ~Radiosity();
            void Reset();
            void Cleanup();
            void ComputeFormFactors();
            void setRayTracer(RayTracer *r) { raytracer = r; }
            

            // =========
            // ACCESSORS
            Mesh* getMesh() const { return mesh; }
            double getFormFactor(int i, int j) const {
               // F_i,j radiant energy leaving i arriving at j
               assert (i >= 0 && i < num_faces);
               assert (j >= 0 && j < num_faces);
               assert (formfactors != NULL);
               return formfactors[i*num_faces+j]; }
            double getArea(int i) const {
               assert (i >= 0 && i < num_faces);
               return area[i]; }
            Vec3f getUndistributed(int i) const {
               assert (i >= 0 && i < num_faces);
               return undistributed[i]; }
            Vec3f getAbsorbed(int i) const {
               assert (i >= 0 && i < num_faces);
               return absorbed[i]; }
            Vec3f getRadiance(int i) const {
               assert (i >= 0 && i < num_faces);
               return radiance[i]; }
            
            // =========
            // MODIFIERS
            double Iterate();
            void setFormFactor(int i, int j, double value) { 
               assert (i >= 0 && i < num_faces);
               assert (j >= 0 && j < num_faces);
               assert (formfactors != NULL);
               formfactors[i*num_faces+j] = value; }
            void normalizeFormFactors(int i) {
               double sum = 0;
               int j;
               for (j = 0; j < num_faces; j++) {
                  sum += getFormFactor(i,j); }
               if (sum == 0) return;
               for (j = 0; j < num_faces; j++) {
                  setFormFactor(i,j,getFormFactor(i,j)/sum); } }
            void setArea(int i, double value) {
               assert (i >= 0 && i < num_faces);
               area[i] = value; }
            void setUndistributed(int i, Vec3f value) { 
               assert (i >= 0 && i < num_faces);
               undistributed[i] = value; }
            void findMaxUndistributed();
            void setAbsorbed(int i, Vec3f value) { 
               assert (i >= 0 && i < num_faces);
               absorbed[i] = value; }
            void setRadiance(int i, Vec3f value) { 
               assert (i >= 0 && i < num_faces);
               radiance[i] = value; }

            private:
               Vec3f setupHelperForColor(Face *f, int i, int j);

               // ==============
               // REPRESENTATION
               Mesh *mesh;
               ArgParser *args;
               int num_faces;
               RayTracer *raytracer;
               int num_form_factor_samples;

               // a nxn matrix
               // F_i,j radiant energy leaving i arriving at j
               double *formfactors;

               // length n vectors
               double *area;
               Vec3f *undistributed; // energy per unit area
               Vec3f *absorbed;      // energy per unit area
               Vec3f *radiance;      // energy per unit area

               int max_undistributed_patch;  // the patch with the most undistributed energy
               double total_undistributed;    // the total amount of undistributed light
               double total_area;             // the total area of the scene
      };
   }
}

#endif
