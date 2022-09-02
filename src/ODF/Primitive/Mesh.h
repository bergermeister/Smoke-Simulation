/**
 * @file Mesh.h
 * @author Eisenberger, Edward (Edward.Eisenberger@live.com)
 * @brief 
 * @version 0.1
 * @date 2022-08-27
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef ODF_Primitive_Mesh_h
#define ODF_Primitive_Mesh_h

// StdLib Includes
#include <vector>

// ODF Includes
#include <ODF/Math/Vector.h>
#include <ODF/Math/Hash.h>
#include <ODF/Primitive/Material.h>
#include "vbo_structs.h"

enum FACE_TYPE { FACE_TYPE_ORIGINAL, FACE_TYPE_RASTERIZED, FACE_TYPE_SUBDIVIDED };

namespace ODF
{
   namespace Primitive
   {
      /**
       * @brief 
       * 
       * A class to store all objects in the scene. The quad faces of the mesh can be subdivided to improve the 
       * resolution of the radiosity solution. The original mesh is maintained for efficient occlusion testing.
       */
      class Mesh 
      {
         public:
            Mesh() 
            {
               num_faces = -1;  
               area = NULL;
               bbox = NULL; 
            }
            virtual ~Mesh();
            void Load(const std::string &input_file, ArgParser *_args);
               
               void initializeVBOs(); 
            void setupVBOs(); 
            void drawVBOs();
            void cleanupVBOs();
            double getArea(int i) const {
               assert (i >= 0 && i < num_faces);
               return area[i]; }
            void setArea(int i, double value) {
               assert (i >= 0 && i < num_faces);
               area[i] = value; }
               void Reset();
            void Cleanup();

            // ========
            // VERTICES
            int numVertices() const { return vertices.size(); }
            Vertex* addVertex(const Vec3f &pos);
            // look up vertex by index from original .obj file
            Vertex* getVertex(int i) const {
               assert (i >= 0 && i < numVertices());
               return vertices[i]; }
            // this creates a relationship between 3 vertices (2 parents, 1 child)
            void setParentsChild(Vertex *p1, Vertex *p2, Vertex *child);
            // this accessor will find a child vertex (if it exists) when given
            // two parent vertices
            Vertex* getChildVertex(Vertex *p1, Vertex *p2) const;

            // =====
            // EDGES
            int numEdges() const { return edges.size(); }
            // this efficiently looks for an edge with the given vertices, using a hash table
            Edge* getEdge(Vertex *a, Vertex *b) const;
            const edgeshashtype& getEdges() const { return edges; }

            // =================
            // ACCESS THE LIGHTS
            std::vector<Face*>& getLights() { return original_lights; }

            // ==================================
            // ACCESS THE QUADS (for ray tracing)
            int numOriginalQuads() const { return original_quads.size(); }
            Face* getOriginalQuad(int i) const {
               assert (i < numOriginalQuads());
               return original_quads[i]; }

            // =======================================
            // ACCESS THE PRIMITIVES (for ray tracing)
            int numPrimitives() const { return primitives.size(); }
            Primitive* getPrimitive(int i) const {
               assert (i >= 0 && i < numPrimitives()); 
               return primitives[i]; }
            // ACCESS THE PRIMITIVES (for radiosity)
            int numRasterizedPrimitiveFaces() const { return rasterized_primitive_faces.size(); }
            Face* getRasterizedPrimitiveFace(int i) const {
               assert (i >= 0 && i < numRasterizedPrimitiveFaces());
               return rasterized_primitive_faces[i]; }

            // ==============================================================
            // ACCESS THE SUBDIVIDED QUADS + RASTERIZED FACES (for radiosity)
            int numFaces() const { return subdivided_quads.size() + rasterized_primitive_faces.size(); }
            Face* getFace(int i) const {
               int num_faces = numFaces();
               assert (i >= 0 && i < num_faces);
               if (i < (int)subdivided_quads.size()) return subdivided_quads[i];
               else return getRasterizedPrimitiveFace(i-subdivided_quads.size()); }

            // ============================
            // CREATE OR SUBDIVIDE GEOMETRY
            void addRasterizedPrimitiveFace(Vertex *a, Vertex *b, Vertex *c, Vertex *d, Material *material) {
               addFace(a,b,c,d,material,FACE_TYPE_RASTERIZED); }
            void addOriginalQuad(Vertex *a, Vertex *b, Vertex *c, Vertex *d, Material *material) {
               addFace(a,b,c,d,material,FACE_TYPE_ORIGINAL); }
            void addSubdividedQuad(Vertex *a, Vertex *b, Vertex *c, Vertex *d, Material *material) {
               addFace(a,b,c,d,material,FACE_TYPE_SUBDIVIDED); }

            // ===============
            // OTHER ACCESSORS
            BoundingBox* getBoundingBox() const { return bbox; }

            // ===============
            // OTHER FUNCTIONS
            void Subdivision();
         
         private:
            Vec3f setupHelperForColor(Face *f, int i, int j);

            // ==============
            // REPRESENTATION
            int num_faces;
            RayTracer *raytracer;

            // length n vectors
            double *area;
            double total_area;             // the total area of the scene

            // VBOs
            GLuint mesh_quad_verts_VBO;
            GLuint mesh_quad_indices_VBO;
            GLuint mesh_textured_quad_indices_VBO;
            GLuint mesh_interior_edge_indices_VBO;
            GLuint mesh_border_edge_indices_VBO;
            std::vector<VBOPosNormalColorTexture> mesh_quad_verts; 
            std::vector<VBOIndexedQuad> mesh_quad_indices;
            std::vector<VBOIndexedQuad> mesh_textured_quad_indices;
            std::vector<VBOIndexedEdge> mesh_interior_edge_indices;
            std::vector<VBOIndexedEdge> mesh_border_edge_indices;
            // ==================================================
            // HELPER FUNCTIONS FOR CREATING/SUBDIVIDING GEOMETRY
            Vertex* AddEdgeVertex(Vertex *a, Vertex *b);
            Vertex* AddMidVertex(Vertex *a, Vertex *b, Vertex *c, Vertex *d);
            void addFace(Vertex *a, Vertex *b, Vertex *c, Vertex *d, Material *material, enum FACE_TYPE face_type);
            void removeFaceEdges(Face *f);
            void addPrimitive(Primitive *p); 

            // ==============
            // REPRESENTATION
            ArgParser *args;
            public:
            std::vector<Material*> materials;
            Vec3f background_color;
            Camera *camera;
            private:

            // the bounding box of all rasterized faces in the scene
            BoundingBox *bbox; 

            // the vertices & edges used by all quads (including rasterized primitives)
            std::vector<Vertex*> vertices;  
            edgeshashtype edges;
            vphashtype vertex_parents;

            // the quads from the .obj file (before subdivision)
            std::vector<Face*> original_quads;
            // the quads from the .obj file that have non-zero emission value
            std::vector<Face*> original_lights; 
            // all primitives (spheres, etc.)
            std::vector<Primitive*> primitives;
            // the primitives converted to quads
            std::vector<Face*> rasterized_primitive_faces;
            // the quads from the .obj file after subdivision
            std::vector<Face*> subdivided_quads;
      };
   }
}

#endif



