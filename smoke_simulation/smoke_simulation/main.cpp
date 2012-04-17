#include "glCanvas.h"
#include "raytracer.h"
#include <iostream> 
#include "mesh.h"
#include "argparser.h"

// =========================================
// =========================================

int main(int argc, char *argv[]) {
  ArgParser *args = new ArgParser(argc, argv);
  if (args->mesh_file == "" && args->smoke_file == "") {
    std::cout << "ERROR: no simulation specified" << std::endl;
    return 0;
  }
  glutInit(&argc,argv);
  Mesh *mesh = new Mesh();
  mesh->Load(args->mesh_file, args);
  GLCanvas::initialize(args,mesh);
  return 0;
}

// =========================================
// =========================================

