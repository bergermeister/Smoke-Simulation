#include "glCanvas.h"

#include <iostream> 
#include "argparser.h"

// =========================================
// =========================================

int main(int argc, char *argv[]) {
  ArgParser args(argc, argv);
  if (args.cloth_file == "" && args.smoke_file == "" && args.smoke_file == "") {
    std::cout << "ERROR: no simulation specified" << std::endl;
    return 0;
  }
  glutInit(&argc,argv);
  GLCanvas::initialize(&args);
  return 0;
}

// =========================================
// =========================================

