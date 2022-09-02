# Smoke Simulation
## Authors
Edward Eisenberger, Maria Montenegro

## Details
This is the final project for our Advanced Computer Graphics course. We will simulate
and render smoke in three dimensions.

## Environment
<table>
    <tr><th colspan=3> Tool Information
    <tr><th> Name <th> Version <th> Location
    <tr><td> Ubuntu <td> 22.04.1 <td> OS: https://releases.ubuntu.com/22.04/ <br>
                                      WSL: https://www.microsoft.com/store/productId/9PN20MSR04DW
    <tr><td> CMake <td> 3.22.1 <td> sudo apt-get install cmake
    <tr><td> gcc <td> 11.2.0 <td> sudo apt update <br>
                                  sudo apt install build-essential
    <tr><td> g++ <td> 11.2.0 <td> sudo apt update <br>
                                  sudo apt install build-essential
    <tr><td> (optional) doxygen <td> 1.9.1 <td> sudo apt-add-repository universe <br> 
                                                sudo apt-get update <br>
                                                sudo apt-get install doxygen
    <tr><td> (optional) lcov <td> <td> sudo apt install lcov
</table>

## Building, Testing, and Code Coverage
    cmake --preset gcc-debug
    cmake --build --preset gcc-debug -j
    ctest --preset gcc-debug
    lcov --capture --directory build/gcc --output-file doc/coverage.info 
    genhtml doc/coverage.info --output-directory doc/coverage
    gprof build/tst/GTestCrypto.exe build/tst/gmon.out > doc/profiling.txt