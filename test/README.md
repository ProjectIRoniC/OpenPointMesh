####Testing Suite Setup
#####Steps
1. Install gtest: follow the directions at this link: https://github.com/ProjectIRoniC/OpenPointMesh/wiki/Compile-OpenPointMesh-on-Ubuntu-14.04-Without-SuperBuild#google-test-170-gtest
2. `cd OpenPointMesh/test/`
3. `mkdir build`
4. `cd build`
5. `cmake ..`
6. `make`

#####Explanation
1. The gtest libraries need to be compiled and installed for the rest of these steps to work.
2. Navigate to the test root folder
3. We need to create a build folder to contain all the cmake files
4. Enter that directory
5. This sets up the cmake project
6. Compiles the test programs. Several executables are compiled: one for each component. When you run each executable, it will
   run the tests over that specific component. 
