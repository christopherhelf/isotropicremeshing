mex -g -v -I./ -I/usr/local/include -L/usr/local/lib/OpenMesh -lOpenMeshTools -lOpenMeshCore resampling.c ./src/BSP.cpp ./src/BSPTraits.cpp ./src/IsotropicRemesher.cpp -DOM_FORCE_STATIC_CAST
load('./data/matlab.mat');
resampling(faces, points, int32([]), 0.2, int32(1))

mex -g -v -I./ -I/usr/local/include -L/usr/local/lib/OpenMesh -lOpenMeshTools -lOpenMeshCore resampling.c ./src/BSP.cpp ./src/BSPTraits.cpp ./src/IsotropicRemesher.cpp
 
 
 mex -g -v GCC='/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/clang' -I./ -I/usr/local/include -L/usr/local/lib/OpenMesh -lOpenMeshTools -lOpenMeshCore resampling.c ./src/BSP.cpp ./src/BSPTraits.cpp ./src/IsotropicRemesher.cpp

 
 
 mex -I./ -I/usr/local/include -L/usr/local/lib/OpenMesh -lOpenMeshTools -lOpenMeshCore remeshing.c ./src/BSP.cpp ./src/BSPTraits.cpp ./src/IsotropicRemesher.cpp