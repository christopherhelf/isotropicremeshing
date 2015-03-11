# Isotropic Remesher

This is an Isoptropic Remesher based on OpenMesh (http://www.openmesh.org/) and OpenFlipper (http://www.openflipper.org/). It requires OpenMesh as a dependency.

Make sure to compile OpenMesh with the CMAKE Flag -DCMAKE_BUILD_TYPE=Release, otherwise dynamic casting is used which is not well supported by Matlab and hence can lead to segmentation faults. 

## Features

- Standalone except OpenMesh library
- Capable of handling feature edges, specified by feature vertices
- Easy Setting of target Edgelength
- Possibility of setting the number of iterations

## Installation

With the project folder as Matlab source, you can compile the file directly in Matlab via

$ mex -I./ -I/usr/local/include -L/usr/local/lib/OpenMesh -lOpenMeshTools -lOpenMeshCore remeshing.c ./src/BSP.cpp ./src/BSPTraits.cpp ./src/IsotropicRemesher.cpp

Make sure to point to the correct location of OpenMesh headers and libraries. On MacOSX, you can install OpenMesh via

$ brew install open-mesh --HEAD

which will install the latest version of OpenMesh. If Homebrew doesn't install the Release version, then you can quickly add the

$ -DCMAKE_BUILD_TYPE=Release

flag to the homebrew formula 

$ /usr/local/Library/Formula/open-mesh.rb

in the line 

$ system "cmake", "..", *std_cmake_args, "-DCMAKE_BUILD_TYPE=Release"

This is a quick-and-dirty fix, so it's much better if you compile OpenMesh yourself and set the flags manually. 

## Usage

In Matlab you can call the function via

$ [facetsOut, pointsOut] = remeshing(facets, points, features, targetedgelength, iterations);

facets is supposed to be a nx3 matrix of type int32 (you can simply convert a double matrix via int32(yourmatrix)) containing the IDs of the your points within the points matrix. IDs should, in Matlab notation, start with 1 and end with the size of the points matrix. 

points is a nx3 matrix of type double, with x,y and z values in each column. 

features is a nx1 matrix with the IDs of points considered as features vertices. 

targetedgelength is your desired target edge length and should be a double value, while iterations specifies the number of iterations and should be of type int32. 