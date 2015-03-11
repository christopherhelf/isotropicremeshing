//
//  main.cpp
//  meshtool
//
//  Created by Christopher Helf on 03.03.15.
//  Copyright (c) 2015 test. All rights reserved.
//

#include <cmath>
#include <iostream>

#include "../src/IsotropicRemesher.h"

int main(int argc, const char * argv[]) {
    
    TriMesh mesh;
    
    mesh.request_edge_status();
    mesh.request_halfedge_status();
    mesh.request_face_status();
    mesh.request_vertex_status();
    
    if ( !OpenMesh::IO::read_mesh(mesh, "/test.obj") )
    {
        std::cerr << "Cannot read mesh" << std::endl;
        return -1;
    }
    
    /// Face normals
    mesh.request_face_normals();
    mesh.request_vertex_normals();
    
    if (!mesh.has_face_normals() || !mesh.has_vertex_normals() ) mesh.update_normals();

    int nFacets = 0;
    int* facets = NULL;
    int nPoints = 0;
    double* points = NULL;
    
    IsotropicRemesher::convertMesh(mesh, nFacets, facets, nPoints, points);
    
    int* features = new int[0];
    IsotropicRemesher* remesher = new IsotropicRemesher(nFacets, facets, nPoints, points, 0, features,  0.02, 10);
    
    int nFacetsOut = 0;
    int* facetsOut = NULL;
    int nPointsOut = 0;
    double* pointsOut = NULL;
    
    remesher->remesh(nFacetsOut, facetsOut, nPointsOut, pointsOut);
    
    return 0;
}
