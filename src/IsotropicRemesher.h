//
//  IsotropicRemesher.h
//
//  Created by Christopher Helf on 05.03.15.
//  Copyright (c) 2015. All rights reserved.
//

#ifndef __meshtool__IsotropicRemesher__
#define __meshtool__IsotropicRemesher__

#ifdef __cplusplus

/*#ifndef OM_FORCE_STATIC_CAST
#define OM_FORCE_STATIC_CAST 1
#endif*/

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include "Types.h"
#include "BSP.h"

#include <cmath>

extern float distPointTriangleSquared( const TriMesh::Point& _p,
                                      const TriMesh::Point& _v0,
                                      const TriMesh::Point& _v1,
                                      const TriMesh::Point& _v2,
                                      TriMesh::Point& _nearestPoint );

class IsotropicRemesher {
    
public:
    IsotropicRemesher(int _nfacetsIn, int* _facetsIn, int _nPointsIn, double* _pointsIn, int _nFeaturesIn, int* _featuresIn,  double _edgelength, int _iterations);
    bool remesh();
    static bool convertMesh(TriMesh& mesh, int& nfacetsOut, int*& _facetsOut, int& nPointsOut, double*& _pointsOut);
    float getMeanEdgeLength(TriMesh& mesh);
    static int loadMesh(const char* name, int& nfacetsOut, int*& _facetsOut, int& nPointsOut, double*& _pointsOut);
    
    int getPointsOut();
    int getFacetsOut();
    int* getFacets();
    double* getPoints();
    
private:
    
    bool getMesh(TriMesh& mesh);
    
    void splitLongEdges( TriMesh& _mesh, const double _maxEdgeLength );
    void collapseShortEdges( TriMesh& _mesh, const double _minEdgeLength, const double _maxEdgeLength );
    void equalizeValences( TriMesh& _mesh );
    int targetValence( TriMesh& _mesh, const TriMesh::VertexHandle& _vh );
    bool isBoundary( TriMesh& _mesh, const TriMesh::VertexHandle& _vh );
    bool isFeature( TriMesh& _mesh, const TriMesh::VertexHandle& _vh );
    void tangentialRelaxation( TriMesh& _mesh );
    TriMesh::Point findNearestPoint(const TriMesh&  _mesh,const TriMesh::Point& _point, TriMesh::FaceHandle& _fh, BSP* _ssearch, double* _dbest);
    void projectToSurface( TriMesh& _mesh, TriMesh& _original, BSP* _ssearch);
    bool isFeatureVertex(int v);
    
    PointM getPoint(double* pts, int id) {
        PointM pt;
        pt.x = pts[id+0*nPointsIn];
        pt.y = pts[id+1*nPointsIn];
        pt.z = pts[id+2*nPointsIn];
        return pt;
    }
   
    FacetM getFacet(int* facets, int id) {
        FacetM ft;
        ft.a = facets[id+0*nfacetsIn];
        ft.b = facets[id+1*nfacetsIn];
        ft.c = facets[id+2*nfacetsIn];
        return ft;
    }
    
   
    int nfacetsIn;
    int* facetsIn;
    int nPointsIn;
    double* pointsIn;
    int nFeaturesIn;
    int* featuresIn;
    double edgelength;
    int iterations;
    bool debug;
    
    struct Result {
        int nP;
        int nF;
        double* p;
        int* f;
    };
    
    Result result;
    
};

#else
typedef struct IsotropicRemesher IsotropicRemesher;
#endif

// access functions
#ifdef __cplusplus
#define EXPORT_C extern "C"
#else
#define EXPORT_C
#endif

EXPORT_C IsotropicRemesher* IsotropicRemesher_new(int _nfacetsIn, int* _facetsIn, int _nPointsIn, double* _pointsIn, int _nFeaturesIn, int* _featuresIn,  double _edgelength, int _iterations);
EXPORT_C void IsotropicRemesher_delete(IsotropicRemesher* t);
EXPORT_C int IsotropicRemesher_remesh(IsotropicRemesher* t);
EXPORT_C int IsotropicRemesher_nP(IsotropicRemesher* t);
EXPORT_C int IsotropicRemesher_nF(IsotropicRemesher* t);
EXPORT_C int* IsotropicRemesher_facets(IsotropicRemesher* t);
EXPORT_C double* IsotropicRemesher_points(IsotropicRemesher* t);
EXPORT_C int IsotropicRemesher_load(const char* name, int* nFacets, int* facets, int* nPoints, double* points);

#endif