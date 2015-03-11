//
//  IsotropicRemesher.cpp
//
//  Created by Christopher Helf on 05.03.15.
//  Copyright (c) 2015. All rights reserved.
//

#include "IsotropicRemesher.h"
#include "mex.h"

IsotropicRemesher::IsotropicRemesher(int _nfacetsIn, int* _facetsIn, int _nPointsIn, double* _pointsIn, int _nFeaturesIn, int* _featuresIn,  double _edgelength, int _iterations)
    :   nfacetsIn(_nfacetsIn),
        facetsIn(_facetsIn),
        nPointsIn(_nPointsIn),
        pointsIn(_pointsIn),
        nFeaturesIn(_nFeaturesIn),
        featuresIn(_featuresIn),
        edgelength(_edgelength),
        iterations(_iterations),
        debug(false)

{

}

int IsotropicRemesher::getPointsOut() {
    return result.nP;
}

int IsotropicRemesher::getFacetsOut() {
    return result.nF;
}

int* IsotropicRemesher::getFacets() {
    return result.f;
}

double* IsotropicRemesher::getPoints() {
    return result.p;
}


bool IsotropicRemesher::remesh() {
   
    int nfacetsOut = 0;
    int* facetsOut = NULL;
    int nPointsOut = 0;
    double* pointsOut = NULL;
    
    TriMesh mesh;
    
    if(!getMesh(mesh)) {
        return false;
    }

    /*boundaries*/
    const double low  = (4.0 / 5.0) * this->edgelength;
    const double high = (4.0 / 3.0) * this->edgelength;
    
    TriMesh meshCopy = mesh;
    
    float eL = getMeanEdgeLength(mesh);
    
    if(debug) mexPrintf("Mean Edge Length before Operation: %f\n", eL);
    
    /*create Triangle BSP*/
    BSP* triangle_bsp = new BSP( meshCopy );
    triangle_bsp->reserve(meshCopy.n_faces());
    TriMesh::FIter f_it  = meshCopy.faces_begin();
    TriMesh::FIter f_end = meshCopy.faces_end();
    for (; f_it!=f_end; ++f_it) {
        triangle_bsp->push_back( *f_it );
    }
    triangle_bsp->build(10, 100);
    
    /*do the iterations*/
    for (int i=0; i < iterations; i++){
        splitLongEdges(mesh, high);
        collapseShortEdges(mesh, low, high);
        equalizeValences(mesh);
        tangentialRelaxation(mesh);
        projectToSurface(mesh, meshCopy, triangle_bsp);
    }
    
    
    eL = getMeanEdgeLength(mesh);
    
    if(debug) mexPrintf("Mean Edge Length After Operation: %f\n", eL); 
    
    /*convert back into the pointers*/
    if(!convertMesh(mesh, nfacetsOut, facetsOut, nPointsOut, pointsOut)) {
        return false;
    };
    
    result.nF = nfacetsOut;
    result.nP = nPointsOut;
    result.p = pointsOut;
    result.f = facetsOut;
    
    return true;
}

int IsotropicRemesher::loadMesh(const char* name, int& nfacetsOut, int*& _facetsOut, int& nPointsOut, double*& _pointsOut) {
    
    TriMesh mesh;
    mesh.request_edge_status();
    mesh.request_halfedge_status();
    mesh.request_face_status();
    mesh.request_vertex_status();
    
    if ( !OpenMesh::IO::read_mesh(mesh, name) )
    {
        mexPrintf("Cannot read mesh\n"); 
        return -1;
    }

    mesh.request_face_normals();
    mesh.request_vertex_normals();
    if (!mesh.has_face_normals() || !mesh.has_vertex_normals() ) mesh.update_normals();
 
    return IsotropicRemesher::convertMesh(mesh, nfacetsOut, _facetsOut, nPointsOut, _pointsOut);
}

bool IsotropicRemesher::convertMesh(TriMesh& mesh, int& nfacetsOut, int*& facetsOut, int& nPointsOut, double*& pointsOut) {
    
    /*count vertices and add id property*/
    OpenMesh::VPropHandleT<int> id;
    mesh.add_property(id);
    nPointsOut = 0;
    TriMesh::VertexIter v_it, v_end(mesh.vertices_end());
    for (v_it=mesh.vertices_begin(); v_it!=v_end; ++v_it) {
        mesh.property(id,*v_it) = nPointsOut;
        nPointsOut++;
    }
    
    /*count facets*/
    nfacetsOut = 0;
    TriMesh::FaceIter vF_it, vF_end(mesh.faces_end());
    for (vF_it=mesh.faces_begin(); vF_it!=vF_end; ++vF_it)
        nfacetsOut++;
    
    int nF = nfacetsOut*3;
    int nP = nPointsOut*3;
    
    /*allocate memory*/
    facetsOut = new int[nF];
    pointsOut = new double[nP];
    
    /*fill memory*/
    double x = 0.;
    double y = 0.;
    double z = 0.;
    int a = 0;
    int b = 0;
    int c = 0;
    int cnt = 0;
    
    /*fill vertices*/
    for (v_it=mesh.vertices_begin(); v_it!=v_end; ++v_it) {
       
        TriMesh::Point pt = mesh.point( *v_it );
        x= pt.values_[0];
        y= pt.values_[1];
        z= pt.values_[2];
                        
        pointsOut[cnt+0*nPointsOut] = x;
        pointsOut[cnt+1*nPointsOut] = y;
        pointsOut[cnt+2*nPointsOut] = z;
        
        cnt++;
    }
    
    cnt = 0;
    
    /*fill facets*/
    for (vF_it=mesh.faces_begin(); vF_it!=vF_end; ++vF_it) {
        
        TriMesh::ConstFaceVertexIter cfv_it = mesh.cfv_iter(*vF_it);
        
        a = mesh.property(id,*cfv_it);
        b = mesh.property(id,*(++cfv_it));
        c = mesh.property(id,*(++cfv_it));
        
        facetsOut[cnt+0*nfacetsOut] = a;
        facetsOut[cnt+1*nfacetsOut] = b;
        facetsOut[cnt+2*nfacetsOut] = c;
        
        cnt++;
        
    }
    
    mesh.remove_property(id);
    mesh.garbage_collection();
    
    return true;
    
}

float IsotropicRemesher::getMeanEdgeLength(TriMesh& mesh) {
    
    TriMesh::EdgeIter e_it;
    TriMesh::EdgeIter e_end = mesh.edges_end();
    
    float sum = 0.0;
    int num = 0;
    
    //iterate over all edges
    for (e_it = mesh.edges_begin(); e_it != e_end; ++e_it){
        
        const TriMesh::HalfedgeHandle & hh = mesh.halfedge_handle( *e_it, 0 );
        
        const TriMesh::VertexHandle & v0 = mesh.from_vertex_handle(hh);
        const TriMesh::VertexHandle & v1 = mesh.to_vertex_handle(hh);
        
        const TriMesh::Point vec = mesh.point(v1) - mesh.point(v0);
        
        float l = sqrt(vec.sqrnorm());
        
        num++;
        sum=sum+l;
        
    }
    
    return sum/(float)num;
    
}


bool IsotropicRemesher::getMesh(TriMesh& mesh) {
    
    OpenMesh::VPropHandleT<int> id;
    mesh.add_property(id);
    
    std::vector<TriMesh::VertexHandle> vHandles;
        
     for(int i=0; i<nPointsIn; i++) {
         PointM pt = getPoint(this->pointsIn, i);
         TriMesh::VertexHandle vhandle = mesh.add_vertex(TriMesh::Point(pt.x, pt.y,  pt.z));
         mesh.property(id,vhandle) = i;
         vHandles.push_back(vhandle);
     }
    
    std::vector<TriMesh::VertexHandle>  face_vhandles;
    
    int nF = this->nfacetsIn;
    
    for(int i=0; i<nF; i++) {
        FacetM ft = getFacet(this->facetsIn, i);
        face_vhandles.clear();
        face_vhandles.push_back(vHandles[ft.a]);
        face_vhandles.push_back(vHandles[ft.b]);
        face_vhandles.push_back(vHandles[ft.c]);
        mesh.add_face(face_vhandles);
    }
    
    
    
    mesh.request_edge_status();
    mesh.request_halfedge_status();
    mesh.request_face_status();
    mesh.request_vertex_status();

    mesh.request_face_normals();
    mesh.request_vertex_normals();
    
    if (!mesh.has_face_normals() || !mesh.has_vertex_normals() ) mesh.update_normals();

    // Features
    if(nFeaturesIn > 0) {
        
        TriMesh::EdgeIter e_it;
        TriMesh::EdgeIter e_end = mesh.edges_end();
        
        for (e_it = mesh.edges_sbegin(); e_it != e_end; ++e_it) {
            
            const TriMesh::HalfedgeHandle & hh = mesh.halfedge_handle( *e_it, 0 );
            const TriMesh::VertexHandle & v0 = mesh.from_vertex_handle(hh);
            const TriMesh::VertexHandle & v1 = mesh.to_vertex_handle(hh);
            
            int a = mesh.property(id,v0);
            int b = mesh.property(id,v1);
            
            if(isFeatureVertex(a) && isFeatureVertex(b)) {
                mesh.status( *e_it ).set_feature( true );
            }
            
        }
        
    }

    mesh.remove_property(id);
    mesh.garbage_collection();
    
    return true;
}

bool IsotropicRemesher::isFeatureVertex(int v) {
    
    for(int i=0; i<this->nFeaturesIn; i++) {
        if(this->featuresIn[i] == v) return true;
    }
    return false;
    
}

void IsotropicRemesher::splitLongEdges( TriMesh& _mesh, const double _maxEdgeLength )
{
    
    const double _maxEdgeLengthSqr = _maxEdgeLength * _maxEdgeLength;
    
    TriMesh::EdgeIter e_it;
    TriMesh::EdgeIter e_end = _mesh.edges_end();

    for (e_it = _mesh.edges_begin(); e_it != e_end; ++e_it){
        
        const TriMesh::HalfedgeHandle & hh = _mesh.halfedge_handle( *e_it, 0 );
        
        const TriMesh::VertexHandle & v0 = _mesh.from_vertex_handle(hh);
        const TriMesh::VertexHandle & v1 = _mesh.to_vertex_handle(hh);
        
        const TriMesh::Point vec = _mesh.point(v1) - _mesh.point(v0);
        
        //edge to long?
        if ( vec.sqrnorm() > _maxEdgeLengthSqr ){
            
            TriMesh::Point midPoint = _mesh.point(v0) + ( 0.5 * vec );
            
            //split at midpoint
            TriMesh::VertexHandle vh = _mesh.add_vertex( midPoint );
            
            bool hadFeature = _mesh.status(*e_it).feature();
            
            _mesh.split(*e_it, vh);
            
            if ( hadFeature ){
                TriMesh::VOHIter vh_it;
                for (vh_it = _mesh.voh_iter(vh); vh_it.is_valid(); ++vh_it)
                    if ( _mesh.to_vertex_handle(*vh_it) == v0 || _mesh.to_vertex_handle(*vh_it) == v1 )
                        _mesh.status( _mesh.edge_handle( *vh_it ) ).set_feature( true );
            }
        }
    }
}


void IsotropicRemesher::collapseShortEdges( TriMesh& _mesh, const double _minEdgeLength, const double _maxEdgeLength )
{
    const double _minEdgeLengthSqr = _minEdgeLength * _minEdgeLength;
    const double _maxEdgeLengthSqr = _maxEdgeLength * _maxEdgeLength;
    
    //add checked property
    OpenMesh::EPropHandleT< bool > checked;
    if ( !_mesh.get_property_handle(checked, "Checked Property") )
        _mesh.add_property(checked,"Checked Property" );
    
    //init property
    TriMesh::ConstEdgeIter e_it;
    TriMesh::ConstEdgeIter e_end = _mesh.edges_end();
    
    for (e_it = _mesh.edges_begin(); e_it != e_end; ++e_it)
        _mesh.property(checked, *e_it) = false;
    
    
    bool finished = false;
    
    while( !finished ){
        
        finished = true;
        
        for (e_it = _mesh.edges_begin(); e_it != _mesh.edges_end() ; ++e_it){
            
            if ( _mesh.property(checked, *e_it) )
                continue;
            
            _mesh.property(checked, *e_it) = true;
            
            const TriMesh::HalfedgeHandle & hh = _mesh.halfedge_handle( *e_it, 0 );
            
            const TriMesh::VertexHandle & v0 = _mesh.from_vertex_handle(hh);
            const TriMesh::VertexHandle & v1 = _mesh.to_vertex_handle(hh);
            
            const TriMesh::Point vec = _mesh.point(v1) - _mesh.point(v0);
            
            const double edgeLength = vec.sqrnorm();
            
            // edge too short but don't try to collapse edges that have length 0
            if ( (edgeLength < _minEdgeLengthSqr) && (edgeLength > DBL_EPSILON) ){
                
                //check if the collapse is ok
                const TriMesh::Point & B = _mesh.point(v1);
                
                bool collapse_ok = true;
                
                for(TriMesh::VOHIter vh_it = _mesh.voh_iter(v0); vh_it.is_valid(); ++vh_it)
                    if ( (( B - _mesh.point( _mesh.to_vertex_handle(*vh_it) ) ).sqrnorm() > _maxEdgeLengthSqr )
                        || ( _mesh.status( _mesh.edge_handle( *vh_it ) ).feature())
                        || ( _mesh.is_boundary( _mesh.edge_handle( *vh_it ) ) )){
                        collapse_ok = false;
                        break;
                    }
                
                if( collapse_ok && _mesh.is_collapse_ok(hh) ) {
                    _mesh.collapse( hh );
                    
                    finished = false;
                }
                
            }
        }
        
    }
    
    _mesh.remove_property(checked);
    _mesh.garbage_collection();
}

void IsotropicRemesher::equalizeValences( TriMesh& _mesh )
{
    
    TriMesh::EdgeIter e_it;
    TriMesh::EdgeIter e_end = _mesh.edges_end();
    
    for (e_it = _mesh.edges_sbegin(); e_it != e_end; ++e_it){
        
        if ( !_mesh.is_flip_ok(*e_it) ) continue;
        if ( _mesh.status( *e_it ).feature() ) continue;
        
        
        const TriMesh::HalfedgeHandle & h0 = _mesh.halfedge_handle( *e_it, 0 );
        const TriMesh::HalfedgeHandle & h1 = _mesh.halfedge_handle( *e_it, 1 );
        
        if (h0.is_valid() && h1.is_valid())
            
            if (_mesh.face_handle(h0).is_valid() && _mesh.face_handle(h1).is_valid()){
                //get vertices of corresponding faces
                const TriMesh::VertexHandle & a = _mesh.to_vertex_handle(h0);
                const TriMesh::VertexHandle & b = _mesh.to_vertex_handle(h1);
                const TriMesh::VertexHandle & c = _mesh.to_vertex_handle(_mesh.next_halfedge_handle(h0));
                const TriMesh::VertexHandle & d = _mesh.to_vertex_handle(_mesh.next_halfedge_handle(h1));
                
                const int deviation_pre =  abs((int)(_mesh.valence(a) - targetValence(_mesh, a)))
                +abs((int)(_mesh.valence(b) - targetValence(_mesh, b)))
                +abs((int)(_mesh.valence(c) - targetValence(_mesh, c)))
                +abs((int)(_mesh.valence(d) - targetValence(_mesh, d)));
                _mesh.flip(*e_it);
                
                const int deviation_post = abs((int)(_mesh.valence(a) - targetValence(_mesh, a)))
                +abs((int)(_mesh.valence(b) - targetValence(_mesh, b)))
                +abs((int)(_mesh.valence(c) - targetValence(_mesh, c)))
                +abs((int)(_mesh.valence(d) - targetValence(_mesh, d)));
                
                if (deviation_pre <= deviation_post)
                    _mesh.flip(*e_it);
            }
    }
}

int IsotropicRemesher::targetValence( TriMesh& _mesh, const TriMesh::VertexHandle& _vh ){
    
    if (isBoundary(_mesh,_vh))
        return 4;
    else
        return 6;
}

bool IsotropicRemesher::isBoundary( TriMesh& _mesh, const TriMesh::VertexHandle& _vh ){
    
    TriMesh::ConstVertexOHalfedgeIter voh_it;
    
    for (voh_it = _mesh.voh_iter( _vh ); voh_it.is_valid(); ++voh_it )
        if ( _mesh.is_boundary( _mesh.edge_handle( *voh_it ) ) )
            return true;
    
    return false;
}

bool IsotropicRemesher::isFeature( TriMesh& _mesh, const TriMesh::VertexHandle& _vh ){
    
    TriMesh::ConstVertexOHalfedgeIter voh_it;
    
    for (voh_it = _mesh.voh_iter( _vh ); voh_it.is_valid(); ++voh_it )
        if ( _mesh.status( _mesh.edge_handle( *voh_it ) ).feature() )
            return true;
    
    return false;
}

void IsotropicRemesher::tangentialRelaxation( TriMesh& _mesh )
{
    _mesh.update_normals();
    
    //add checked property
    OpenMesh::VPropHandleT< TriMesh::Point > q;
    if ( !_mesh.get_property_handle(q, "q Property") )
        _mesh.add_property(q,"q Property" );
    
    TriMesh::ConstVertexIter v_it;
    TriMesh::ConstVertexIter v_end = _mesh.vertices_end();
    
    //first compute barycenters
    for (v_it = _mesh.vertices_sbegin(); v_it != v_end; ++v_it){
        
        TriMesh::Point tmp(0.0, 0.0, 0.0);
        uint N = 0;
        
        TriMesh::VVIter vv_it;
        for (vv_it = _mesh.vv_iter(*v_it); vv_it.is_valid(); ++vv_it){
            tmp += _mesh.point(*vv_it);
            N++;
        }
        
        if (N > 0)
            tmp /= (double) N;
        
        _mesh.property(q, *v_it) = tmp;
    }
    
    //move to new position
    for (v_it = _mesh.vertices_sbegin(); v_it != v_end; ++v_it){
        if ( !isBoundary(_mesh, *v_it) && !isFeature(_mesh, *v_it) )
            _mesh.set_point(*v_it,  _mesh.property(q, *v_it) + (_mesh.normal(*v_it)| (_mesh.point(*v_it) - _mesh.property(q, *v_it) ) ) * _mesh.normal(*v_it));
    }
    
    _mesh.remove_property(q);
}

Point IsotropicRemesher::findNearestPoint(const TriMesh &_mesh, const TriMesh::Point &_point, TriMesh::FaceHandle &_fh, BSP *_ssearch, double *_dbest) {
    
    Point  p_best = _mesh.point(_mesh.vertex_handle(0));
    Scalar d_best = (_point-p_best).sqrnorm();
    
    TriMesh::FaceHandle fh_best;
    
    if( _ssearch == 0 )
    {
        // exhaustive search
        TriMesh::ConstFaceIter cf_it  = _mesh.faces_begin();
        TriMesh::ConstFaceIter cf_end = _mesh.faces_end();
        
        for(; cf_it != cf_end; ++cf_it)
        {
            TriMesh::ConstFaceVertexIter cfv_it = _mesh.cfv_iter(*cf_it);
            
            const Point& pt0 = _mesh.point(   *cfv_it);
            const Point& pt1 = _mesh.point( *(++cfv_it));
            const Point& pt2 = _mesh.point( *(++cfv_it));
            
            Point ptn;
            
            Scalar d = distPointTriangleSquared( _point,pt0,pt1,pt2,ptn );
            
            if( d < d_best)
            {
                d_best = d;
                p_best = ptn;
                
                fh_best = *cf_it;
            }
        }
        
        // return facehandle
        _fh = fh_best;
        
        // return distance
        if(_dbest)
            *_dbest = sqrt(d_best);
        
        
        return p_best;
    }
    else
    {
        TriMesh::FaceHandle     fh = _ssearch->nearest(_point).handle;
        TriMesh::CFVIter        fv_it = _mesh.cfv_iter(fh);
        
        const Point&   pt0 = _mesh.point( *(  fv_it));
        const Point&   pt1 = _mesh.point( *(++fv_it));
        const Point&   pt2 = _mesh.point( *(++fv_it));
        
        // project
        d_best = distPointTriangleSquared(_point, pt0, pt1, pt2, p_best);
        
        // return facehandle
        _fh = fh;
        
        // return distance
        if(_dbest)
            *_dbest = sqrt(d_best);
        
        return p_best;
    }
}

void IsotropicRemesher::projectToSurface(TriMesh &_mesh, TriMesh &_original, BSP *_ssearch) {
    
    TriMesh::VertexIter v_it;
    TriMesh::VertexIter v_end = _mesh.vertices_end();
    
    for (v_it = _mesh.vertices_begin(); v_it != v_end; ++v_it){
        
        if (isBoundary(_mesh, *v_it)) continue;
        if ( isFeature(_mesh, *v_it)) continue;
        
        Point p = _mesh.point(*v_it);
        TriMesh::FaceHandle fhNear;
        double distance;
        
        Point pNear = findNearestPoint(_original, p, fhNear, _ssearch, &distance);
        
        _mesh.set_point(*v_it, pNear);
    }
}

// access functions
EXPORT_C IsotropicRemesher* IsotropicRemesher_new(int _nfacetsIn, int* _facetsIn, int _nPointsIn, double* _pointsIn, int _nFeaturesIn, int* _featuresIn,  double _edgelength, int _iterations)
{
    return new IsotropicRemesher(_nfacetsIn, _facetsIn, _nPointsIn, _pointsIn, _nFeaturesIn, _featuresIn, _edgelength, _iterations);
}

EXPORT_C void IsotropicRemesher_delete(IsotropicRemesher* t)
{
    delete t;
}

EXPORT_C int IsotropicRemesher_remesh(IsotropicRemesher* t)
{
    if(t->remesh()) return 1;
    else return 0;
}

EXPORT_C int IsotropicRemesher_load(const char* name, int* nFacets, int* facets, int* nPoints, double* points) {
    return IsotropicRemesher::loadMesh(name, *nFacets, facets, *nPoints, points);
}

EXPORT_C int IsotropicRemesher_nP(IsotropicRemesher* t) {
    return t->getPointsOut();
}

EXPORT_C int IsotropicRemesher_nF(IsotropicRemesher* t) {
    return t->getFacetsOut();
}

EXPORT_C int* IsotropicRemesher_facets(IsotropicRemesher* t) {
    return t->getFacets();
}
EXPORT_C double* IsotropicRemesher_points(IsotropicRemesher* t) {
    return t->getPoints();
}
