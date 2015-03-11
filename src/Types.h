//
//  Types.h
//
//  Created by Christopher Helf on 06.03.15.
//  Copyright (c) 2015. All rights reserved.
//

#ifndef meshtool_Types_h
#define meshtool_Types_h

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <cmath>

struct TriTraits : public OpenMesh::DefaultTraits {
    typedef OpenMesh::Vec3d Point;
    typedef OpenMesh::Vec3d Normal;
    typedef OpenMesh::Vec4f Color;
};

typedef OpenMesh::TriMesh_ArrayKernelT<TriTraits>   TriMesh;
typedef TriMesh::Scalar                             Scalar;
typedef TriMesh::Point                              Point;
typedef TriMesh::FaceHandle                         Handle;
typedef std::vector<Handle>                         Handles;
typedef Handles::iterator                           HandleIter;
typedef TriMesh::VertexHandle                       VertexHandle;


struct PointM {
    double x;
    double y;
    double z;
};

struct FacetM {
    int a;
    int b;
    int c;
};

class Plane
{
public:
    typedef Point  Vec3;
    
    Plane( Scalar _a=0, Scalar _b=0, Scalar _c=0, Scalar _d=0 )
    : coeffsA(_a), coeffsB(_b), coeffsC(_c), coeffsD(_d)
    { HNF(); }
    
    Plane( const Vec3& _o, const Vec3& _n )
    : coeffsA(_n[0]), coeffsB(_n[1]), coeffsC(_n[2]), coeffsD(-(_n|_o))
    { HNF(); }
    
    Plane( const Vec3& _v0, const Vec3& _v1, const Vec3& _v2 )
    {
        Vec3 n = (_v1-_v0) % (_v2-_v0);
        coeffsA = n[0];
        coeffsB = n[1];
        coeffsC = n[2];
        coeffsD = -(n|_v0);
        HNF();
    }
 
    Vec3 normal() const { return Vec3(coeffsA, coeffsB, coeffsC); }
    
    Scalar distance( const Vec3& _v ) const
    {
        return ( _v[0]*coeffsA +
                _v[1]*coeffsB +
                _v[2]*coeffsC +
                coeffsD );
    }
    
    bool operator() ( const Vec3& _v ) const { return distance(_v) > 0; }
     
private:
    
    void HNF() {
        Scalar n = normal().norm();
        if (n != 0.0) {
            coeffsA /= n;
            coeffsB /= n;
            coeffsC /= n;
            coeffsD /= n;
        }
    }
    
    Scalar coeffsA;
    Scalar coeffsB;
    Scalar coeffsC;
    Scalar coeffsD;
};

#endif
