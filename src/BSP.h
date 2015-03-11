//
//  BSP.h
//
//  Created by Christopher Helf on 07.03.15.
//  Copyright (c) 2015. All rights reserved.
//

#ifndef meshtool_BSP_h
#define meshtool_BSP_h

#include "BSPTraits.h"

extern float distPointTriangleSquared( const TriMesh::Point& _p,
                                      const TriMesh::Point& _v0,
                                      const TriMesh::Point& _v1,
                                      const TriMesh::Point& _v2,
                                      TriMesh::Point& _nearestPoint );

class BSP
{
public: //---------------------------------------------------------------------
    
    typedef BSPTraits                           Traits;
    typedef BSPTraits::Node            Node;
    typedef Point::value_type          Scalar;
    typedef std::vector<Handle>                 Handles;
    typedef Handles::iterator          HandleIter;
    
    
public: //---------------------------------------------------------------------
    
    BSP(const TriMesh& _mesh, const Scalar _infinity = std::numeric_limits<Scalar>::infinity()) :
    traits_(Traits(_mesh)),
    root_(0),
    nodes(0),
    n_triangles(0),
    infinity_(_infinity)
    {
    }

    ~BSP() {
        delete root_;
    }
    
    struct NearestNeighbor
    {
        NearestNeighbor() {}
        NearestNeighbor(Handle _h, Scalar _d) : handle(_h), dist(_d) {}
        Handle  handle;
        Scalar  dist;
    };
    
    typedef  std::vector< std::pair<Handle,Scalar> > RayCollision;
    NearestNeighbor nearest(const Point& _p) const;

    void reserve(size_t _n) { handles_.reserve(_n); }
    void push_back(Handle _h)     { handles_.push_back(_h); ++n_triangles; }
    bool empty()     { return n_triangles == 0; }
    size_t size()     { return n_triangles; }
    void build(unsigned int _max_handles, unsigned int _max_depth);
    
private:

    BSP(const BSP &rhs);
    BSP &operator=(const BSP &rhs);
    
    
private:
    
    void _build(Node*        _node,
                unsigned int _max_handles,
                unsigned int _depth);
    
    
    struct NearestNeighborData
    {
        Point   ref;
        Scalar  dist;
        Handle  nearest;
    };
    
    void _nearest(Node* _node, NearestNeighborData& _data) const;
    
    template<typename T,typename U>
    struct less_pair_second: public std::binary_function<T,U,bool> {
        bool operator()(const std::pair<T,U> &left, const std::pair<T,U> &right) {
            return (fabs(left.second) < fabs(right.second));
        }
    };
    
protected: //-------------------------------------------------------------------
    
    
    BSPTraits  traits_;
    Handles    handles_;
    Node*      root_;
    int	       nodes, n_triangles;

    
    const Scalar infinity_;
    
};


#endif
