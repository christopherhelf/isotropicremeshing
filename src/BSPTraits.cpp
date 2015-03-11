//
//  BSPTraits.cpp
//
//  Created by Christopher Helf on 07.03.15.
//  Copyright (c) 2015. All rights reserved.
//

#include "BSPTraits.h"
#include "Functions.h"

void BSPTraits::points(const Handle _h, Point& _p0, Point& _p1, Point& _p2) const
{
    TriMesh::CFVIter fv_it(mesh_.cfv_iter(_h));
    _p0 = mesh_.point(*fv_it);
    ++fv_it;
    _p1 = mesh_.point(*fv_it);
    ++fv_it;
    _p2 = mesh_.point(*fv_it);
}

Scalar BSPTraits::sqrdist(const Handle _h, const Point& _p) const
{
    Point p0, p1, p2, q;
    points(_h, p0, p1, p2);
    return distPointTriangleSquared(_p, p0, p1, p2, q);
}

void BSPTraits::calculateBoundingBox(Node* _node, Point& median, int& axis)
{
    HandleIter it_h;
    Point p0, p1, p2, bb_min, bb_max;
    bb_min.vectorize(std::numeric_limits<Point::value_type>::infinity());
    bb_max.vectorize(-std::numeric_limits<Point::value_type>::infinity());
    std::list<Point> vertices;
    
    for (it_h = _node->begin(); it_h != _node->end(); ++it_h) {
        points(*it_h, p0, p1, p2);
        
        vertices.push_back(p0);
        vertices.push_back(p1);
        vertices.push_back(p2);
    }
    bb_min = _node->bb_min;
    bb_max = _node->bb_max;
    
    Point bb = bb_max - bb_min;
    Scalar length = bb[0];
    axis = 0;
    if (bb[1] > length)
        length = bb[(axis = 1)];
    if (bb[2] > length)
        length = bb[(axis = 2)];
    
    switch (axis) {
        case 0:
            vertices.sort(x_sort());
            break;
        case 1:
            vertices.sort(y_sort());
            break;
        case 2:
            vertices.sort(z_sort());
            break;
    }
    vertices.unique();
    
    size_t size = vertices.size();
    std::list<Point>::iterator it_v;
    it_v = vertices.begin();
    std::advance(it_v, size / 2);
    median = *it_v;
    
}

void BSPTraits::calculateBoundingBoxRoot(Node* _node)
{
    HandleIter it;
    Point p0, p1, p2, bb_min, bb_max;
    bb_min.vectorize(FLT_MAX);
    bb_max.vectorize(-FLT_MAX);
    for (it = _node->begin(); it != _node->end(); ++it) {
        points(*it, p0, p1, p2);
        bb_min.minimize(p0);
        bb_min.minimize(p1);
        bb_min.minimize(p2);
        bb_max.maximize(p0);
        bb_max.maximize(p1);
        bb_max.maximize(p2);
    }
    _node->bb_min = bb_min;
    _node->bb_max = bb_max;
}