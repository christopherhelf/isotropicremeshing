//
//  BSPTraits.h
//
//  Created by Christopher Helf on 07.03.15.
//  Copyright (c) 2015. All rights reserved.
//

#ifndef meshtool_BSPTraits_h
#define meshtool_BSPTraits_h

#include "Types.h"

#include "BSPTreeNode.h"
#include <list>

class BSPTraits
{
public:
    
    typedef BSPTreeNode                       Node;
    
    BSPTraits(const TriMesh& _mesh) : mesh_(_mesh) {}
    
    void points(const Handle _h, Point& _p0, Point& _p1, Point& _p2) const;
    Scalar sqrdist(const Handle _h, const Point& _p) const;
    void calculateBoundingBox(Node* _node, Point& median, int& axis);
    void calculateBoundingBoxRoot(Node* _node);
    
private:
    
    const TriMesh& mesh_;
    struct x_sort { bool operator()(const Point& first, const Point& second) { return (first[0] < second[0]); }  };
    struct y_sort { bool operator()(const Point& first, const Point& second) { return (first[1] < second[1]); }  };
    struct z_sort { bool operator()(const Point& first, const Point& second) { return (first[2] < second[2]); }  };
};

#endif
