//
//  BSP.cpp
//
//  Created by Christopher Helf on 07.03.15.
//  Copyright (c) 2015. All rights reserved.
//

#include "BSP.h"

#include <cfloat>
#include <vector>
#include <stdexcept>
#include <limits>

void BSP::_nearest(Node* _node, NearestNeighborData& _data) const
{
    if (!_node->left_child_)
    {
        Scalar dist(0);
        for (HandleIter it=_node->begin(); it!=_node->end(); ++it)
        {
            dist = this->traits_.sqrdist(*it, _data.ref);
            if (dist < _data.dist)
            {
                _data.dist = dist;
                _data.nearest = *it;
            }
        }
    }
    else
    {
        Scalar dist = _node->plane_.distance(_data.ref);
        if (dist > 0.0)
        {
            _nearest(_node->left_child_, _data);
            if (dist*dist < _data.dist)
                _nearest(_node->right_child_, _data);
        }
        else
        {
            _nearest(_node->right_child_, _data);
            if (dist*dist < _data.dist)
                _nearest(_node->left_child_, _data);
        }
    }
}

BSP::NearestNeighbor BSP::nearest(const Point &_p) const {
    NearestNeighborData  data;
    data.ref  = _p;
    data.dist = infinity_;
    if (this->root_ == 0)
        throw std::runtime_error("It seems like the BSP hasn't been built, yet. Did you call build(...)?");
    _nearest(this->root_, data);
    return NearestNeighbor(data.nearest, sqrt(data.dist));
}

void BSP::build(unsigned int _max_handles, unsigned int _max_depth) {
    delete root_;
    root_ = new Node(handles_, 0);
    handles_ = Handles();
    nodes=1;
    traits_.calculateBoundingBoxRoot (root_);
    _build(root_, _max_handles, _max_depth);
}

void BSP::_build(Node*        _node, unsigned int _max_handles, unsigned int _depth) {
    
    if ((_depth == 0) || ((_node->end()-_node->begin()) <= (int)_max_handles))
        return;
    
    Point median;
    int axis;
    traits_.calculateBoundingBox (_node, median, axis);
    
    const Point XYZ[3] = { Point(1,0,0), Point(0,1,0), Point(0,0,1) };
    _node->plane_ = Plane(median, XYZ[axis]);
    
    Handles lhandles, rhandles;
    lhandles.reserve(_node->handles_.size()/2);
    rhandles.reserve(_node->handles_.size()/2);
    
    HandleIter it;
    Point p0, p1, p2;
    for (it=_node->begin(); it!=_node->end(); ++it)
    {
        traits_.points(*it, p0, p1, p2);
        if ( _node->plane_(p0)  || _node->plane_(p1)  || _node->plane_(p2)  ) lhandles.push_back(*it);
        if ( !_node->plane_(p0) || !_node->plane_(p1) || !_node->plane_(p2) ) rhandles.push_back(*it);
        
    }
    
    if (lhandles.size() == _node->handles_.size() ||
        rhandles.size() == _node->handles_.size())
        return;
    else
        _node->handles_ = Handles();
    
    _node->left_child_  = new Node(lhandles, _node);  lhandles = Handles();
    _node->right_child_ = new Node(rhandles, _node);  rhandles = Handles();
    nodes+=2;
    
    _node->right_child_->bb_min  = _node->bb_min;
    _node->right_child_->bb_max  = _node->bb_max;
    _node->right_child_->bb_max[axis] = median [axis];
    
    _node->left_child_->bb_min = _node->bb_min;
    _node->left_child_->bb_min[axis] = median [axis];
    _node->left_child_->bb_max = _node->bb_max;
    
    _build(_node->left_child_,  _max_handles, _depth-1);
    _build(_node->right_child_, _max_handles, _depth-1);
}