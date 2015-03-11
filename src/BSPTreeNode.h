//
//  BSPTreeNode.h
//
//  Created by Christopher Helf on 07.03.15.
//  Copyright (c) 2015. All rights reserved.
//

#ifndef meshtool_BSPTreeNode_h
#define meshtool_BSPTreeNode_h

#include "Types.h"

struct BSPTreeNode
{
    
    BSPTreeNode(const Handles& _handles, BSPTreeNode* _parent)
    : handles_(_handles),
    parent_(_parent), left_child_(0), right_child_(0) {}
    ~BSPTreeNode()
    {
        delete left_child_;
        delete right_child_;
        
        if (parent_)
        {
            if (this == parent_->left_child_)
                parent_->left_child_ = 0;
            else
                parent_->right_child_ = 0;
        }
    }
    
    HandleIter begin() {
        return handles_.begin();
    }
    HandleIter end()   {
        return handles_.end();
    }
    
    size_t size() const {
        return handles_.size();
    }
    
    Handles     handles_;
    BSPTreeNode    *parent_, *left_child_, *right_child_;
    Point	bb_min, bb_max;
    Plane       plane_;
    
private:

    BSPTreeNode(const BSPTreeNode &rhs);
    BSPTreeNode &operator=(const BSPTreeNode &rhs);
    
};


#endif
