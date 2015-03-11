//
//  Functions.h
//
//  Created by Christopher Helf on 06.03.15.
//  Copyright (c) 2015. All rights reserved.
//

#ifndef meshtool_Functions_h
#define meshtool_Functions_h

#include "Types.h"
#include "mex.h"

float distPointTriangleSquared( const TriMesh::Point& _p,
                               const TriMesh::Point& _v0,
                               const TriMesh::Point& _v1,
                               const TriMesh::Point& _v2,
                               TriMesh::Point& _nearestPoint )
{
    TriMesh::Point v0v1 = _v1 - _v0;
    TriMesh::Point v0v2 = _v2 - _v0;
    TriMesh::Point n = v0v1 % v0v2;
    float d = n.sqrnorm();
    
    if (d < FLT_MIN && d > -FLT_MIN) {
        mexErrMsgTxt("distPointTriangleSquared: Degenerated triangle !\n");
        return -1.0;
    }
    
    float invD = float(1.0) / d;
    
    TriMesh::Point v1v2 = _v2 - _v1;
    float inv_v0v2_2 = float(1.0) / v0v2.sqrnorm();
    float inv_v0v1_2 = float(1.0) / v0v1.sqrnorm();
    float inv_v1v2_2 = float(1.0) / v1v2.sqrnorm();
    
    
    TriMesh::Point v0p = _p - _v0;
    TriMesh::Point t = v0p % n;
    float  s01, s02, s12;
    float a = (t | v0v2) * -invD;
    float b = (t | v0v1) * invD;
    
    
    if (a < 0)
    {
        s02 = ( v0v2 | v0p ) * inv_v0v2_2;
        if (s02 < 0.0)
        {
            s01 = ( v0v1 | v0p ) * inv_v0v1_2;
            if (s01 <= 0.0) {
                v0p = _v0;
            } else if (s01 >= 1.0) {
                v0p = _v1;
            } else {
                v0p = _v0 + v0v1 * s01;
            }
        } else if (s02 > 1.0) {
            s12 = ( v1v2 | ( _p - _v1 )) * inv_v1v2_2;
            if (s12 >= 1.0) {
                v0p = _v2;
            } else if (s12 <= 0.0) {
                v0p = _v1;
            } else {
                v0p = _v1 + v1v2 * s12;
            }
        } else {
            v0p = _v0 + v0v2 * s02;
        }
    } else if (b < 0.0) {
       
        s01 = ( v0v1 | v0p ) * inv_v0v1_2;
        if (s01 < 0.0)
        {
            s02 = ( v0v2 |  v0p ) * inv_v0v2_2;
            if (s02 <= 0.0) {
                v0p = _v0;
            } else if (s02 >= 1.0) {
                v0p = _v2;
            } else {
                v0p = _v0 + v0v2 * s02;
            }
        } else if (s01 > 1.0) {
            s12 = ( v1v2 | ( _p - _v1 )) * inv_v1v2_2;
            if (s12 >= 1.0) {
                v0p = _v2;
            } else if (s12 <= 0.0) {
                v0p = _v1;
            } else {
                v0p = _v1 + v1v2 * s12;
            }
        } else {
            v0p = _v0 + v0v1 * s01;
        }
    } else if (a+b > 1.0) {
        
        s12 = ( v1v2 | ( _p - _v1 )) * inv_v1v2_2;
        if (s12 >= 1.0) {
            s02 = ( v0v2 | v0p ) * inv_v0v2_2;
            if (s02 <= 0.0) {
                v0p = _v0;
            } else if (s02 >= 1.0) {
                v0p = _v2;
            } else {
                v0p = _v0 + v0v2*s02;
            }
        } else if (s12 <= 0.0) {
            s01 = ( v0v1 |  v0p ) * inv_v0v1_2;
            if (s01 <= 0.0) {
                v0p = _v0;
            } else if (s01 >= 1.0) {
                v0p = _v1;
            } else {
                v0p = _v0 + v0v1 * s01;
            }
        } else {
            v0p = _v1 + v1v2 * s12;
        }
    } else {
        
        _nearestPoint = _p - n*((n|v0p) * invD);
        return (_nearestPoint - _p).sqrnorm();
    }
    
    _nearestPoint = v0p;
    
    return (_nearestPoint - _p).sqrnorm();
}

#endif
