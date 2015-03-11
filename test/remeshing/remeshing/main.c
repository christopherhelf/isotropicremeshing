//
//  main.c
//  remeshing
//
//  Created by Christopher Helf on 11.03.15.
//  Copyright (c) 2015 test. All rights reserved.
//

#include <stdint.h>

/*Quick Clang Fix*/
#ifndef char16_t
typedef uint16_t char16_t;
#endif

/*Include matlab functions*/
#include <mex.h>
#include <matrix.h>

/*forward declaration of function in resampling.c*/
void loadPoints(const char* name, int* nFacets, int** facets, int* nPoints, double** points);

int main(int argc, const char * argv[]) {
    
    /*input and output numbers*/
    int nrhs = 5;
    int nlhs = 2;
    
    /*create the arrays*/
    mxArray* plhs[nlhs];
    const mxArray* prhs[nrhs];
    
    /*read obj file*/
    double* points = NULL;
    int* facets = NULL;
    int npts = 0;
    int nfts = 0;
    
    /*load the points*/
    loadPoints("/test.obj", &nfts, &facets, &npts, &points);
    
    /*test parameters*/
    double edgeLength = 0.2;
    int iterations = 10;
    
    /*create the inputs*/
    const mwSize three = 3;
    const mwSize nPoints = npts;
    const mwSize nFacets = nfts;
    
    /*inputs*/
    prhs[0] = mxCreateNumericArray(nFacets, &three, mxINT32_CLASS, mxREAL);
    prhs[1] = mxCreateDoubleMatrix(nPoints, three, mxREAL);
    prhs[3] = mxCreateDoubleScalar(edgeLength);
    prhs[4] = mxCreateDoubleScalar(iterations);
    
    /*fill inputs*/
    memcpy(mxGetPr(prhs[0]), facets, (size_t) nFacets*3*sizeof(int));
    memcpy(mxGetPr(prhs[1]), points, (size_t) nPoints*3*sizeof(double));
    memcpy(mxGetPr(prhs[3]), &edgeLength, (size_t) sizeof(double));
    memcpy(mxGetPr(prhs[4]), &facets, (size_t) sizeof(int));
    
    /*call the mexfunction*/
    mexFunction(nlhs, plhs, nrhs, prhs);

    printf("Hello, World!\n");
    return 0;
}
