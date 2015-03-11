//
//  Created by Christopher Helf on 03.03.15.
//  Copyright (c) 2015. All rights reserved.
//  OpenMesh library is required, Code partly taken from OpenFlipper
//
//  Make sure you build OpenMesh library in Release mode, otherwise the
//  dynamic casting won't work under Matlab and you get a segmentation fault

#include <stdint.h>

/*Quick Clang Fix*/
#ifndef char16_t
typedef uint16_t char16_t;
#endif

#include "mex.h"
#include <math.h>
#include <string.h>
#include "src/IsotropicRemesher.h"

/****************************************************************************/
/* Input arguments                                                          */
/****************************************************************************/
#define IN_FACET       		prhs[0]
#define IN_POINTS         	prhs[1]
#define IN_FEATURES       	prhs[2]
#define IN_EDGELENGTH       prhs[3]
#define IN_ITERATIONS       prhs[4]

/****************************************************************************/
/* Output arguments                                                         */
/****************************************************************************/
#define OUT_FACET        	plhs[0]
#define OUT_POINTS        	plhs[1]

/****************************************************************************/
/* Function for loading meshes (*.obj)                                      */
/****************************************************************************/
void loadPoints(const char* name, int* nFacets, int** facets, int* nPoints, double** points) {
     IsotropicRemesher_load(name, nFacets, *facets, nPoints, *points);
}

/****************************************************************************/
/* Main mexFunction                                                         */
/****************************************************************************/
void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[])
{
	/*do some error checking*/
    if (nlhs != 2) {
        mexErrMsgTxt("You need to output two arguments");
    }
    
	if (nrhs != 5) {
		mexErrMsgTxt("You need to input five arguments");
	}

	if ( mxGetM(IN_FACET) == 0 ) {
		mexErrMsgTxt("No Facets Given");
	}

	if ( mxGetN(IN_FACET) != 3 ) {
		mexErrMsgTxt("Facets must have three columns!");
	}

	if ( mxGetM(IN_POINTS) == 0 ) {
		mexErrMsgTxt("No Points Given");
	}

	if ( mxGetN(IN_POINTS) != 3 ) {
		mexErrMsgTxt("Points must have three columns!");
	}

	if ( mxGetN(IN_FEATURES) > 1 ) {
		mexErrMsgTxt("Features should have only one column!");
	}

	if ( mxGetN(IN_EDGELENGTH) != 1 || mxGetN(IN_EDGELENGTH) != 1) {
		mexErrMsgTxt("Edgelength must be a single element scalar!");
	}

	if ( mxGetN(IN_ITERATIONS) != 1 || mxGetN(IN_ITERATIONS) != 1) {
		mexErrMsgTxt("Interations must be a single element scalar!");
	}

	if(!mxIsClass(IN_POINTS,"double"))
	{
	    mexErrMsgTxt("Points must be of type double!!!\n");
	}

    if(!mxIsClass(IN_FEATURES,"int32"))
	{
	    mexErrMsgTxt("Features must be of type int32!!!\n");
	}

	if(!mxIsClass(IN_FACET,"int32"))
	{
	    mexErrMsgTxt("Facets must be of type int32!!!\n");
	}

	/*get iterations and edge length*/
	int iterations = mxGetScalar(IN_ITERATIONS);
	double edgelength = mxGetScalar(IN_EDGELENGTH);

	/*get facets, points and features*/
	int nFacets = (int) mxGetM(IN_FACET);
	int nPoints = (int) mxGetM(IN_POINTS);
	int nFeatures = (int) mxGetM(IN_FEATURES);
    
	int *facets, *features;
	double* points;

    /*we will copy the facet array as we will modify it*/
    int* facetOriginal = (int*) mxGetData(IN_FACET);
    facets = (int*) malloc(nFacets*3*sizeof(int));
    memcpy(facets, facetOriginal, nFacets*3*sizeof(int));
    
    /*we will copy the features array as we will modify it*/
    if(nFeatures>0) {
        int* featuresOriginal = (int*) mxGetData(IN_FEATURES);
        features = (int*) malloc(nFeatures*sizeof(int));
        memcpy(features, featuresOriginal, nFeatures*sizeof(int));
    }

    /*the rest we just take as given*/
	points  = (double*) mxGetData(IN_POINTS);
    
    /*check minimum and maximum*/
    int min = -1;
    int max = -1;
    
    /*decrease facets by one due to different array structure*/
    for(int i=0; i<nFacets*3; i++) {
        facets[i]=facets[i]-1;
        if(min<0 || facets[i]<min) min=facets[i];
        if(max<0 || facets[i]>max) max=facets[i];
    }
    
    /*throw error if one facet is out of range for the points*/
    if(min < 0 || max > nPoints) {
        mexErrMsgTxt("Facets must reference to points!!!\n");
    }
    
    min = -1;
    max = -1;
    
    /*decrease facets by one due to different array structure*/
    for(int i=0; i<nFeatures; i++) {
        features[i]=features[i]-1;
        if(min<0 || features[i]<min) min=features[i];
        if(max<0 || features[i]>max) max=features[i];
    }
    
    /*throw error if one facet is out of range for the points*/
    if((min < 0 || max > nPoints) && nFeatures>0) {
        mexErrMsgTxt("Features must reference to points!!!\n");
    }
   

	/*point output*/
    double* pointsOut;
    int nPointsOut;
	
	/*facets output*/
	int* facetsOut;
    int nFacetsOut;
	
	/*remesher instance*/
	IsotropicRemesher* remesherInstance = IsotropicRemesher_new(nFacets, facets, nPoints, points, nFeatures, features, edgelength, iterations);
    
    /*the actual remesing*/
	int success = IsotropicRemesher_remesh(remesherInstance);
    
    /*check for errors*/
    if(!success) {
        mexErrMsgTxt("Something went wrong during the remeshing process\n");
        IsotropicRemesher_delete(remesherInstance);
        free(facets);
        return;
    } else {
        
        /*assign variables*/
        nFacetsOut = IsotropicRemesher_nF(remesherInstance);
        nPointsOut = IsotropicRemesher_nP(remesherInstance);
        pointsOut = IsotropicRemesher_points(remesherInstance);
        facetsOut = IsotropicRemesher_facets(remesherInstance);
    }
    
    /*create outputs*/
    OUT_FACET = mxCreateNumericMatrix(nFacetsOut, 3, mxINT32_CLASS, mxREAL);
    OUT_POINTS = mxCreateDoubleMatrix(nPointsOut, 3, mxREAL);
    
    /*assign outputs*/
    double* _opts = mxGetPr(OUT_POINTS);
    int* _ofts = (int*) mxGetPr(OUT_FACET);
    
    /*fill outputs, increase facets back by one*/
    for(int i=0; i<nFacetsOut*3; i++) _ofts[i] = facetsOut[i]+1;
    for(int i=0; i<nPointsOut*3; i++) _opts[i] = pointsOut[i];
    
    /*delete the allocated object & memory*/
    IsotropicRemesher_delete(remesherInstance);
    free(facets);
    
    return;

}


