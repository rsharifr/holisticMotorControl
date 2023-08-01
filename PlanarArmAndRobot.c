#include <math.h>
#include "mex.h"
#include "matrix.h"
#include "cPlanarArmAndRobot.c"

enum inputs {
    STARTAT = 0,
    IC,
    INP,
    PARAMS,
    DT,
    NPTS,
};

void mexFunction(int nlhs, /* number of expected outputs  */
        mxArray *plhs[], /* array of pointers to output arguments */
        int nrhs, /* number of inputs*/
        const mxArray *prhs[])/* array of pointers to input arguments */
{
    /*******************************************************************/
    /* Expected inputs */
    /* t0, initialCond, inputs, parameters */
    /* or
     * /* t0, initialCond, inputs, parameters, dt, npts  */
    
    /*Expected Outputs*/
    /* diff - array of state derivatives*/
    /* state - array of states */
    /* out - array of outputs */
    /* error - string describing error */
    
    /*******************************************************************/  
    /* Process input mode*/
    /*******************************************************************/
    
    if ((nrhs!=6)&&(nrhs!=4)) mexErrMsgIdAndTxt("ErrSource:MexFunction",
            "Incompatible number of inputs.\nExpected inputs: (t0, initialCond, inputs, parameters, dt, npts)");
    
    /* Input variables */
    double t0, dt, *ic, *p, *u;
    long npts;
    
    /* Get inputs out of the mex call */
    t0 = *mxGetPr(prhs[STARTAT]);
    ic = mxGetPr(prhs[IC]);
    p = mxGetPr(prhs[PARAMS]);
    u = mxGetPr(prhs[INP]);
    if (nrhs==6)
    {
        dt = *mxGetPr(prhs[DT]);
        npts = *mxGetPr(prhs[NPTS]);
        if (npts<1) mexErrMsgIdAndTxt("ErrSource:MexFunction","Number of integration points should be at least 1");
    }
    else if(nrhs==4)
    {
        dt = 1e-3;
        npts = 1;
    }
    
    
    /*******************************************************************/
    /* Process outputs*/
    /*******************************************************************/

    // Pointer to the "states"
    plhs[0] = mxCreateDoubleMatrix(NDIFF,1,mxREAL);
    // Pointer to the "state derivatives"
    plhs[1] = mxCreateDoubleMatrix(NDIFF,1,mxREAL);
    // Pointer to the "outputs" (# outputs +1 for time)
    plhs[2] = mxCreateDoubleMatrix(NOUT+1,npts,mxREAL);
    
    
    double *state, *diff, *out;
    diff = mxGetPr(plhs[0]);
    state = mxGetPr(plhs[1]);
    out = mxGetPr(plhs[2]);
    
    

    /*******************************************************************/
    /* Solver variables */
    char errbuf[1000]="0";
    long internal = 0;
    SolverStruct S;
    
    double y[NOUT+1];
    double w[1+2*NEQ+NPAR+NDFA+NEVT];
    long i,j;

    /* Setup */
    for(i=0;i<npts*(NOUT+1);i++) out[i]=*dsn_undef;
    S.w=w;
    if(internal==0) S.err=0; else S.err=-1;
    S.buf=errbuf;
    SolverSetup(t0,ic,u,p,y,dt,&S);
    
    out[0]=t0; for(j=0;j<NOUT;j++) out[j+1]=y[j];
    
    /* Integrate the states */
    for(i=1;i<npts;i++)
    {
        RK4Step(u,&S);
        SolverOutputs(y,&S);
        out[i*(NOUT+1)]=S.w[0];
        for(j=0;j<NOUT;j++) out[i*(NOUT+1)+j+1]=y[j];
    }
    
    /*Assign the last step states derivatives*/
    for(j=0;j<NDIFF;j++) state[j]=S.w[j+1];
    for(j=0;j<NDIFF;j++) diff[j]=S.w[j+NEQ+NPAR+1];
    plhs[3] = mxCreateString(errbuf);
    
    return;
}