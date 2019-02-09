#include <stdio.h>
#include <stdlib.h>
#include "tuw_multi_robot_route_to_path/GoalFinderOpt.h"

/***********************************************************************/
/*
//     This is an example call of MIDACO 6.0
//     -------------------------------------
//
//     MIDACO solves Multi-Objective Mixed-Integer Non-Linear Problems:
//
//
//      Minimize     F_1(X),... F_O(X)  where X(1,...N-NI)   is CONTINUOUS
//                                      and   X(N-NI+1,...N) is DISCRETE
//
//      subject to   G_j(X)  =  0   (j=1,...ME)      equality constraints
//                   G_j(X) >=  0   (j=ME+1,...M)  inequality constraints
//
//      and bounds   XL <= X <= XU
//
//
//     The problem statement of this example is given below. You can use
//     this example as template to run your own problem. To do so: Replace
//     the objective functions 'F' (and in case the constraints 'G') given
//     here with your own problem and follow the below instruction steps.
*/
/***********************************************************************/
void problem_function( double *F, double *G, double *X );
/***********************************************************************/
extern"C"{ int midaco(long int*,long int*,long int*,long int*,long int*,
                      long int*,double*,double*,double*,double*,double*,
                      long int*,long int*,double*,double*,long int*,
                      long int*,long int*,double*,long int*,char*);}
/***********************************************************************/
extern"C"{ int midaco_print(int,long int,long int,long int*,long int*,double*,
                            double*,double*,double*,double*,long int,long int,
                            long int,long int,long int,double*,double*,
                            long int,long int,double*,long int,char*);}
/***********************************************************************/
/************************   MAIN PROGRAM   *****************************/
/***********************************************************************/
int goal_finder::mainfunc()
{
      /* Variable and Workspace Declarations */
      long int o,n,ni,m,me,maxeval,maxtime,printeval,save2file,iflag,istop;
      long int liw,lrw,lpf,i,iw[5000],p=1; double rw[20000],pf[20000];
      double   f[10],g[1000],x[1000],xl[1000],xu[1000],param[13];
      char key[] = "MIDACO_LIMITED_VERSION___[CREATIVE_COMMONS_BY-NC-ND_LICENSE]";

      /*****************************************************************/
      /***  Step 1: Problem definition  ********************************/
      /*****************************************************************/

      /* STEP 1.A: Problem dimensions
      ******************************/
      o  = 1; /* Number of objectives                          */
      n  = 4; /* Number of variables (in total)                */
      ni = 2; /* Number of integer variables (0 <= ni <= n)    */
      m  = 3; /* Number of constraints (in total)              */
      me = 1; /* Number of equality constraints (0 <= me <= m) */

      /* STEP 1.B: Lower and upper bounds 'xl' & 'xu'
      **********************************************/
      for( i=0; i<n; i++)
      {
         xl[i] = 1.0;
         xu[i] = 4.0;
      }

      /* STEP 1.C: Starting point 'x'
      ******************************/
      for( i=0; i<n; i++)
      {
         x[i] = xl[i]; /* Here for example: starting point = lower bounds */
      }

      /*****************************************************************/
      /***  Step 2: Choose stopping criteria and printing options   ****/
      /*****************************************************************/

      /* STEP 2.A: Stopping criteria
      *****************************/
      maxeval = 10000;    /* Maximum number of function evaluation (e.g. 1000000)  */
      maxtime = 60*60*24; /* Maximum time limit in Seconds (e.g. 1 Day = 60*60*24) */

      /* STEP 2.B: Printing options
      ****************************/
      printeval = 1000; /* Print-Frequency for current best solution (e.g. 1000) */
      save2file = 1;    /* Save SCREEN and SOLUTION to TXT-files [ 0=NO/ 1=YES]  */

      /*****************************************************************/
      /***  Step 3: Choose MIDACO parameters (FOR ADVANCED USERS)    ***/
      /*****************************************************************/

      param[ 0] =  0.0;  /* ACCURACY  */
      param[ 1] =  0.0;  /* SEED      */
      param[ 2] =  0.0;  /* FSTOP     */
      param[ 3] =  0.0;  /* ALGOSTOP  */
      param[ 4] =  0.0;  /* EVALSTOP  */
      param[ 5] =  0.0;  /* FOCUS     */
      param[ 6] =  0.0;  /* ANTS      */
      param[ 7] =  0.0;  /* KERNEL    */
      param[ 8] =  0.0;  /* ORACLE    */
      param[ 9] =  0.0;  /* PARETOMAX */
      param[10] =  0.0;  /* EPSILON   */
      param[11] =  0.0;  /* BALANCE   */
      param[12] =  0.0;  /* CHARACTER */

      /*****************************************************************/
      /*
         Call MIDACO by Reverse Communication
      */
      /*****************************************************************/
      /* Workspace length calculation */
      lrw=sizeof(rw)/sizeof(double);
      lpf=sizeof(pf)/sizeof(double);
      liw=sizeof(iw)/sizeof(long int);
      /* Print midaco headline and basic information */
      midaco_print(1,printeval,save2file,&iflag,&istop,&*f,&*g,&*x,&*xl,&*xu,
                 o,n,ni,m,me,&*rw,&*pf,maxeval,maxtime,&*param,p,&*key);
      while(istop==0) /*~~~ Start of the reverse communication loop ~~~*/
      {
          /* Evaluate objective function */
          problem_function( &*f, &*g, &*x);

          /* Call MIDACO */
          midaco(&p,&o,&n,&ni,&m,&me,&*x,&*f,&*g,&*xl,&*xu,&iflag,
                 &istop,&*param,&*rw,&lrw,&*iw,&liw,&*pf,&lpf,&*key);
          /* Call MIDACO printing routine */
          midaco_print(2,printeval,save2file,&iflag,&istop,&*f,&*g,&*x,&*xl,&*xu,
                       o,n,ni,m,me,&*rw,&*pf,maxeval,maxtime,&*param,p,&*key);
      } /*~~~End of the reverse communication loop ~~~*/
      /*****************************************************************/
      // printf("\n Solution f[0] = %f ", f[0]);
      // printf("\n Solution g[0] = %f ", g[0]);
      // printf("\n Solution x[0] = %f ", x[0]);
      /*****************************************************************/
      // printf("\n Pause"); getchar();
      return 0;
      /*****************************************************************/
      /***********************  END OF MAIN  ***************************/
      /*****************************************************************/
}



/***********************************************************************/
/*********************   OPTIMIZATION PROBLEM   ************************/
/***********************************************************************/
void problem_function( double *f, double *g, double *x )
{
    /* Objective functions F(X) */
    f[0] = (x[0]-1.0)*(x[0]-1.0)
         + (x[1]-2.0)*(x[1]-2.0)
         + (x[2]-3.0)*(x[2]-3.0)
         + (x[3]-4.0)*(x[3]-4.0)
         + 1.23456789;

    /* Equality constraints G(X) = 0 MUST COME FIRST in g[0:me-1] */
    g[0] = x[0] - 1.0;
    /* Inequality constraints G(X) >= 0 MUST COME SECOND in g[me:m-1] */
    g[1] = x[1] - 1.333333333;
    g[2] = x[2] - 2.666666666;
}
/***********************************************************************/
/*************************** END OF FILE *******************************/
/***********************************************************************/
