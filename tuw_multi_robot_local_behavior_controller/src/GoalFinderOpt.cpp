#include <stdio.h>
#include <stdlib.h>
#include "tuw_multi_robot_route_to_path/GoalFinder.h"
#include "tuw_multi_robot_route_to_path/GoalFinderOpt.h"
#include <limits>
#include <tf/tf.h>
#include <pose_cov_ops/pose_cov_ops.h>


#define _USE_MATH_DEFINES

namespace goal_finder_opt
{

template <typename R>
R wrapScalarToPi(R angle)
{
    if (angle >= M_PI)
        angle = fmod(angle + M_PI, 2*M_PI) - M_PI;
    else if (angle < -M_PI)
        angle = fmod(angle - M_PI, 2*M_PI) + M_PI;
    return angle;
};

void problem_function(double *f, double *g, double *x, const gm::GridMap& grid_map, const geometry_msgs::PoseStamped& orig_goal);

extern "C" { int midaco(long int*,long int*,long int*,long int*,long int*,
                        long int*,double*,double*,double*,double*,double*,
                        long int*,long int*,double*,double*,long int*,
                        long int*,long int*,double*,long int*,char*); }

extern "C" { int midaco_print(int,long int,long int,long int*,long int*,double*,
                              double*,double*,double*,double*,long int,long int,
                              long int,long int,long int,double*,double*,
                              long int,long int,double*,long int,char*); }

bool findOptGoal(int timeout,
                 const std::string& base_link_frame_id,
                 const tf::TransformListener& tf_listener,
                 const gm::GridMap& grid_map,
                 const geometry_msgs::PolygonStamped& footprint,
                 const geometry_msgs::PoseStamped& orig_goal,
                 geometry_msgs::PoseStamped& new_goal)
{
    /*****************************************************************/
    /***  Step 1: Problem definition  ********************************/
    /*****************************************************************/

    /* STEP 1.A: Problem dimensions
    ******************************/
    long int o  = 2; /* Number of objectives                          */
    long int n  = 3; /* Number of variables (in total)                */
    long int ni = 2; /* Number of integer variables (0 <= ni <= n)    */
    long int m  = 1; /* Number of constraints (in total)              */
    long int me = 1; /* Number of equality constraints (0 <= me <= m) */

    /* Variable and Workspace Declarations */
    double x[n], xl[n], xu[n], f[o], g[m];
    long int maxeval;
    long int maxtime;
    long int printeval;
    long int save2file;
    long int iflag;
    long int istop;
    double param[13];

    // TODO
    long int liw, lrw, lpf, i, iw[5000], p=1;
    double rw[20000], pf[20000];

    char key[] = "MIDACO_LIMITED_VERSION___[CREATIVE_COMMONS_BY-NC-ND_LICENSE]";

    /* STEP 1.B: Lower and upper bounds 'xl' & 'xu'
    **********************************************/
    // int max_x = grid_map.getLength().x();
    // int max_y = grid_map.getLength().y();

    double most_dist_vertex_norm = 0;
    for (const auto& vertex : footprint.polygon.points)
    {
        double norm = sqrt(vertex.x*vertex.x + vertex.y*vertex.y);
        if (norm > most_dist_vertex_norm)
        {
            most_dist_vertex_norm = norm;
        }
    }
    double offset = ceil(most_dist_vertex_norm / grid_map.getResolution());
    xl[0] = -M_PI + std::numeric_limits<double>::epsilon();
    xl[1] = offset;
    xl[2] = offset;
    xu[0] = M_PI;
    xu[1] = grid_map.getSize() (0) - offset;
    xu[2] = grid_map.getSize() (1) - offset;

    // std::cout << "UBOX 0:\n" << xu[0] << std::endl;
    // std::cout << "UBOX 1:\n" << xu[1] << std::endl;
    // std::cout << "UBOX 2:\n" << xu[2] << std::endl;

    /* STEP 1.C: Starting point 'x'
    ******************************/
    gm::Index goal_idx;
    grid_map.getIndex(gm::Position(orig_goal.pose.position.x, orig_goal.pose.position.y), goal_idx);
    // x[0] = wrapScalarToPi(tf::getYaw(orig_goal.pose.orientation));
    double orig_yaw = wrapScalarToPi(tf::getYaw(orig_goal.pose.orientation));
    x[0] = 0.0;
    x[1] = goal_idx.x();
    x[2] = goal_idx.y();

    // std::cout << "x0 0:\n" << x[0] << std::endl;
    // std::cout << "x0 1:\n" << x[1] << std::endl;
    // std::cout << "x0 2:\n" << x[2] << std::endl;

    /*****************************************************************/
    /***  Step 2: Choose stopping criteria and printing options   ****/
    /*****************************************************************/

    /* STEP 2.A: Stopping criteria
    *****************************/
    maxeval = 10000;      /* Maximum number of function evaluation (e.g. 1000000)  */
    maxtime = timeout;   /* Maximum time limit in Seconds (e.g. 1 Day = 60*60*24) */

    /* STEP 2.B: Printing options
    ****************************/
    printeval = 0;   /* Print-Frequency for current best solution (e.g. 1000) */
    save2file = 0;      /* Save SCREEN and SOLUTION to TXT-files [ 0=NO/ 1=YES]  */

    /*****************************************************************/
    /***  Step 3: Choose MIDACO parameters (FOR ADVANCED USERS)    ***/
    /*****************************************************************/

    param[ 0] =  0.0;    /* ACCURACY  */
    param[ 1] =  0.0;    /* SEED      */
    param[ 2] =  0.0;    /* FSTOP     */
    param[ 3] =  0.0;    /* ALGOSTOP  */
    param[ 4] =  0.0;    /* EVALSTOP  */
    param[ 5] =  0.0;    /* FOCUS     */
    param[ 6] =  0.0;    /* ANTS      */
    param[ 7] =  0.0;    /* KERNEL    */
    param[ 8] =  0.0;    /* ORACLE    */
    param[ 9] =  0.0;    /* PARETOMAX */
    param[10] =  0.0;    /* EPSILON   */
    param[11] =  0.0;    /* BALANCE   */
    param[12] =  0.0;    /* CHARACTER */

    /*****************************************************************/
    /*
       Call MIDACO by Reverse Communication
     */
    /*****************************************************************/
    /* Workspace length calculation */
    lrw = sizeof(rw)/sizeof(double);
    lpf = sizeof(pf)/sizeof(double);
    liw = sizeof(iw)/sizeof(long int);
    /* Print midaco headline and basic information */
    midaco_print(1,printeval,save2file,&iflag,&istop,&*f,&*g,&*x,&*xl,&*xu,
                 o,n,ni,m,me,&*rw,&*pf,maxeval,maxtime,&*param,p,&*key);


    gm::Polygon transf_footprint;
    GoalFinder::transformFootprint(base_link_frame_id,
                                   tf_listener,
                                   orig_goal,
                                   footprint,
                                   transf_footprint);
    int lethal_counts = 0;
    for (gm::PolygonIterator iterator(grid_map, transf_footprint);
         !iterator.isPastEnd(); ++iterator)
    {
        if (grid_map.at("local_costmap", *iterator) == 100 || grid_map.at("local_costmap", *iterator) == -1)
        {
            lethal_counts++;
        }
    }
    printf(" Initial g[0] = %d\n", lethal_counts);

    while(istop == 0)   /*~~~ Start of the reverse communication loop ~~~*/
    {
        // problem_function( &*f, &*g, &*x, grid_map, orig_goal);

        /* Objective functions F(X) */
        geometry_msgs::PoseStamped candidate_goal;
        gm::Position candidate_position;
        geometry_msgs::Pose diff;
        grid_map.getPosition(gm::Index(x[1], x[2]), candidate_position);
        candidate_goal.pose.position.x = candidate_position.x();
        candidate_goal.pose.position.y = candidate_position.y();
        candidate_goal.pose.orientation.x = 0;
        candidate_goal.pose.orientation.y = 0;
        candidate_goal.pose.orientation.z = sin((x[0]+orig_yaw)/2.);
        candidate_goal.pose.orientation.w = cos((x[0]+orig_yaw)/2.);

        pose_cov_ops::inverseCompose(candidate_goal.pose, orig_goal.pose, diff);

        f[0] = sqrt(diff.position.x*diff.position.x + diff.position.y*diff.position.y);
        f[1] = wrapScalarToPi(x[0]);
        f[1] = f[1]*f[1];

        /* Constraints functions G(X) */
        gm::Polygon transf_footprint;
        GoalFinder::transformFootprint(base_link_frame_id,
                                       tf_listener,
                                       candidate_goal,
                                       footprint,
                                       transf_footprint);

        // g[0] = 0;
        int lethal_counts = 0;
        for (gm::PolygonIterator iterator(grid_map, transf_footprint);
             !iterator.isPastEnd(); ++iterator)
        {
            if (grid_map.at("local_costmap", *iterator) == 100 || grid_map.at("local_costmap", *iterator) == -1)
            {
                lethal_counts++;
            }
        }
        g[0] = lethal_counts;

        /* Call MIDACO */
        midaco(&p,&o,&n,&ni,&m,&me,&*x,&*f,&*g,&*xl,&*xu,&iflag,
               &istop,&*param,&*rw,&lrw,&*iw,&liw,&*pf,&lpf,&*key);
        /* Call MIDACO printing routine */
        midaco_print(2,printeval,save2file,&iflag,&istop,&*f,&*g,&*x,&*xl,&*xu,
                     o,n,ni,m,me,&*rw,&*pf,maxeval,maxtime,&*param,p,&*key);
    }   /*~~~End of the reverse communication loop ~~~*/
    /*****************************************************************/
    // printf("\n Solution f[0] = %f ", f[0]);
    printf(" Solution g[0] = %d\n", int(g[0]));
    // printf("\n Solution x[0] = %f ", x[0]);
    /*****************************************************************/
    // printf("\n Pause"); getchar();

    geometry_msgs::Pose new_pose;
    gm::Position new_position;
    grid_map.getPosition(gm::Index(x[1], x[2]), new_position);
    new_goal.pose.position.x = new_position.x();
    new_goal.pose.position.y = new_position.y();
    new_goal.pose.orientation.x = 0;
    new_goal.pose.orientation.y = 0;
    new_goal.pose.orientation.z = sin((x[0]+orig_yaw)/2.);
    new_goal.pose.orientation.w = cos((x[0]+orig_yaw)/2.);

    // solution_idx = gm::Index(x[1], x[2]);
    // pos = gm::Index(x[1], x[2]);

    // geometry_msgs::Pose solution_pose;
    // gm::Position solution_position;
    // geometry_msgs::Pose diff;
    // grid_map.getPosition(gm::Index(x[1], x[2]), solution_position);
    // solution_pose.position.x = solution_position.x();
    // solution_pose.position.y = solution_position.y();
    // solution_pose.orientation.x = 0;
    // solution_pose.orientation.y = 0;
    // solution_pose.orientation.z = sin(x[0]/2.);
    // solution_pose.orientation.w = cos(x[0]/2.);
    //
    // pose_cov_ops::inverseCompose(solution_pose, orig_goal.pose, diff);
    //
    // if (sqrt(diff.position.x*diff.position.x + diff.position.y*diff.position.y) <= ) // f[0] = sqrt(diff.position.x*diff.position.x + diff.position.y*diff.position.y);
    // f[1] = abs(wrapScalarToPi(tf::getYaw(diff.orientation)));

    return true;
}

// void problem_function( double *f, double *g, double *x, const gm::GridMap& grid_map, const geometry_msgs::PoseStamped& orig_goal)
// {
/* Objective functions F(X) */
// }

}
