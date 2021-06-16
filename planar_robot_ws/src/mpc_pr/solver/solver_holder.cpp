#include "acado_common.h"
#include "acado_auxiliary_functions.h"
#include "solver_holder.h"

#include <stdio.h>
#include <math.h>
#include "ros/ros.h"

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD  /* Number of online data values. */

#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */

#define N           ACADO_N   /* Number of intervals in the horizon. */

//#define NUM_STEPS   5        /* Number of real-time iterations. */
//#define VERBOSE     0         /* Show iterations: 1, silent: 0.  */

/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;

//MPC_solver::MPC_solver() : num_steps(0) { }
MPC_solver::MPC_solver(int n): num_steps(n) {
	int i;
	/* Initialize the solver. */
	acado_initializeSolver();

	/* Initialize the states and controls. */
	for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 0.0;
	for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;

	/* Initialize the measurements/reference. */
	for (i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0;
};

int MPC_solver::reinitialize(){
	int i;
	/* Initialize the solver. */
	acado_initializeSolver();

	/* Initialize the states and controls. */
	for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[ i ] = 0.0;
	for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;

	/* Initialize the measurements/reference. */
	for (i = 0; i < NY * N; ++i)  acadoVariables.y[ i ] = 0.0;
	return 0;
};

double * MPC_solver::solve_mpc(double input_arr[4]) {
	/* currentState_targetValue parsing:
	0 - 6: current theta
	7 - 13: goal theta

	output j_dot[7]
	*/
	double x0[] = {input_arr[0], input_arr[1]};
	double yN[] = {input_arr[3], input_arr[4]};
	
	printf("\ncurrent: %.3f, %.3f", x0[0], x0[1]);
	printf("\n___goal: %.3f, %.3f", yN[0], yN[1]);

	/* Some temporary variables. */
	int    i, j, iter;
	acado_timer t;
	
	// in model we have 7 theta and 7 theta_dot, we assign fixed reference with zero velocity
        
    double diff_of_value[2] = {0.0, 0.0};
	for (i = 0; i < 2; i++) 
	{
		diff_of_value[i] = yN[i] - x0[i]; // find differences for xyz
	}

	int count_step = 0; // each step
	for (j = 0; j < N; ++j) 
	{
        for (i = 0; i < 2; i++){
            acadoVariables.y[j*NY + i] = x0[i]+j*diff_of_value[i]/N;	
        }
    }

	
    // terminal goal
	for (i = 0; i < NYN; ++i)  acadoVariables.yN[i] = yN[ i ];

	/* Initialize online data. */
	// for (j = 0; j < N; ++j) {
	// 	for (i = 0; i < NOD; ++i)  acadoVariables.od[ j*NOD+i ] = human_spheres[i];
	// }
        //printf("values %.3f %.3f %.3f %.3f", human_spheres[44], human_spheres[45], human_spheres[46], human_spheres[47]);

	/* MPC: initialize the current state feedback. */
    #if ACADO_INITIAL_STATE_FIXED
	for (i = 0; i < 2; ++i) acadoVariables.x0[ i ] = x0[ i ];
    #endif

	for (i = 0; i < NU * N; ++i)  acadoVariables.u[ i ] = 0.0;

	acado_tic( &t );
	for(iter = 0; iter < num_steps; ++iter)
	    {
		/* Prepare for the RTI step. */
		acado_preparationStep();
		/* Compute the feedback step. */
		acado_feedbackStep( );
	    }
	real_t te = acado_toc( &t );
	real_t KKT_val = acado_getKKT();

	ROS_INFO("Time: %.3g ms; KKT = %.3e", 1e3 * te, KKT_val);


	static double joint_commands[5];
        joint_commands[4] = 0;  // infisibility marker
        for (i = 0; i < 2; ++i) {
            if (acadoVariables.u[i] != acadoVariables.u[i]){
                acadoVariables.u[i] = 0;
                joint_commands[4] = 1;
            }
	    joint_commands[i] = acadoVariables.u[i];
	}
	joint_commands[2] = 1e3 * te;
	joint_commands[3] = KKT_val;

	if (KKT_val > 0.01) {
		acado_initializeSolver();

		/* Initialize the states and controls. */
		for (i = 0; i < NX * (N + 1); ++i)  acadoVariables.x[i] = 0.0;
		for (i = 0; i < NU * N; ++i)  acadoVariables.u[i] = 0.0;

		/* Initialize the measurements/reference. */
		for (i = 0; i < NY * N; ++i)  acadoVariables.y[i] = 0.0;
	} else {
		acado_shiftStates(2, 0, 0);
		acado_shiftControls( 0 );
	}
	
	return joint_commands;
}
