/*
This software is freely available for use in research, teaching and other
non-commercial purposes.  Users have the right to modify, alter, improve,
or enhance the software without limitation, under the condition that you
do not remove or alter the copyright information in this section.

If you publish results which were obtained using the software, or its
source code, please cite the software with a reference to the associated
publication:

J.H. Boyle, S. Berri and N. Cohen (2012), Gait modulation in C. elegans:
an integrated neuromechanical model, Front. Comput. Neurosci, 6:10
doi: 10.3389/fncom.2012.00010

Licence information for Sundials IDA can be found in "sundials-2.3.0/LICENCE"
*/

#include "wormsim.h"

#ifdef _WIN32
#define M_PI 3.14159265358979323846
#define HALFPI 1.57079632679489661923
#define SUNDIALS_DOUBLE_PRECISION 1
#endif

// Required includes
#include <iostream>
#include <cmath>
#include <fstream>
#include <cstdlib>
#include <android/log.h>

#ifdef _WIN32
#define M_PI 3.14159265358979323846
#define HALFPI 1.57079632679489661923
#define SUNDIALS_DOUBLE_PRECISION 1
#include <vector>
#endif

#include <ida/ida.h>
#include <ida/ida_dense.h>
#include <nvector/nvector_serial.h>
#include <sundials/sundials_types.h>
#include <sundials/sundials_math.h>

using namespace std;

// Simulator constants
#define MEDIUM 1.0
#define NSEG_MINUS_1 (NSEG-1)
#define NEQ   3*(NBAR)
#define DELTAT .1
#define HALFPI M_PI/2.0

// General body constants
realtype D = 80e-6;
realtype R[NBAR];
realtype L_seg = 1e-3/NSEG;

// Horizontal element constants
realtype k_PE = (NSEG/24.0)*RCONST(10.0e-3);
realtype D_PE = RCONST(0.025)*k_PE;
realtype AE_PE_ratio = RCONST(20.0);
realtype k_AE = AE_PE_ratio*k_PE;
realtype D_AE = RCONST(5.0)*AE_PE_ratio*D_PE;

// Diagonal element constants
realtype k_DE = RCONST(350.0)*k_PE;
realtype D_DE = RCONST(0.01)*k_DE;

// Length constant holders
realtype L0_P[NSEG];
realtype L_min[NSEG];
realtype L0_P_minus_L_min[NSEG];
realtype L0_D[NSEG];

// Stretch receptor constant holders
realtype SR_shape_compensation[NSEG];

// Muscle time constant
realtype T_muscle = RCONST(0.1);

// Environment constants
realtype CL_water = RCONST(3.3e-6 / (2.0 * NBAR));
realtype CN_water = RCONST(5.2e-6 / (2.0 * NBAR));
realtype CL_agar  = RCONST(3.2e-3 / (2.0 * NBAR));
realtype CN_agar  = RCONST(128e-3 / (2.0 * NBAR));

// Environment variables
realtype                K_agar = CN_agar / CL_agar;
realtype                CN[NBAR];
realtype                CL[NBAR];

// Variables for total input current to each B-class motorneuron
const int N_units = 12;            // Number of neural units
double     I_D[N_units];
double     I_V[N_units];

// Communication variables
realtype L_SR[NSEG][2];
realtype I_SR[NSEG][2];

// Neuron and muscle state variables
realtype I_on = 0.675;         // AVB input current (makes the model go)
int      State[N_units][2];
realtype V_muscle[NSEG][2];
realtype V_neuron[NSEG][2];
realtype I     = 0.5;
realtype theta = -0.5;
realtype G     = 0.0;

// Steering.
#define ASER_ACTIVATION 50.0
#define ASEL_ACTIVATION 5.0
struct SteeringNeuron
{
	realtype I;
	realtype theta;
	realtype activation;
};
struct SteeringNeuron asel, aser;
struct SteeringNeuron aiyl, aiyr;
struct SteeringNeuron aizl, aizr;
struct SteeringNeuron smbd, smbv;
realtype asel0_weight;
realtype asel1_weight;
realtype aser0_weight;
realtype aser1_weight;
realtype aiyl0_weight;
realtype aiyr0_weight;
realtype aizl0_weight;
realtype aizl1_weight;
realtype aizr0_weight;
realtype aizr1_weight;
#define ASER_SMB_MUSCLE_AMPLIFIER 30.0
#define ASEL_SMB_MUSCLE_AMPLIFIER 3.5
double dorsal_smb_muscle_amplifier;
double ventral_smb_muscle_amplifier;
enum { NEUTRAL=0, STABILIZE=1, DORSAL=2, VENTRAL=3 } smb_muscle_amplifier_control;
double ventral_salt_stimulus, dorsal_salt_stimulus, salt_stimulus_marker;
double head_angle_tracker[2];
#define ASER_TRIGGER_THRESHOLD 2
int aser_trigger_count;
void set_smb_muscle_amplification();

// IDA variables (Copied from Sundials examples)
void *mem;
N_Vector yy, yp, avtol;
realtype rtol, *yval, *ypval, *atval;
realtype t0, tout, tret;
int retval;

// Prototypes of functions called by IDA (Copied from Sundials examples)
int resrob(realtype tres, N_Vector yy, N_Vector yp, N_Vector resval, void *rdata);
static int grob(realtype t, N_Vector yy, N_Vector yp, realtype *gout, void *g_data);

// Update function prototypes.
void update_neurons(double salt_stimulus);
void update_steering_neurons(double salt_stimulus);
void update_muscles();
void update_SR();

// Prototypes of private functions (Copied from Sundials examples)
static int check_flag(void *flagvalue, char *funcname, int opt);
double randn(double mu, double sigma);
void rotatePoint(realtype *x, realtype *y, realtype angle);

// Set steering neuron synapse weight.
void set_steering_synapse_weight(int synapse, double weight)
{
	switch(synapse)
	{
		case ASEL0:
			asel0_weight = weight;
			break;
		case ASEL1:
			asel1_weight = weight;
			break;
		case ASER0:
			aser0_weight = weight;
			break;
		case ASER1:
			aser1_weight = weight;
			break;
		case AIYL0:
			aiyl0_weight = weight;
			break;
		case AIYR0:
			aiyr0_weight = weight;
			break;
		case AIZL0:
			aizl0_weight = weight;
			break;
		case AIZL1:
			aizl1_weight = weight;
			break;
		case AIZR0:
			aizr0_weight = weight;
			break;
		case AIZR1:
			aizr1_weight = weight;
			break;
	}
}

// Get steering neuron activation.
double get_steering_activation(int neuron)
{
	switch(neuron)
	{
		case ASEL:
			return asel.activation;
		case ASER:
			return aser.activation;
		case AIYL:
			return aiyl.activation;
		case AIYR:
			return aiyr.activation;
		case AIZL:
			return aizl.activation;
		case AIZR:
			return aizr.activation;
		case SMBD:
			return smbd.activation;
		case SMBV:
			return smbv.activation;
	}
	return 0.0;
}

// Get dorsal motor neuron activation.
double get_dorsal_motor_activation(int segment)
{
    return V_neuron[segment][0];
}

// Get ventral motor neuron activation.
double get_ventral_motor_activation(int segment)
{
	return V_neuron[segment][1];
}


// Muscle activations.
double dorsal_muscle_activations[N_units];
double ventral_muscle_activations[N_units];
double get_dorsal_muscle_activation(int segment)
{
	return dorsal_muscle_activations[segment];
}
double get_ventral_muscle_activation(int segment)
{
	return ventral_muscle_activations[segment];
}

// Segment angles.
#define SEGMENT_ANGLE_SCALE 0.5
double segment_angles[N_units];
double get_segment_angle(int segment)
{
    return segment_angles[segment];
}

// Get body point.
double get_body_point(int index)
{
	return yval[index];
}

/*
 *--------------------------------------------------------------------
 *Initialize
 *--------------------------------------------------------------------
 */

int init()
{
	mem = NULL;
	yy = yp = avtol = NULL;
	yval = ypval = atval = NULL;

	// Allocate N-vectors (Copied from Sundials examples)
	yy = N_VNew_Serial(NEQ);
	if(check_flag((void *)yy, "N_VNew_Serial", 0)) return(0);
	yp = N_VNew_Serial(NEQ);
	if(check_flag((void *)yp, "N_VNew_Serial", 0)) return(0);
	avtol = N_VNew_Serial(NEQ);
	if(check_flag((void *)avtol, "N_VNew_Serial", 0)) return(0);

	// Create and initialize  y, y', and absolute tolerance vectors (Copied from Sundials examples)
	yval  = NV_DATA_S(yy);
	ypval = NV_DATA_S(yp);
	rtol = (MEDIUM < 0.015 ? 0.1 : 1)*RCONST(1.0e-12);
	atval = NV_DATA_S(avtol);

	for(int i = 0; i < NBAR; ++i)
	{
		// Initialize body in straight line
		yval[i*3] = i*L_seg;
		yval[i*3+1] = RCONST(0.0);
		yval[i*3+2] = M_PI/RCONST(2.0);

		// Initialize derivative values (Copied from Sundials examples)
		ypval[i*3] = RCONST(0.0);
		ypval[i*3+1] = RCONST(0.0);
		ypval[i*3+2] = RCONST(0.0);

		// Set absolute tolerances for solver (Copied from Sundials examples)
		// Tolerance must be set lower when simulating in water, due to lower drag coefficients
		atval[i*3] = (MEDIUM < 0.015 ? 0.1 : 1)*RCONST(1.0e-9);
		atval[i*3+1] = (MEDIUM < 0.015 ? 0.1 : 1)*RCONST(1.0e-9);
		atval[i*3+2] = (MEDIUM < 0.015 ? 0.1 : 1)*RCONST(1.0e-5);

	}

	// Initialize model variables
	for(int i = 0; i < NSEG; ++i)
	{
		V_muscle[i][0] = 0.0;
		V_muscle[i][1] = 0.0;
		V_neuron[i][0] = 0.0;
		V_neuron[i][1] = 0.0;
		I_SR[i][0] = 0.0;
		I_SR[i][1] = 0.0;
	}

	// Set local body radius values based on elliptical approximation
	for(int i = 0; i < NBAR; ++i)
	{
		R[i] = D/2.0*fabs(sin(acos((i-NSEG/2.0)/(NSEG/2.0 + 0.2))));
	}

	// Set stretch receptor weightings that compensate for the elliptical shape,
	// giving approximately the same SR response to segment mending angle
	for(int i = 0; i < NSEG; ++i)
	{
		SR_shape_compensation[i] = D/(R[i] + R[i+1]);
	}

	// Set muscle constants (rest length, minimum length etc) accounting
	// for length differences due to the elliptical body shape
	for(int i = 0; i < NSEG; ++i)
	{
		double scale = 0.65*((R[i] + R[i+1])/D);
		L0_P[i] = sqrt(pow(L_seg,2) + pow((R[i] - R[i+1]),2));
		L_min[i] = (1.0-scale)*L0_P[i];
		L0_P_minus_L_min[i] = L0_P[i] - L_min[i];
		L0_D[i] = sqrt(pow(L_seg,2) + pow((R[i] + R[i+1]),2));
	}

	// Set drag constants according to medium
	for(int i = 0; i < NBAR; ++i)
	{
		CL[i] = (CL_agar - CL_water)*MEDIUM + CL_water;
		CN[i] = (CN_agar - CN_water)*MEDIUM + CN_water;
	}

	// Integration start time
	t0 = RCONST(0.0);

	// Call IDACreate and IDAMalloc to initialize IDA memory (Copied from Sundials examples)
	mem = IDACreate();
	if(check_flag((void *)mem, "IDACreate", 0)) return(0);

	retval = IDAMalloc(mem, resrob, t0, yy, yp, IDA_SV, rtol, avtol);
	if(check_flag(&retval, "IDAMalloc", 1)) return(0);

	// Free avtol (Copied from Sundials examples)
	N_VDestroy_Serial(avtol);

	// Call IDADense and set up the linear solver (Copied from Sundials examples)
	retval = IDADense(mem, NEQ);
	if(check_flag(&retval, "IDADense", 1)) return(0);

	// Integrator inputs
	tout = DELTAT;

    // Initialize steering.
    dorsal_smb_muscle_amplifier = 1.0;
    ventral_smb_muscle_amplifier = 1.0;
    smb_muscle_amplifier_control = NEUTRAL;
    ventral_salt_stimulus = dorsal_salt_stimulus = -1.0;
    salt_stimulus_marker = 0.0;
    head_angle_tracker[0] = head_angle_tracker[1] = 0.0;
    aser_trigger_count = 0;

	return 1;
}

/*
*--------------------------------------------------------------------
*Step
*--------------------------------------------------------------------
*/

void step(double salt_stimulus)
{
	// Call stretch receptor update function
	update_SR();

	// Call neural model update function
	update_neurons(salt_stimulus);

	//Call muscle model update function
	update_muscles();

    // Call residual function (Copied from Sundials examples)
    // to update physical model (Sundials takes multiple steps)
    retval = IDASolve(mem, tout, &tret, yy, yp, IDA_NORMAL);

    // Check integration went ok (Copied from Sundials examples)
    if(check_flag(&retval, "IDASolve", 1)) return;

    // Publish segment angles.
    for (int i = 0; i < NSEG; i += 4)
    {
        double a = 0.0;
        for (int j = 0; j < 4; j++) {
            a += ((yval[i * 3 + 2] * (180.0 / M_PI)) - 90.0);
        }
        a /= 4.0;
        a *= SEGMENT_ANGLE_SCALE;
        a = (int)(a * 1000.0) / 1000;
        if (a < -180.0)
        {
            a += 360.0;
        } else if (a > 180.0)
        {
            a -= 360.0;
        }
        segment_angles[i / 4] = a;
    }

	// Prepare to go to next step
	if (retval == IDA_SUCCESS)
	{
		tout += DELTAT;
	}
}

/*
*--------------------------------------------------------------------
*Terminate
*--------------------------------------------------------------------
*/

void terminate()
{
	// Free memory (Copied from Sundials examples)
	IDAFree(&mem);
	N_VDestroy_Serial(yy);
	N_VDestroy_Serial(yp);
}

/*
 *--------------------------------------------------------------------
 * Model Functions
 *--------------------------------------------------------------------
 */

// Neural circuit function
void update_neurons(double salt_stimulus)
{
	// Update steering.
	update_steering_neurons(salt_stimulus);

	// Neural paramaters
	const double Hyst = 0.5;		// Neural hysteresis
    double I_coupling = 0.0;		// Optional gap junction coupling between adjacent neurons (has virtually no effect, not usually used)

	// Set up neuromuscular junctions
    double NMJ_weight[NSEG];
	for(int i = 0; i < NSEG; ++i)
	{
		NMJ_weight[i] =  0.7*(1.0 - i * 0.6/NSEG);	// Decreasing gradient in NMJ strength / muscle efficacy
	}
	NMJ_weight[0] /= 1.5;				// Helps to prevent excessive bending of head

	// If this is the first time update_neurons is called, initialize with all neurons on one side ON
	static bool initialized = false;
	if(!initialized)
	{
		for(int i = 0; i < N_units; ++i)
		{
			State[i][0] = 1;
			State[i][1] = 0;
		}
		initialized = true;
	}

	// Stretch receptor variables
    double I_SR_D[N_units];
    double I_SR_V[N_units];
    double SR_weight[N_units];

	int N_SR = 6;	//This refers to the number of UNITS (not segments) that each unit receives feedback from (thus 1 means just local feedback)
	int N_seg_per_unit = (int)(NSEG/N_units);

	// SR_weight is a global weighting for each unit, used to get the compensate for curvature gradient induced by the NMJ gradient above
	for(int i = 0; i < N_units; ++i)
	{
		SR_weight[i] = 0.65*(0.4 + 0.08*i)*(N_units/12.0)*(2.0/N_seg_per_unit);
	}

	// Add up stretch receptor contributions from all body segments in receptive field for each neural unit
	for(int i = 0; i <= N_units - N_SR; ++i)
	{
		I_SR_D[i] = 0.0;
		I_SR_V[i] = 0.0;
		for(int j = 0; j < N_SR; ++j)
		{
			I_SR_D[i] += I_SR[(i+j)*N_seg_per_unit][0] + (N_seg_per_unit >= 2)*I_SR[(i+j)*N_seg_per_unit + 1][0] + (N_seg_per_unit >= 3)*I_SR[(i+j)*N_seg_per_unit + 2][0]  + (N_seg_per_unit >= 4)*I_SR[(i+j)*N_seg_per_unit + 3][0];
			I_SR_V[i] += I_SR[(i+j)*N_seg_per_unit][1] + (N_seg_per_unit >= 2)*I_SR[(i+j)*N_seg_per_unit + 1][1] + (N_seg_per_unit >= 3)*I_SR[(i+j)*N_seg_per_unit + 2][1]  + (N_seg_per_unit >= 4)*I_SR[(i+j)*N_seg_per_unit + 3][1];
		}
	}

	// For units near the tail, fewer segments contribute (because the body ends)
	int tmp_N_SR = N_SR;
	for(int i = (N_units - N_SR + 1); i < N_units; ++i)
	{
		tmp_N_SR --;
		I_SR_D[i] = 0.0;
		I_SR_V[i] = 0.0;
		for(int j = 0; j < tmp_N_SR; ++j)
		{
			I_SR_D[i] += I_SR[(i+j)*N_seg_per_unit][0] + (N_seg_per_unit >= 2)*I_SR[(i+j)*N_seg_per_unit + 1][0] + (N_seg_per_unit >= 3)*I_SR[(i+j)*N_seg_per_unit + 2][0]  + (N_seg_per_unit >= 4)*I_SR[(i+j)*N_seg_per_unit + 3][0];
			I_SR_V[i] += I_SR[(i+j)*N_seg_per_unit][1] + (N_seg_per_unit >= 2)*I_SR[(i+j)*N_seg_per_unit + 1][1] + (N_seg_per_unit >= 3)*I_SR[(i+j)*N_seg_per_unit + 2][1]  + (N_seg_per_unit >= 4)*I_SR[(i+j)*N_seg_per_unit + 3][1];
		}
	}

	// Compensate for the posterior segments with shorter processes
	for(int i = (N_units - N_SR + 1); i < N_units; ++i)
	{
		I_SR_D[i] *= sqrt(-(N_SR/(i-N_units)));
		I_SR_V[i] *= sqrt(-(N_SR/(i-N_units)));
	}

	// Current bias to compensate for the fact that neural inhibition only goes one way
    double I_bias = 0.8;

	// Combine AVB current, stretch receptor current, neural inhibition and bias
	for(int i = 0; i < N_units; ++i)
	{
		I_D[i] = I_on + SR_weight[i]*I_SR_D[i];
		I_V[i] = (I_bias - State[i][0]) + I_on + SR_weight[i]*I_SR_V[i];
	}

	// Add gap junction currents if they are being used (typically I_coupling = 0)
	I_D[0] += (State[1][0] - State[0][0])*I_coupling;
	I_V[0] += (State[1][1] - State[0][1])*I_coupling;

	for(int i = 1; i < N_units-1; ++i)
	{
		I_D[i] += ((State[i+1][0] - State[i][0]) + (State[i-1][0] - State[i][0]))*I_coupling;
		I_V[i] += ((State[i+1][1] - State[i][1]) + (State[i-1][1] - State[i][1]))*I_coupling;
	}

	I_D[N_units-1] += (State[N_units-2][0] - State[N_units-1][0])*I_coupling;
	I_V[N_units-1] += (State[N_units-2][1] - State[N_units-1][1])*I_coupling;

	// Update state for each bistable B-class neuron
	for(int i = 0; i < N_units; ++i)
	{
		if(I_D[i] > (0.5 + Hyst/2.0 - Hyst*State[i][0])){
			State[i][0] = 1;}
		else{
			State[i][0] = 0;}

		if(I_V[i] > (0.5 + Hyst/2.0 - Hyst*State[i][1])){
			State[i][1] = 1;}
		else{
			State[i][1] = 0;}
	}

	// Compute effective input to each muscle including B-class excitation and contralateral D-class inhibition
	for(int i = 0; i < NSEG; ++i)
	{
		V_neuron[i][0] = NMJ_weight[i]*State[(int)(i*N_units/NSEG)][0] - NMJ_weight[i]*State[(int)(i*N_units/NSEG)][1];
		V_neuron[i][1] = NMJ_weight[i]*State[(int)(i*N_units/NSEG)][1] - NMJ_weight[i]*State[(int)(i*N_units/NSEG)][0];
	}
}

// Update steering neurons.
void update_steering_neurons(double salt_stimulus)
{
	// Dorsal head swing complete?
    if (segment_angles[0] >= head_angle_tracker[0] &&
        head_angle_tracker[0] <= head_angle_tracker[1]) {
		if (dorsal_salt_stimulus >= 0.0 && ventral_salt_stimulus >= 0.0) {
            double d = ventral_salt_stimulus - salt_stimulus_marker;
            double v = dorsal_salt_stimulus - ventral_salt_stimulus;
            dorsal_smb_muscle_amplifier = 1.0;
            ventral_smb_muscle_amplifier = 1.0;
			if (smb_muscle_amplifier_control == NEUTRAL) {
                asel.activation = aser.activation = 0.0f;
				if (d < 0.0 && v < 0.0) {
                    aser_trigger_count++;
                    if (aser_trigger_count >= ASER_TRIGGER_THRESHOLD) {
                        smb_muscle_amplifier_control = VENTRAL;
                        aser.activation = ASER_ACTIVATION;
                    } else {
                        smb_muscle_amplifier_control = DORSAL;
                        asel.activation = ASEL_ACTIVATION;
                    }
				} else if (d > 0.0 && v > 0.0)
                {
                    aser_trigger_count = 0;
					if (d > v) {
						smb_muscle_amplifier_control = DORSAL;
					} else if (v > d) {
						smb_muscle_amplifier_control = VENTRAL;
					}
                    asel.activation = ASEL_ACTIVATION;
				} else {
                    aser_trigger_count = 0;
                }
                set_smb_muscle_amplification();
			} else {
                if (smb_muscle_amplifier_control == STABILIZE){
                    smb_muscle_amplifier_control = NEUTRAL;
                } else {
                    smb_muscle_amplifier_control = STABILIZE;
                }
			}
			ventral_salt_stimulus = -1.0;
            dorsal_salt_stimulus = -1.0;
		} else {
			dorsal_salt_stimulus = salt_stimulus_marker;
		}
    }

	// Ventral head swing complete?
    if (segment_angles[0] <= head_angle_tracker[0] &&
        head_angle_tracker[0] >= head_angle_tracker[1]) {
		if (dorsal_salt_stimulus >= 0.0 && ventral_salt_stimulus >= 0.0) {
            double v = dorsal_salt_stimulus - salt_stimulus_marker;
            double d = ventral_salt_stimulus - dorsal_salt_stimulus;
            dorsal_smb_muscle_amplifier = 1.0;
            ventral_smb_muscle_amplifier = 1.0;
			if (smb_muscle_amplifier_control == NEUTRAL) {
                asel.activation = aser.activation = 0.0f;
				if (d < 0.0 && v < 0.0) {
                    aser_trigger_count++;
                    if (aser_trigger_count >= ASER_TRIGGER_THRESHOLD) {
                        smb_muscle_amplifier_control = VENTRAL;
                        aser.activation = ASER_ACTIVATION;
                    } else {
                        smb_muscle_amplifier_control = DORSAL;
                        asel.activation = ASEL_ACTIVATION;
                    }
                } else if (d > 0.0 && v > 0.0)
                {
                    aser_trigger_count = 0;
					if (d > v) {
						smb_muscle_amplifier_control = DORSAL;
					} else if (v > d) {
						smb_muscle_amplifier_control = VENTRAL;
					}
                    asel.activation = ASEL_ACTIVATION;
                } else {
                    aser_trigger_count = 0;
                }
                set_smb_muscle_amplification();
			} else {
                if (smb_muscle_amplifier_control == STABILIZE){
                    smb_muscle_amplifier_control = NEUTRAL;
                } else {
                    smb_muscle_amplifier_control = STABILIZE;
                }
			}
			ventral_salt_stimulus = -1.0;
            dorsal_salt_stimulus = -1.0;
		} else {
			ventral_salt_stimulus = salt_stimulus_marker;
		}
    }
    salt_stimulus_marker = salt_stimulus;
    head_angle_tracker[1] = head_angle_tracker[0];
    head_angle_tracker[0] = segment_angles[0];
}

// Set SMB muscle amplification.
void set_smb_muscle_amplification() {
    dorsal_smb_muscle_amplifier = 1.0;
    ventral_smb_muscle_amplifier = 1.0;
    aiyl.activation = (asel.activation * asel0_weight) + (aser.activation * aser0_weight);
    aiyr.activation = (asel.activation * asel1_weight) + (aser.activation * aser1_weight);
    aizl.activation = aiyl.activation * aiyl0_weight;
    aizr.activation = aiyr.activation * aiyr0_weight;
    if (smb_muscle_amplifier_control == DORSAL) {
        dorsal_smb_muscle_amplifier =
                (aizl.activation * aizl0_weight) + (aizr.activation * aizr0_weight);
    } else if (smb_muscle_amplifier_control == VENTRAL) {
        ventral_smb_muscle_amplifier =
                (aizl.activation * aizl1_weight) + (aizr.activation * aizr1_weight);
    }

#ifdef BYPASS_INTERNEURONS
    dorsal_smb_muscle_amplifier = 1.0;
    ventral_smb_muscle_amplifier = 1.0;
    if (smb_muscle_amplifier_control == DORSAL) {
        if (asel.activation > 0.0) {
            dorsal_smb_muscle_amplifier = ASEL_SMB_MUSCLE_AMPLIFIER;
        } else if (aser.activation > 0.0) {
            dorsal_smb_muscle_amplifier = ASER_SMB_MUSCLE_AMPLIFIER;
        }
    } else if (smb_muscle_amplifier_control == VENTRAL) {
        if (asel.activation > 0.0) {
            ventral_smb_muscle_amplifier = ASEL_SMB_MUSCLE_AMPLIFIER;
        } else if (aser.activation > 0.0) {
            ventral_smb_muscle_amplifier = ASER_SMB_MUSCLE_AMPLIFIER;
        }
    }
#endif
}

// Update the stretch receptors (for each segment). These
// are weighted and combined as input to the neural units in function "update_neurons"
void update_SR()
{
	for(int i = 0; i < NSEG; ++i)
	{
		// Bilinear SR function on one side to compensate for asymmetry and help worm go straight
		I_SR[i][0] = SR_shape_compensation[i]*((L_SR[i][0] - L0_P[i])/L0_P[i]*((L_SR[i][0] > L_seg) ? 0.8:1.2));
		I_SR[i][1] = SR_shape_compensation[i] * ((L_SR[i][1] - L0_P[i]) / L0_P[i]);
	}
}

// Update the simple muscle "model" (electronic)
void update_muscles()
{
	//Muscle transfer function is just a simple LPF
	for(int i = 0; i < NSEG; ++i)
	{
		for(int j = 0; j < 2; ++j)
		{
			V_muscle[i][j] = V_muscle[i][j];
			realtype dV = (V_neuron[i][j] - V_muscle[i][j])/T_muscle;
			V_muscle[i][j] += dV*DELTAT;
			dV = (V_neuron[i][j] - V_muscle[i][j]) / T_muscle;
			V_muscle[i][j] += dV*DELTAT;
		}
	}

    // Publish muscle activations.
    for (int i = 0; i < NSEG; i += 4)
    {
        double d = 0.0;
        double v = 0.0;
        for (int j = 0; j < 4; j++) {
            d += V_muscle[i + j][0];
            v += V_muscle[i + j][1];
        }
        d /= 4.0;
        v /= 4.0;
        dorsal_muscle_activations[i / 4] = d;
        ventral_muscle_activations[i / 4] = v;
    }
}

// System residual function which implements physical model (Based on Sundials examples)
int resrob(realtype tres, N_Vector yy, N_Vector yp, N_Vector rr, void *rdata)
{
	// Import data from vectors
	realtype *yval, *ypval, *rval;
	yval = NV_DATA_S(yy);
	ypval = NV_DATA_S(yp);
	rval = NV_DATA_S(rr);

	//Declare variables
	realtype CoM[NBAR][3];
	realtype V_CoM[NBAR][3];
	realtype term[NBAR][2][2];  		// Nseg, d/v, x/y
	realtype V_term[NBAR][2][2];
	realtype dy,dx,dVy,dVx,F_even,F_odd;
	realtype F_term[NBAR][2][2];
	realtype F_term_rotated[NBAR][2][2];
	realtype V_CoM_rotated[NBAR][3];

	realtype L[NSEG][2];				// Nseg, d/v
	realtype Dir[NSEG][2][2];			// Nseg, d/v, x/y
	realtype S[NSEG][2];
	realtype L_D[NSEG][2];			// Nseg, \,/   <- these are the angles of the diagonals
	realtype Dir_D[NSEG][2][2];  			// Nseg, \,/ , x/y
	realtype S_D[NSEG][2];

	realtype L0_AE, T, F_AE, F_PE, F_PD;
	realtype F_H[NSEG][2];
	realtype F_D[NSEG][2];

	realtype L_EXT;
	realtype F_EXT[2];

	for(int i = 0; i < NBAR; ++i)
	{
		// Extract CoM of each solid rod from vectors
		int three_i = i*3;
		CoM[i][0] = yval[three_i];
		CoM[i][1] = yval[three_i + 1];
		CoM[i][2] = yval[three_i + 2];

		// Calculate positions of D/V points based on CoM, angle and radius
		dx = R[i]*cos(CoM[i][2]);
		dy = R[i]*sin(CoM[i][2]);

		term[i][0][0] = CoM[i][0] + dx;
		term[i][0][1] = CoM[i][1] + dy;
		term[i][1][0] = CoM[i][0] - dx;
		term[i][1][1] = CoM[i][1] - dy;

		// Extract CoM velocities of each solid rod from vectors
		V_CoM[i][0] = ypval[three_i];
		V_CoM[i][1] = ypval[three_i + 1];
		V_CoM[i][2] = ypval[three_i + 2];

		// Calculate velocity of D/V points based on CoM velocity, rate of rotation and radius
		realtype V_arm = R[i]*V_CoM[i][2];
		dVx = V_arm*cos(CoM[i][2] + HALFPI);
		dVy = V_arm*sin(CoM[i][2] + HALFPI);

		V_term[i][0][0] = V_CoM[i][0] + dVx;
		V_term[i][0][1] = V_CoM[i][1] + dVy;
		V_term[i][1][0] = V_CoM[i][0] - dVx;
		V_term[i][1][1] = V_CoM[i][1] - dVy;
	}


	// Get Horizontal/Diagonal element lengths and lengthening/shortening velocities
	for(int i = 0; i < NSEG; ++i)
	{
		// Strange format for efficiency
		int iplus1 = i+1;

		Dir[i][0][0] = (term[iplus1][0][0] - term[i][0][0]);
		Dir[i][0][1] = (term[iplus1][0][1] - term[i][0][1]);
		L[i][0] = sqrt(pow(Dir[i][0][0],2.0) + pow(Dir[i][0][1],2.0));
		Dir[i][0][0] /= L[i][0];
		Dir[i][0][1] /= L[i][0];
		S[i][0] = (V_term[iplus1][0][0] - V_term[i][0][0])*Dir[i][0][0] + (V_term[iplus1][0][1] - V_term[i][0][1])*Dir[i][0][1];

		Dir[i][1][0] =  (term[iplus1][1][0] - term[i][1][0]);
		Dir[i][1][1] =  (term[iplus1][1][1] - term[i][1][1]);
		L[i][1] = sqrt(pow(Dir[i][1][0],2.0) + pow(Dir[i][1][1],2.0));
		Dir[i][1][0] /= L[i][1];
		Dir[i][1][1] /= L[i][1];
		S[i][1] = (V_term[iplus1][1][0] - V_term[i][1][0])*Dir[i][1][0] + (V_term[iplus1][1][1] - V_term[i][1][1])*Dir[i][1][1];

		Dir_D[i][0][0] =  (term[iplus1][1][0] - term[i][0][0]);
		Dir_D[i][0][1] =  (term[iplus1][1][1] - term[i][0][1]);
		L_D[i][0] = sqrt(pow(Dir_D[i][0][0],2.0) + pow(Dir_D[i][0][1],2.0));
		Dir_D[i][0][0] /= L_D[i][0];
		Dir_D[i][0][1] /= L_D[i][0];
		S_D[i][0] = (V_term[iplus1][1][0] - V_term[i][0][0])*Dir_D[i][0][0] + (V_term[iplus1][1][1] - V_term[i][0][1])*Dir_D[i][0][1];

		Dir_D[i][1][0] =  (term[iplus1][0][0] - term[i][1][0]);
		Dir_D[i][1][1] =  (term[iplus1][0][1] - term[i][1][1]);
		L_D[i][1] = sqrt(pow(Dir_D[i][1][0],2.0) + pow(Dir_D[i][1][1],2.0));
		Dir_D[i][1][0] /= L_D[i][1];
		Dir_D[i][1][1] /= L_D[i][1];
		S_D[i][1] = (V_term[iplus1][0][0] - V_term[i][1][0])*Dir_D[i][1][0] + (V_term[iplus1][0][1] - V_term[i][1][1])*Dir_D[i][1][1];

		// Calculate force contributions on each D/V point

		//Dorsal forces due to horizontal elements
		L0_AE = L0_P[i] - fmax(V_muscle[i][0],0)*(L0_P_minus_L_min[i]);

		F_AE = k_AE*fmax(V_muscle[i][0],0)*(L0_AE - L[i][0]);
		F_PE = k_PE*((L0_P[i] - L[i][0]) + ((L[i][0]-L0_P[i]) > RCONST(0.0))*pow(RCONST(2.0)*(L[i][0]-L0_P[i]),4));
		F_PD = (D_PE + fmax(V_muscle[i][0],0)*D_AE)*S[i][0];

		F_H[i][0] = F_PE + F_AE - F_PD;

		//Ventral forces due to horizontal elements
		L0_AE = L0_P[i] - fmax(V_muscle[i][1],0)*(L0_P_minus_L_min[i]);

		F_AE = k_AE*fmax(V_muscle[i][1],0)*(L0_AE - L[i][1]);
		F_PE = k_PE*((L0_P[i] - L[i][1]) + ((L[i][1]-L0_P[i]) > RCONST(0.0))*pow(RCONST(2.0)*(L[i][1]-L0_P[i]),4));
		F_PD = (D_PE + fmax(V_muscle[i][1],0)*D_AE)*S[i][1];

		F_H[i][1] = F_PE + F_AE - F_PD;

		//Diagonal forces due to diagonal elements
		F_D[i][0] = (L0_D[i] - L_D[i][0])*k_DE - D_DE*S_D[i][0];
		F_D[i][1] = (L0_D[i] - L_D[i][1])*k_DE - D_DE*S_D[i][1];
	}

	// Add up force contributions for each D/V point
	F_term[0][0][0] = -F_H[0][0]*Dir[0][0][0] - F_D[0][0]*Dir_D[0][0][0];
	F_term[0][0][1] = -F_H[0][0]*Dir[0][0][1] - F_D[0][0]*Dir_D[0][0][1];

	F_term[0][1][0] = -F_H[0][1]*Dir[0][1][0] - F_D[0][1]*Dir_D[0][1][0];
	F_term[0][1][1] = -F_H[0][1]*Dir[0][1][1] - F_D[0][1]*Dir_D[0][1][1];

	for(int i = 1; i < NSEG; ++i)
	{
		int i_minus_1 = i-1;

		F_term[i][0][0] = F_H[i_minus_1][0]*Dir[i_minus_1][0][0] - F_H[i][0]*Dir[i][0][0] + F_D[i_minus_1][1]*Dir_D[i_minus_1][1][0] - F_D[i][0]*Dir_D[i][0][0];
		F_term[i][0][1] = F_H[i_minus_1][0]*Dir[i_minus_1][0][1] - F_H[i][0]*Dir[i][0][1] + F_D[i_minus_1][1]*Dir_D[i_minus_1][1][1] - F_D[i][0]*Dir_D[i][0][1];

		F_term[i][1][0] = F_H[i_minus_1][1]*Dir[i_minus_1][1][0] - F_H[i][1]*Dir[i][1][0] + F_D[i_minus_1][0]*Dir_D[i_minus_1][0][0] - F_D[i][1]*Dir_D[i][1][0];
		F_term[i][1][1] = F_H[i_minus_1][1]*Dir[i_minus_1][1][1] - F_H[i][1]*Dir[i][1][1] + F_D[i_minus_1][0]*Dir_D[i_minus_1][0][1] - F_D[i][1]*Dir_D[i][1][1];
	}

	F_term[NSEG][0][0] = F_H[NSEG_MINUS_1][0]*Dir[NSEG_MINUS_1][0][0] + F_D[NSEG_MINUS_1][1]*Dir_D[NSEG_MINUS_1][1][0];
	F_term[NSEG][0][1] = F_H[NSEG_MINUS_1][0]*Dir[NSEG_MINUS_1][0][1] + F_D[NSEG_MINUS_1][1]*Dir_D[NSEG_MINUS_1][1][1];

	F_term[NSEG][1][0] = F_H[NSEG_MINUS_1][1]*Dir[NSEG_MINUS_1][1][0] + F_D[NSEG_MINUS_1][0]*Dir_D[NSEG_MINUS_1][0][0];
	F_term[NSEG][1][1] = F_H[NSEG_MINUS_1][1]*Dir[NSEG_MINUS_1][1][1] + F_D[NSEG_MINUS_1][0]*Dir_D[NSEG_MINUS_1][0][1];

	// Convert net forces on D/V points to force and torque	acting on rod CoM
	for(int i = 0; i < NBAR; ++i)
	{
		realtype cos_thi = cos(CoM[i][2]);
		realtype sin_thi = sin(CoM[i][2]);
		for(int j = 0; j < 2; ++j)
		{
			F_term_rotated[i][j][0] = F_term[i][j][0]*cos_thi + F_term[i][j][1]*sin_thi;	// This is Fperp
			F_term_rotated[i][j][1] = F_term[i][j][0]*sin_thi - F_term[i][j][1]*cos_thi;    // THis is Fparallel
		}

        // Incorporate steering.
        if (i == 0) {
            F_term_rotated[i][0][0] *= dorsal_smb_muscle_amplifier;
            F_term_rotated[i][0][1] *= dorsal_smb_muscle_amplifier;
            F_term_rotated[i][1][0] *= ventral_smb_muscle_amplifier;
            F_term_rotated[i][1][1] *= ventral_smb_muscle_amplifier;
        }

		V_CoM_rotated[i][0] = (F_term_rotated[i][0][0] + F_term_rotated[i][1][0])/CN[i];

		F_even = (F_term_rotated[i][0][1] + F_term_rotated[i][1][1]);	//Took out the /2
		F_odd = (F_term_rotated[i][1][1] - F_term_rotated[i][0][1])/RCONST(2.0);

		V_CoM_rotated[i][1] = (F_even)/CL[i];				//Allowing me to take out *2
		V_CoM[i][2] = (F_odd/CL[i])/(M_PI*2.0*R[i]);

		V_CoM[i][0] = V_CoM_rotated[i][0]*cos_thi + V_CoM_rotated[i][1]*sin_thi;
		V_CoM[i][1] = V_CoM_rotated[i][0]*sin_thi - V_CoM_rotated[i][1]*cos_thi;

		int three_i = i*3;

		rval[three_i] = V_CoM[i][0] - ypval[three_i];
		rval[three_i+1] = V_CoM[i][1] - ypval[three_i+1];
		rval[three_i+2] = V_CoM[i][2] - ypval[three_i+2];
	}

	// Store old lengths for Stretch Receptors
	for(int i = 0; i < NSEG; ++i)
	{
		L_SR[i][0] = L[i][0];
		L_SR[i][1] = L[i][1];
	}

	return(0);
}

/*
 *--------------------------------------------------------------------
 * Private functions
 *--------------------------------------------------------------------
 */

/*
 * Check function return value... (Copied from Sundials examples)
 *   opt == 0 means SUNDIALS function allocates memory so check if
 *            returned NULL pointer
 *   opt == 1 means SUNDIALS function returns a flag so check if
 *            flag >= 0
 *   opt == 2 means function allocates memory so check if returned
 *            NULL pointer 
 */

static int check_flag(void *flagvalue, char *funcname, int opt)
{
	int *errflag;
	/* Check if SUNDIALS function returned NULL pointer - no memory allocated */
	if (opt == 0 && flagvalue == NULL)
	{
		fprintf(stderr,
				"\nSUNDIALS_ERROR: %s() failed - returned NULL pointer\n\n",
				funcname);
		return(1);
	} else if (opt == 1)
	{
		/* Check if flag < 0 */
		errflag = (int *) flagvalue;
		if (*errflag < 0)
		{
			fprintf(stderr,
					"\nSUNDIALS_ERROR: %s() failed with flag = %d\n\n",
					funcname, *errflag);
			return(1);
		}
	} else if (opt == 2 && flagvalue == NULL)
	{
		/* Check if function returned NULL pointer - no memory allocated */
		fprintf(stderr,
				"\nMEMORY_ERROR: %s() failed - returned NULL pointer\n\n",
				funcname);
		return(1);
	}

	return(0);
}

// Rotate a point by a given angle.
void rotatePoint(realtype *x, realtype *y, realtype angle)
{
	realtype l = sqrt((*x * *x) + (*y * *y));
	realtype a = 0.0;

	if (*x == 0.0)
	{
		if (*y > 0.0)
		{
			a = M_PI * .5;
		}
		else
		{
			a = M_PI * 1.5;
		}
	}
	else
	{
		a = atan(*y / *x);
		if (*x > 0.0)
		{
			if (*y < 0.0)
			{
				a += M_PI * 2.0;
			}
		}
		else
		{
			a += M_PI;
		}
	}
	a += angle;
	*x = l * cos(a);
	*y = l * sin(a);
}

