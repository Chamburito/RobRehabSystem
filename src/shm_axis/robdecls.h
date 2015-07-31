// robdecls.h - data declarations for the InMotion2 robot software system
//

// InMotion2 robot system software for RTLinux

// Copyright 2003-2005 Interactive Motion Technologies, Inc.
// Cambridge, MA, USA
// http://www.interactive-motion.com
// All rights reserved

#ifndef ROBDECLS_H
#define ROBDECLS_H

#include "ruser.h"
#include <math.h>

// close to zero, for double compares.
#define EPSILON 0.0000001

#define ROBOT_LOOP_THREAD_NAME "IMT Robot Control Loop"

#define ARRAY_SIZE(a) (sizeof (a) / sizeof ((a)[0]))
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) < (b) ? (a) : (b))
// u8/s8 etc. are defined in types.h

// I tripped over an f32, so I'm removing them
// typedef float f32;
#define f32 woops!
typedef double f64;

#define FIFOLEN 0x4000

// the Refbuf buffer will store 4 minutes of refs,
// each ref row will have 5 items
#define REFARR_COLS 5
#define REFARR_ROWS (200*60*4)

// errors, see main.c and uei.c
// if you change this, you MUST change tools/errpt

enum {
	ERR_NONE=0,
	ERR_MAIN_LATE_TICK,
	WARN_MAIN_SLOW_TICK,
	WARN_MAIN_SLOW_SAMPLE,
	ERR_UEI_NSAMPLES,
	ERR_UEI_RET,
	ERR_UEI_BOARD_RANGE,
	ERR_UEI_BAD_ARRAY_PTRS,
	WARN_MAIN_FAST_TICK,
	ERR_AN_HIT_STOPS,
	ERR_AN_SHAFT_SLIP_LEFT,
	ERR_AN_SHAFT_SLIP_RIGHT,
	ERR_PL_ENC_KICK,
	ERR_LAST
};

// math

// 2d
typedef struct xy_s {
	f64 x;
	f64 y;
} xy;

// 3d
typedef struct xyz_s {
	f64 x;
	f64 y;
	f64 z;
} xyz;

// shoulder/elbow pair
typedef struct se_s {
	f64 s;
	f64 e;
} se;

// 2x2 matrix
typedef struct mat22_s {
    f64 e00;
    f64 e01;
    f64 e10;
    f64 e11;
} mat22;

#ifdef NOTDEF
// 2x2 box center/w/h
typedef struct box_cwh_s {
	xy c;
	xy wh;
} box_cwh;


// 2x2 box x1y1/x2y2
typedef struct box_xy_s {
	xy p1;
	xy p2;
} box_xy;
#endif // NOTDEF

// performance metrics
typedef struct pm_s {
    f64 active_power;       // pm2a
    f64 robot_power;       // pm2a
    f64 min_jerk_deviation; // pm2b
    f64 min_jerk_dgraph; // graph
    f64 dist_straight_line; // pm3
    f64 max_dist_along_axis; // pm4
    u32 npoints;            // npts
    u32 five_d;             // planarwrist adaptive
} PM;

// wrist types
typedef struct wrist_dof_s {
        f64 fe;	// flexion/extension
        f64 aa;	// abduction/adduction
        f64 ps;	// pronation/supination
} wrist_dof;

typedef struct wrist_ob_s {    // world coordinate parameters
    wrist_dof pos;	// position
    wrist_dof vel;	// velocity
    wrist_dof fvel;	// filtered velocity
    wrist_dof torque;	// command torque
    wrist_dof offset;   // offset from zero;
    wrist_dof moment_csen;
    wrist_dof moment_cmd;
    wrist_dof back;	// back wall for adap
    wrist_dof norm;	// normalized posn for adap
    wrist_dof ref_pos;	// for ref control
    f64 diff_stiff;	// stiffness
    f64 ps_stiff;	// stiffness
    f64 diff_side_stiff;	// stiffness
    f64 diff_damp;	// damping
    f64 ps_damp;	// damping
    f64 diff_gcomp;	// gravity compensation
    f64 ps_gcomp;	// gravity compensation
    u32 ps_adap_going_up; // adaptive going up
    u32 nocenter3d;	// don't center the uncontrolled dof
    f64 velmag;		// velocity magnitude
    f64 rl_pfomax;	// preserve force orientation
    f64 rl_pfotest;	// preserve force orientation
    u32 ft_motor_force;	// use motor force instead of ft
} wrist_ob;


// right/left dof
typedef struct rl_s {
    f64 r;
    f64 l;
} rl;

// ankle types

typedef struct ankle_dof_s {	// ankle degrees of freedom
        f64 dp; // dorsiflexion/plantarflexion
        f64 ie; // inversion/eversion
} ankle_dof;

typedef struct ankle_mattr_s {	// ankle motor attributes
    u32 enc_channel;
    f64 disp;
    f64 devtrq;
    f64 xform;
    f64 volts;
    f64 test_volts;
    f64 force;
    u32 ao_channel;
    u32 csen_channel;
    u32 rot_enc_channel;
    f64 rot_disp;
    f64 rot_lin_disp;
    f64 rot_lin_vel;
    f64 vel;
} ankle_MAttr;

typedef struct ankle_trans_s {	// ankle gear ratios from motor to world
    f64 ratio;
    f64 lead;
    f64 ankle_ball_length;
    f64 ball_ball_width;
    f64 av_shin_length;
    f64 av_actuator_length;
    f64 enc_xform;
    f64 slip_thresh;
} ankle_trans;

typedef struct ankle_knee_s {
        u32 channel;
        f64 raw;
        // xform1 * potvoltage^2 + xform2 * potvoltage + bias
        f64 xform1;
        f64 xform2;
        f64 bias;
        f64 angle;
} ankle_knee;

typedef struct ankle_s {	// group the parameters of each motor and
			// include an overall gear ratio for the differential
    ankle_MAttr left;
    ankle_MAttr right;
    ankle_trans trans;
    ankle_knee knee;
    u32 uei_ao_board_handle;
} Ankle;

typedef struct ankle_ob_s {	// ankle world coordinates
    ankle_dof pos;
    ankle_dof vel;
    ankle_dof fvel;
    ankle_dof torque;
    ankle_dof back;
    ankle_dof norm;
    ankle_dof offset;
    ankle_dof ref_pos;
    ankle_dof moment_csen;
    ankle_dof moment_cmd;
    ankle_dof accel;
    f64 vel_mag;
    f64 accel_mag;
    f64 safety_vel;
    f64 safety_accel;
    f64 stiff;
    f64 damp;
    f64 rl_pfomax;
    f64 rl_pfotest;
    u32 ueimf;
    f64 ankle_games_trgt;
    f64 ankle_games_P1;
    f64 ankle_games_P2;
    f64 ankle_games_P3;
    f64 ankle_games_P4;
    f64 ankle_games_speed;
    f64 ankle_games_gate_size;
    f64 ankle_games_splattime;
    f64 ankle_games_score;
    f64 ankle_games_paddle_size;
    f64 ankle_games_hdir; 
    f64 ankle_games_opponent_score; 
    f64 ankle_games_opponent_speed; 
    f64 ankle_games_wait_time_ie; 
    f64 ankle_games_wait_time_dp; 
} ankle_ob;

typedef struct ankle_prev_s {	// previous ankle world coordinate parameters
    ankle_dof pos;
    ankle_dof vel;
    ankle_dof fvel;
    rl rot_lin_disp;
    rl disp;
} ankle_prev;

// linear types
typedef struct linear_ob_s {    // linear world coordinates
    f64 pos;
    f64 vel;
    f64 fvel;
    f64 force;
    f64 back;	// back wall for adap
    f64 norm;	// normalized posn for adap
    f64 offset;
    f64 ref_pos;
    f64 stiff;
    f64 damp;
    f64 force_bias;
    f64 pfomax;		// pfo
    f64 pfotest;	// pfo
    u32 adap_going_up;
} linear_ob;

// hand types
typedef struct hand_ob_s {    // hand world coordinates
    f64 pos;
    f64 vel;
    f64 fvel;
    f64 force;
    f64 grasp;
    f64 offset;
    f64 ref_pos;
    f64 stiff;
    f64 damp;
    f64 force_bias;
    f64 pfomax;		// pfo
    f64 pfotest;	// pfo
    u32 adap_going_up;
    u32 active_power;
    u32 npoints;
} hand_ob;

//Junction Box Buffer for EMG amplitude estimation
typedef struct emg_buffer_s {
	f64 *buff_array;   //Array for buffer- construct upon initialization
	f64 *buff_target_chan;  //Points to target DAQ channel
	f64 buff_new_val;  //Incoming value
	f64 buff_old_val;  //Value to be discarded
	u32 buff_length;   //Stores length of buff_array
	u32 buff_last_el;  //Points to element to be discarded
	u32 buff_first_el; //Points to element most recently added
	f64 buff_amp_est;  //Amplitude estimate of this channel
        f64 buff_bias;     //DC bias of this channel
	u32 buff_active;   //Is buffer active?
} EMG_buffer;

// time vars

typedef struct time_s {
    hrtime_t time_at_start;
    hrtime_t time_before_sample;
    hrtime_t time_after_sample;
    hrtime_t time_before_last_sample;
    hrtime_t time_after_last_sample;
    hrtime_t time_delta_sample;
    hrtime_t time_delta_tick;
    hrtime_t time_delta_call;
    hrtime_t time_since_start;

    u32 ns_delta_call;
    u32 ns_delta_tick;
    u32 ns_delta_tick_thresh;
    u32 ns_delta_sample;
    u32 ns_delta_sample_thresh;
    u32 ms_since_start;
    u32 sec;

    u32 ns_max_delta_tick;
    u32 ns_max_delta_sample;
} Timev;

// data that gets copied to ob atomically, when you set go

typedef struct restart_s {
    u32 go;			// write this last!
    u32 Hz;			// when you're ready to go,
    u32 ovsample;
    f64 stiff;			// copy all this to ob.
    f64 damp;
} Restart;

typedef struct ref_s {
    xy pos;
    xy vel;
} Ref;

typedef struct max_s {
    xy vel;
    xy motor_force;
    se motor_torque;
} Max;

// this needs to be extended for > 2d position descriptions

typedef struct box_s {
    xy point;
    f64 w;	// width
    f64 h;	// height
} box;

typedef struct slot_s {
    u32 id;			// id (for copy_slot)

    // a la for loop
    u32 i;			// the incremented index
    u32 incr;			// amount to increment each sample
    u32 term;			// termination
    u32 termi;			// index incremented after termination
    				// for making controls stiffer, etc.

    box b0;			// initial position
    box b1;			// final position
    box bcur;			// current position

    f64 rot;			// slot rotation in radians
    u32 fnid;			// index into slot fn * table
    u32 running;		// this slot is running
    u32 go;			// go (for copy_slot)
} Slot;

typedef struct pos_error_s {
    u32 mod;
    f64 dx;
    f64 dy;
} Pos_error;

// safety envelope
typedef struct safety_s {
	f64 pos;
	f64 vel;
	f64 torque;
	f64 ramp;
	f64 damping_nms;
	f64 velmag_kick;
	u32 override;

	u32 planar_just_crossed_back;
	u32 was_planar_damping;
	u32 damp_ret_ticks;
	f64 damp_ret_secs;
} Safety;

typedef struct sim_s {
    u32 sensors;		// boolean, yes, simulate
    xy pos;			// position, read from user space
    xy vel;			// vel
    wrist_dof wr_pos;           // wrist position
    wrist_dof wr_vel;           // wrist velocity
} Sim;

// the main object struct

typedef struct ob_s {
    s8 tag[8];			// unused
    /* TODO: delete pthread_t main_thread; */	// main thread (currently only thread)
    //RT_TASK main_thread;

    u32 doinit;			// run init code if set.
    u32 didinit;		// so we only init once
    u32 quit;                   // quit program if set.

    u32 i;			// loop iteration number.
    u32 fasti;			// fast tick count for ft
    u32 samplenum;		// sample number.

    u32 total_samples;		// stop after this many

    u32 Hz;			// samples per second
    				// following are derived.
    f64 rate;			// 1.0/(samples per second),
    u32 irate;			// BILLION/(samples per second),
    				// i.e. nanoseconds per sample

				// for oversampled ft sampling
    u32 ovsample;		// oversampling multiplier
    				// set ovsample, the following are derived.
    u32 fastHz;			// fast samples per second
    f64 fastrate;		// 1.0/(samples per second),
    u32 fastirate;		// BILLION/(samples per second),
    				// i.e. nanoseconds per sample

    u32 ticks30Hz;		// number of samplenum ticks in 30Hz,
    				// for display stuff

    f64 stiff;			// stiffness for controller
    f64 damp;			// damping
    f64 curl;			// curl

    xy const_force;		// constant force control

    f64 side_stiff;		// side stiffness for adapative controller

    f64 pfomax;			// preserve force orientation max
    f64 pfotest;		// pfo test value

    se dvolts;			// impulse threshold delta volts
    f64 impulse_thresh_volts;	// impulse threshold cutoff

    u32 busy;			// sample is not in sleep wait.
    u32 paused;			// tick clock, but don't write actuators
    u32 fault;			// control loop triggered fault
    s32 stiffener;		// a stiffness % increment
				// 0 is normal stiffness
				// -100 is no stiffness
				// 100 is double, 200 is triple
			        // generalizing slot_term_stiffen
    s32 stiff_delta;		// how much to add to stiffener each tick
    u32 no_motors;		// never write torques

    Restart restart;		// copied into Ob on restart.

    Timev times;		// timing vars

    u32 have_tach;		// we have a tachometer
    u32 have_ft;		// we have a non-ISA force transducer
    u32 have_isaft;		// we have an ISA force transducer
    u32 have_accel;		// we have an acceleromter
    u32 have_grasp;		// we have a grasp sensor
    u32 have_planar;		// we have a planar robot
    u32 have_wrist;		// we have a wrist robot
    u32 have_ankle;		// we have an ankle robot
    u32 have_linear;		// we have an linear robot
    u32 have_hand;		// we have a hand robot
    u32 have_planar_incenc;	// we have a planar with incr encoders 
    u32 have_planar_ao8;	// we have a planar with ao8 output
    u32 have_mf_aout_for_dout;	// we use the mf aouts for douts
    u32 have_thermal_model;	// we have thermal model calcs

    Slot copy_slot;		// for input from shm
    Slot slot[8];		// slot control
    Pos_error pos_error;	// inject position errors
    PM pm;			// performance metrics for adaptive
    void (*slot_fns[32])(u32); 	// array of slot functions

    u32 slot_max;		// max number of slots;

    f64 pi;			// pi (make sure trig works)

    u32 nlog;			// number of items to write out
    f64 log[32];		// array of items to write out
    void (*log_fns[32])(void); 	// array of log functions
    u32 logfnid;		// which log function in array

    u32 ndisp;			// number of items to write out
    f64 disp[32];		// array of items to write out

    Ref ref;			// references for logfile playback

    u32 refri;			// refarr read index
    u32 refwi;			// refarr write index
    u32 refterm;		// refarr index of last entry

    u32 nwref;			// number of items write to the ref buf
    u32 nrref;			// number of items to read from the ref buf

    f64 refin[32];		// array of items to read
    void (*ref_fns[32])(void); 	// array of ref functions
    u32 reffnid;		// which log function in array
    u32 ref_switchback_go;	// run the switchback fn

    //RT_PIPE dififo;	       	// data in (like stdin)
    //RT_PIPE dofifo;	       	// data out (stdout)
    //RT_PIPE eofifo;	       	// error out (stderr)
    //RT_PIPE cififo;	       	// command in
    //RT_PIPE ddfifo;	       	// display data out
    //RT_PIPE tcfifo;             // tick data out
    // TODO: delete s32 nfifos;			// number of fifos

    u32 ntickfifo;		// do the tcfifo output

    u32 fifolen;		// fifo buffer size
    s8 *ci_fifo_buffer;		// pointer to command input fifo buffer
    				// (handled differently from other fifos)

    Safety safety;		// safety zone variables

    u32 vibrate;		// random vibration factor for testing
    s32 xvibe;			// vibration components
    s32 yvibe;

    xy pos;			// world position
    xy tach_vel;		// world velocities from hardware tach
    xy ftach_vel;		// filtered tach_vel
    xy soft_vel;		// world velocities from position
    xy fsoft_vel;		// filtered soft_vel
    xy vel;			// assigned from one of the vels
    xy motor_force;		// world forces sent to motors, from controller
    se motor_torque;		// device torques from motor_force
    se motor_volts;		// device volts from motor_torque
    xy back;			// back wall for adap
    xy norm;			// normalized posn for adap

    f64 velmag;			// magnitude of the velocity

    xy soft_accel;		// accel derived from position
    f64 soft_accelmag;		// accel magnitude

    se theta;			// encoder angles
    se thetadot;		// angular velocity
    se fthetadot;		// filtered thetadot

    u32 planar_uei_ao_board_handle; // handle of the ao8 board

    wrist_ob wrist;		// world coordinates
    ankle_ob ankle;
    linear_ob linear;
    hand_ob hand;

    u32 test_raw_torque;	// raw torque test mode;
    se raw_torque_volts;	// raw volts for testing
    u32 test_no_torque;		// don't write torques at all for testing

    Sim sim;			// simulate sensors

    f64 sin_period;		// for the sinewave generator
    f64 sin_amplitude;		// amplitude
    u32 sin_which_motor;	// shoulder, elbow, neither, both

    u32 butcutoff;		// butterworth cutoff W(n) * 100
    				// for 200 Hz, 15Hz cutoff, use 15
    				// (2 * 15 Hz cutoff) / 200 Hz

    Max max;			// maxima

    				// see main:do_error() and calls to it.
    u32 errnum;			// error this sample (couldn't call it errno)
    u32 nerrors;		// cumulative number of errors
    u32 errori[128];
    u32 errorcode[128];
    u32 errorindex;		// index into error arrays

    // scr is for random debugging
    f64 scr[64];		// scratch registers
    // game is for when programs game processes to communicate
    // with each other while the lkm is loaded.
    f64 game[64];		// game registers
    f64 aodiff[16];		// for ao8 testing
    f64 aocum[16];
    f64 aorms[16];
    s32 aocount;

    s32 debug_level;		// for dpr

    u32 last_shm_val;		// sanity check


#define JUNCTINPUTS 8
#define BUFFLEN 40

    EMG_buffer channelest[JUNCTINPUTS]; //Eight buffers for junction box amplitude est.
    f64 bufferarrays[JUNCTINPUTS][BUFFLEN];
    f64 buffscale;
    f64 display_vars_ref[JUNCTINPUTS];
    u32 buff_commands[JUNCTINPUTS];

    f64 ankle_games_trgt;
    f64 ankle_games_P1;
    f64 ankle_games_P2;
    f64 ankle_games_P3;
    f64 ankle_games_P4;
    f64 ankle_games_speed;
    f64 ankle_games_gate_size;
    f64 ankle_games_splattime;
    f64 ankle_games_score;
    f64 ankle_games_paddle_size;
    f64 ankle_games_hdir;
    f64 ankle_games_opponent_score;
    f64 ankle_games_opponent_speed;  
    f64 ankle_games_wait_time_ie; 
    f64 ankle_games_wait_time_dp;  
} Ob;


// 0x494D5431 is 'IMT1'
#define OB_KEY   0x494D5431
/*#define ROB_KEY  0x494D5432
#define DAQ_KEY  0x494D5433
#define PREV_KEY 0x494D5434
#define GAME_KEY 0x494D5435
#define REFBUF_KEY 0x494D5436
#define SHAREDBUFFER_KEY 0x494D5437*/

void check_safety_fn(void);
void planar_check_safety_fn(void);
void wrist_check_safety_fn(void);

void user_init(void);

// these call the _fn functions that the user sets up in user_init
void call_read_sensors(void);
void call_read_reference(void);
void call_compute_controls(void);
void call_check_safety(void);
void call_write_actuators(void);
void call_write_log(void);
void call_write_display(void);

// slot.c
// void load_slot(u32, u32, u32, u32, void (*)(u32), s8 *);
void load_slot(u32, u32, u32, u32, u32, s8 *);
void do_slot(void);
void stop_slot(u32);
void stop_all_slots(void);

// main.c
void do_time_before_sample(void);
void handle_fifo_input(void);
void check_quit(void);
void do_error(u32);
void check_late(void);
void refarr_switchback(void);
void read_sensors_fn(void);
void clear_sensors(void);
void print_sample_times(void);
void do_time_after_sample(void);
void wait_for_tick(void);

void write_to_refbuf(void);
void refbuf_to_refin(void);
void planar_write_to_refbuf(void);
void wrist_write_to_refbuf(void);
void ankle_write_to_refbuf(void);

void write_to_netbuf(void);
// (an_ulog.c)
void ankle_write_to_netbuf(void);

s32 init_module(void);
// void cleanup_module(void);
void cleanup_signal(s32);

void unload_module(void);
void start_routine(void *);
void shm_copy_commands(void);
void main_tests(void);
void main_init(void);
void main_loop(void);

void docarr(void);

// math.c
xy jacob2d_x_p2d(mat22, se);
mat22 jacob2d_x_j2d(mat22, mat22);
mat22 jacob2d_inverse(mat22);
mat22 jacob2d_transpose(mat22);
xy xy_polar_cartesian_2d(se, se);
mat22 j_polar_cartesian_2d(se, se);
xy rotate2d(xy, f64);
xy xlate2d(xy, xy);
f64 xform1d(f64, f64, f64);
s32 ibracket(s32, s32, s32);
f64 dbracket(f64, f64, f64);
se preserve_orientation(se, f64);
f64 butter(f64, f64, f64);
f64 butstop(f64 *, f64 *);
f64 apply_filter(f64, f64 *);
f64 delta_radian_normalize(f64);
f64 radian_normalize(f64);
f64 min_jerk(f64, f64);
f64 i_min_jerk(u32, u32, f64);
f64 xasin(f64 x);
f64 xacos(f64 x);

// uei.c
void uei_ptr_init(void);
void uei_aio_init(void);
void uei_aio_close(void);
void uei_ain_read(void);
void uei_aout_write(f64, f64);
void test_uei_write(void);
void uei_dio_scan(void);
s32 uei_dout_write_masked(s32, u32, u32);
void uei_din_read(s32, u32 *);
void uei_dout01(s32);
void uei_aout32_test(void);
void uei_aout32_write(s32, u16, f64);
void cleanup_devices(void);

// isaft.c
void isa_ft_init(void);
void isa_ft_read(void);

// pc7266.c
void pc7266_init(void);
f64 pc7266_read_ch(u32);
void pc7266_reset_all_ctrs(void);
s32 pc7266_safe_check(void);
void pc7266_encoder_read(void);
void pc7266_calib(void);

// pci4e.c
void pci4e_init(void);
void pci4e_close(void);
void pci4e_reset_all_ctrs(void);
void pci4e_set_all_ctrs(void);
s32 pci4e_safe_check(void);
void pci4e_encoder_read(void);
void pci4e_calib(void);

// sensact.c
void sensact_init(void);
void dio_encoder_sensor(void);
void adc_tach_sensor(void);
void adc_ft_sensor(void);
void fast_read_ft_sensor(void);
void ft_zero_bias(void);
void adc_grasp_sensor(void);
void adc_accel_sensor(void);
void adc_current_sensor(void);

void dac_torque_actuator(void);
void set_zero_torque(void);
void write_zero_torque(void);
void vibrate(void);
void do_max(void);
void planar_set_zero_torque(void);
void planar_write_zero_torque(void);
void planar_after_compute_controls(void);

void wrist_set_zero_torque(void);
void wrist_write_zero_torque(void);
void wrist_init(void);
void wrist_after_compute_controls(void);
void wrist_sensor(void);
void wrist_moment(void);
void wrist_calc_vel(void);
void dac_wrist_actuator(void);

void ankle_set_zero_torque(void);
void ankle_write_zero_torque(void);
void ankle_init(void);
void ankle_after_compute_controls(void);
void ankle_sensor(void);
void ankle_moment(void);
void ankle_calc_vel(void);
void dac_ankle_actuator(void);

void linear_set_zero_force(void);
void linear_write_zero_force(void);
void linear_init(void);
void linear_after_compute_controls(void);
void linear_sensor(void);
void linear_calc_vel(void);
void dac_linear_actuator(void);

void hand_set_zero_force(void);
void hand_write_zero_force(void);
void hand_init(void);
void hand_after_compute_controls(void);
void hand_sensor(void);
void hand_calc_vel(void);
void dac_hand_actuator(void);



void initbuff(EMG_buffer *,f64 *,f64 *);
void start_emgbuff(EMG_buffer *);
void updateemgbuff (EMG_buffer *);
void set_emgbuff_bias(EMG_buffer *);
void stop_emgbuff(EMG_buffer *);



// pl_ulog.c
void init_log_fns(void);
void init_ref_fns(void);

// pl_uslot.c
void init_slot_fns(void);

// fifo.c
void init_fifos(void);
void cleanup_fifos(void);
// TODO: delete s32 fifo_input_handler(u32);
void fifo_input_handler(void);
void print_fifo_input(void);

void set_ob_variable(void);
s32 get_option_x64 (s8 **, u64 *);
s8 *get_options_x64 (s8 *, s32 , u64 *);

void dpr(s32 level, const s8 *format, ...);
void dpr_clear(void);
void dpr_flush(void);

//void fprf(RT_PIPE *, const s8 *, ...);
#define prf fprf
#include <stdio.h>
#include <unistd.h>
#include <syslog.h>

#endif				// ROBDECLS_H
