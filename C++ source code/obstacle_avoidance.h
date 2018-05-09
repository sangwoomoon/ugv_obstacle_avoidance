#define	N_step	(int)(7)	// receding horizon time stpes
#define	N_angle	(int)(7)	// angular rate variation stpes
#define	dt		(double)(0.2)	// time stpe

#define	MAX_omega	(double)(50*D2R)	// maximum angular velocity
#define	MAX_omega_dot	(double)(MAX_omega/dt)	// maximum angular acceleration

#define VELO_LAW	(double)(1)	// m/sec
#define	VELO_HIGH	(double)(3)	// m/sec

#define	BIAS_OBS	(double)(0.1)	// biased cost
#define	BIAS_GOAL	(double)(0.5)	// biased cost

#define COMMAND_IDX (int)(1)		// command step index variable
#define TARGET_DIST (double)(5.0)    // target distance from vehicle to destination

#define THRESH_LASER (double)(1.0)  // threshold