
#include "obstacle_avoidance.h"
#include "occupancygrid.h"

void obstacle_avoidance(VehicleStates VS, GridDataType OccupancyGrid[][], double *DestCoord)
{
	double POS_UGV_cnd[N_step+1][9*(N_angle+1)];
	double VELO_UGV_cnd[N_step+1][9*(N_angle+1)];
	double COST_cnd[3*(N_angle+1)];
	double POS_UGV[3];
	double VELO_UGV[3];

	double VELO_mag[3][N_step];
	double VELO_omega[3][N_step];
	double heading=0;
	double omega_dot=0;
	int idx;

	double ref_obs[2], ref_goal[2], ref_heading[2];
	double temp_obs_num, temp_obs_den, temp_obs, temp_goal_num, temp_goal_den, temp_goal, ref_theta, ref_theta_goal, laser_range;

	double min_cost;
	int min_idx;

	BOOL bPlanningThresh = FALSE;

	for (int iter_profile = 0; iter_profile < 3; iter_profile++)
	{
		for (int iter_step = 0; iter_step < N_step; iter_step++)
		{
			if (iter_profile == 0)
			{
				VELO_mag[iter_profile][iter_step] = 
					pow(pow(VS.Vned[0],2)+pow(VS.Vned[1],2),0.5)
					+(VELO_HIGH-pow(pow(VS.Vned[0],2)+pow(VS.Vned[1],2),0.5))
					/(double)(N_step)*(double)(iter_step+1);
			}
			else if (iter_profile == 1)
			{
				if (pow(pow(VS.Vned[0],2)+pow(VS.Vned[1],2),0.5) > VELO_LAW)
				{
					VELO_mag[iter_profile][iter_step] = 
						pow(pow(VS.Vned[0],2)+pow(VS.Vned[1],2),0.5)
						-(pow(pow(VS.Vned[0],2)+pow(VS.Vned[1],2),0.5)-VELO_LAW)
						/(double)(N_step)*(double)(iter_step+1);
				}
				else
				{	
					VELO_mag[iter_profile][iter_step] = 
						pow(pow(VS.Vned[0],2)+pow(VS.Vned[1],2),0.5)
						+((VELO_HIGH+VELO_LAW)/2+VELO_LAW-pow(pow(VS.Vned[0],2)
						+pow(VS.Vned[1],2),0.5))/(double)(N_step)*(double)(iter_step+1);
				}
			}
			else if (iter_profile == 2)
			{
				if (pow(pow(VS.Vned[0],2)+pow(VS.Vned[1],2),0.5) > VELO_LAW)
				{
					VELO_mag[iter_profile][iter_step] = 
						pow(pow(VS.Vned[0],2)+pow(VS.Vned[1],2),0.5);
				}
				else
				{
					VELO_mag[iter_profile][iter_step] = 
						pow(pow(VS.Vned[0],2)+pow(VS.Vned[1],2),0.5)
						-(pow(pow(VS.Vned[0],2)+pow(VS.Vned[1],2),0.5)-VELO_LAW)
						/(double)(N_step)*(double)(iter_step+1);
				}
			}

			omega_dot = MAX_omega_dot*(0.9*exp(-pow(VELO_mag[iter_profile][iter_step],2)/20)+0.1);

			for (int iter_angle = 0; iter_angle <= N_angle; iter_angle++)
			{
				if (iter_step == 0)
				{
					idx = 3*(iter_profile*(N_angle+1)+iter_angle);
					POS_UGV_cnd[0][idx] = VS.LCC_Comp[0];        // ??? LCC_Comp or LCC_Rcvd
					POS_UGV_cnd[0][idx+1] = VS.LCC_Comp[1];
					POS_UGV_cnd[0][idx+2] = VS.Att[2];
					VELO_UGV_cnd[0][idx] = VS.Vned[0];
					VELO_UGV_cnd[0][idx+1] = VS.Vned[1];
					VELO_UGV_cnd[0][idx+2] = VS.pqr[2];
				}
	
				idx = 3*(iter_profile*(N_angle+1)+iter_angle);

				VELO_omega[iter_profile][iter_step] = VS.pqr[2]+omega_dot*(-1+2*(double)(iter_angle)/(double)(N_angle))*dt;
				heading = VS.Att[2] + VELO_omega[iter_profile][iter_step]*dt;
	
				VELO_UGV_cnd[iter_step+1][idx] = VELO_mag[iter_profile][iter_step]*cos(heading);
				VELO_UGV_cnd[iter_step+1][idx+1] = VELO_mag[iter_profile][iter_step]*sin(heading);
				VELO_UGV_cnd[iter_step+1][idx+2] = VELO_omega[iter_profile][iter_step];
				POS_UGV_cnd[iter_step+1][idx] = POS_UGV_cnd[iter_step][idx]+VELO_mag[iter_profile][iter_step]*dt*cos(heading);
				POS_UGV_cnd[iter_step+1][idx+1] = POS_UGV_cnd[iter_step][idx+1]+VELO_mag[iter_profile][iter_step]*dt*sin(heading);
				POS_UGV_cnd[iter_step+1][idx+2] = heading; 

				//printf("(%2.2f %2.2f %2.2f)",POS_UGV_cnd[iter_step+1][idx],POS_UGV_cnd[iter_step+1][idx+1],POS_UGV_cnd[iter_step+1][idx+2]);
			}
			//printf("\n");
		}
		//printf("\n");
	}

	for (int iter_cnd = 0; iter_cnd < 3*(N_angle+1); iter_cnd++)
	{
		COST_cnd[iter_cnd] = 0;

		for (int iter_step = 0; iter_step <= N_step; iter_step++)
		{
			for (int Gridx = 0; Gridx < SIZE_X; Gridx++)
			{
				for (int Gridy = 0; Gridy < SIZE_Y; Gridy++)
				{
					ref_obs[0] = (Gridx-CEN_X)*GRID_X;
					ref_obs[1] = (Gridy-CEN_Y)*GRID_Y;

					temp_obs_num = ref_obs[0]*VELO_UGV_cnd[iter_step][3*iter_cnd]+ref_obs[1]*VELO_UGV_cnd[iter_step][3*iter_cnd+1];
					temp_obs_den = pow(pow(ref_obs[0],2)+pow(ref_obs[1],2),0.5)
						*pow(pow(VELO_UGV_cnd[iter_step][3*iter_cnd],2)+pow(VELO_UGV_cnd[iter_step][3*iter_cnd+1],2),0.5);
					temp_obs = temp_obs_num / temp_obs_den;
	
					if (temp_obs > 1)
						temp_obs = 1;
					else if (temp_obs < -1)
						temp_obs = -1;

					ref_theta = acos(temp_obs); 
	
					COST_cnd[iter_cnd] += 
						1/((double)(iter_step+1))*(BIAS_OBS+0.5*cos(ref_theta-PI)+0.5)*exp(-pow(pow(ref_obs[0],2)+pow(ref_obs[1],2),0.5)*abs(OccupancyGrid[Gridx][Gridy]/128));
				}
			} 

			ref_goal[0] = POS_UGV_cnd[iter_step][3*iter_cnd] - DestCoord[0];
			ref_goal[1] = POS_UGV_cnd[iter_step][3*iter_cnd+1] - DestCoord[1];

			temp_goal_num = cos(DestCoord[3])*VELO_UGV_cnd[iter_step][3*iter_cnd]+sin(DestCoord[3])*VELO_UGV_cnd[iter_step][3*iter_cnd+1];
			temp_goal_den = pow(pow(cos(DestCoord[3]),2)+pow(sin(DestCoord[3]),2),0.5)*pow(pow(VELO_UGV_cnd[iter_step][3*iter_cnd],2)+pow(VELO_UGV_cnd[iter_step][3*iter_cnd+1],2),0.5);
			temp_goal = temp_goal_num / temp_goal_den;

			if (temp_goal > 1)
				temp_goal = 1;
			else if (temp_goal < -1)
				temp_goal = -1;

			ref_theta_goal = acos(temp_goal);
			COST_cnd[iter_cnd] += (1/(double)(iter_step+1))*(BIAS_GOAL)*(1-exp(-(pow(pow(ref_goal[0],2)+pow(ref_goal[1],2),0.5))));//0.5*cos(ref_theta_goal-PI)+0.5+
		}

		if (iter_cnd == 0)
		{
			min_cost = COST_cnd[0];
			min_idx = 0;
		}
		else
		{
			if (min_cost > COST_cnd[iter_cnd])
			{
				min_cost = COST_cnd[iter_cnd];
				min_idx = iter_cnd;
			}
		}
	}

	if (min_cost > THRESH_LASER)
		bPlanningThresh = TRUE;

	POS_UGV[0] = POS_UGV_cnd[COMMAND_IDX][3*min_idx];
	POS_UGV[1] = POS_UGV_cnd[COMMAND_IDX][3*min_idx+1];
	POS_UGV[2] = POS_UGV_cnd[COMMAND_IDX][3*min_idx+2];
	VELO_UGV[0] = VELO_UGV_cnd[COMMAND_IDX][3*min_idx];
	VELO_UGV[1] = VELO_UGV_cnd[COMMAND_IDX][3*min_idx+1];
	VELO_UGV[2] = VELO_UGV_cnd[COMMAND_IDX][3*min_idx+2];

	printf("\n");
	for (int iter_print = 1; iter_print <= N_step; iter_print++)
	{
		if (iter_print==1)
		COLOR_ON_RED
		else
		COLOR_ON_CYAN
			printf("POSITION::(%2.2f %2.2f %2.1f) VELOCITY::(%2.2f %2.2f %2.1f MAG%2.1f) TIME::%2.1f\n",
				POS_UGV_cnd[iter_print][3*min_idx],
				POS_UGV_cnd[iter_print][3*min_idx+1],
				POS_UGV_cnd[iter_print][3*min_idx+2]*R2D,
				VELO_UGV_cnd[iter_print][3*min_idx],
				VELO_UGV_cnd[iter_print][3*min_idx+1],
				VELO_UGV_cnd[iter_print][3*min_idx+2]*R2D,
				pow(pow(VELO_UGV_cnd[iter_print][3*min_idx],2)+pow(VELO_UGV_cnd[iter_print][3*min_idx+1],2),0.5),
				iter_print*dt);
	}

	printf("\n");
	for (iter_print = 0; iter_print < N_angle+1; iter_print++)
	{
		COLOR_ON_WHITE
			printf("COST::%2.10f %2.10f %2.10f\n",COST_cnd[iter_print],COST_cnd[N_angle+1+iter_print],COST_cnd[2*(N_angle+1)+iter_print]);
	}
	printf("\n");

	if (bPlanningThresh) // if there are any obstacles
	{
		DestCoord[0] = POS_UGV[0]+TARGET_DIST*cos(POS_UGV[2]);  // margin to stable following (ad-hoc approach)
		DestCoord[1] = POS_UGV[1]+TARGET_DIST*sin(POS_UGV[2]);  // margin to stable following (ad-hoc approach)
		DestCoord[2] = 0;
		DestCoord[3] = POS_UGV[2];
		DestCoord[4] = pow(pow(VELO_UGV[0],2)+pow(VELO_UGV[1],2),0.5);
	}
}