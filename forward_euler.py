##################################################
#   4 Degree of Freedom Simulation for 
#   Surface Splitting Hydrofoil Boat
#   roll, pitch, z, and x (p, q, z, x)
#   Forward Rising Sky-Pitching Right Roll is positive
#
#   Copyright (c) Amira Malik 2024
##################################################

from parameters import vehicle_params, env_params
from compute_accels import compute_accels

######### STARTING CONDITIONS ############
pitch = 0; roll = 0; x = 0; z= 0
pitchRate = 0; rollRate = 0; Vx = 0; Vz = 0
pitchAccel = 0; rollAccel = 0; Ax = 0; Az = 0

veh_accels = [pitchAccel, rollAccel, Ax, Az]
veh_vels = [pitchRate, rollRate, Vx, Vz]
veh_locs = [pitch, roll, x, z]

dt = 0.1

############### RUN CONDITIONS ############

def evaluate_run_condition(veh_state, dt):
    
    # # RUN FOREVER
    # always_condition_met = True

    # RUN FOR SET TIME 
    run_time = 1000 #seconds
    time_condition_met = veh_state[0] > run_time/dt

    # RUN UNTIL HEIGHT IS MET
    target_height = 1 #meters
    height_condition_met = veh_state[2] > target_height

    return height_condition_met or time_condition_met

############## FORWARD EULER #################

while (not condition_met):

    veh_accels = veh_accels + compute_accels(veh_locs, veh_vels, vehicle_params, env_params)
    """from lander.py:

        for method in m_list:
        t, u = IVPlib.solve(lander_IVP, dt, method)
        V = []
        z = []
        a = []
        for i in range(len(t)):
            V.append(u[i][0])
            z.append(u[i][1])
            [an, vn] = lander_IVP.evalf(u[i][0], t)
            a.append(an)
            
    lander_Vzaplot(t, V, z, a, method.__name__)
    
    """

    ### NEED TO FIGURE OUT HOW TO DO THE VEHICLE STATE AND LIST APPEND!

    condition_met = evaluate_run_condition(veh_state, dt)