"""
Roll Stability of a Surface-Piercing Hydrofoil Boat with a single "V-Foil"

Given a symmetric V-foil, length of foil, angle of foil to vertical axis, and
the length of the foil to the CG of the boat, a system response can be made.

We can also inject disturbances to height, bank, and pitch, to assess system disturbance rejection!

TO MATH OUT:
    DOUBLE CHECK PROPER SIGNS:
        p, roll, -> right is positive
        q, pitch -> nose up is positive
        r, yaw,  -> nose right is positive
    Convert forces/torques to body angles!
    transform the CG downward (the cg is currently assumed to be at the 
        intersection of the two foils extended above the boat, which is waaaay too high)

    Long Term:
        Find max bank angle as function of foil length
        Create proper VDB!
        able to inject noise into h (height), bank, and pitch, also other variables if it's easy
"""

from math import *
import fluids as flu

def get_lengths(p, G, h):
    """
    Based on some geometry, we can get:
    Laa - the moment arm to the midpoint of the left foil above water
    Lwa - the moment arm to the midpoint of the left foil below water
    Raa - the moment arm to the midpoint of the right foil above water
    Rwa - the moment arm to the midpoint of the right foil below water
    Lwl - the length of submerged left foil
    Lal - the length of unsubmerged left foil
    Rwl - the length of submerged right foil
    Ral - the length of unsubmerged right foil
    
    given the:
    
    p - bank angle, in radians, left = positive
    gamma - the foil angle to vertical axis, in radians
    Ll - total length of left foil
    Rl - total length of right foil (= Ll)
    lb - the length of the foil inside the boat to the cg
    h - height of CG over water
    """

    gamma = G["gamma"]
    Ll = G["Ll"]
    Rl = G["Rl"]
    lb = G["lb"]
    
    #Get foil length in air
    if p >= 0:
        Lal = h/sin(pi/2-p-gamma) - lb
        Ral = h/cos(p+gamma) - lb
    elif p < 0:
        p = abs(p)
        Ral = h/sin(pi/2-p-gamma) - lb
        Lal = h/cos(p+gamma) - lb   
    else:
        raise Exception('p undefined in get_lengths')
    
    #Get foil length in water
    Lwl = Ll - Lal - lb
    Rwl = Rl - Ral - lb
    
    #Get moment arms for air and water forces
    Laa = lb + Lal/2
    Lwa = lb + Lal + Lwl/2
    Raa = lb + Ral/2
    Rwa = lb + Ral + Rwl/2

    lengths = {"Laa": Laa,
               "Lwa": Lwa,
               "Raa": Raa,
               "Rwa": Rwa,
               "Lwl": Lwl,
               "Lal": Lal,
               "Rwl": Rwl,
               "Ral": Ral}
    
    return lengths

def get_thrust(v):
    
    force = 100 #Newtons

    prop_eff = .99
    motor_eff = .80
    speed_eff = v*.9

    Thrust = force*prop_eff*motor_eff*speed_eff/v
    notes = "http://www.epi-eng.com/propeller_technology/selecting_a_propeller.htm"
    
    return Thrust
    
def compute_accels(U, B, G):
    
    pw = 998
    pa = 1.225

    z = U(3)
    v = U(2)
    q = U(7)
    aoa = G["aoi"] + q
    aoa_t = G["aoi_t"] + q

    lengths = get_lengths(U["p"], G, z)

    F_LW = flu.foil_lift(pw, lengths["Lwl"], G["c"], v, aoa)
    F_LA = flu.foil_lift(pa, lengths["Lal"], G["c"], v, aoa)
    F_RW = flu.foil_lift(pw, lengths["Rwl"], G["c"], v, aoa)
    F_RA = flu.foil_lift(pa, lengths["Ral"], G["c"], v, aoa)
    F_TF = flu.tail_lift(v, aoa_t) #assumes tail force is -x direction

    D_LW = flu.foil_drag(pw, lengths["Lwl"], G["c"], v, aoa)
    D_LA = flu.foil_drag(pa, lengths["Lal"], G["c"], v, aoa)
    D_RW = flu.foil_drag(pw, lengths["Rwl"], G["c"], v, aoa)
    D_RA = flu.foil_drag(pa, lengths["Ral"], G["c"], v, aoa)
    D_TF = flu.tail_drag(v, aoa_t)
    
    Thrust = get_thrust(U["dxdt"])

    #ROLL TORQUE
    #from main foil lift, multiplied by changing lengths
    Tp_LW = F_LW*lengths["Lwa"]
    Tp_LA = F_LA*lengths["Laa"]
    Tp_RW = F_RW*lengths["Rwa"]
    Tp_RA = F_RA*lengths["Raa"]
    #from main foil drag, is zero, because in-plane
    #from tail lift, is zero, because symmetric
    #from tail drag, is zero, because in-plane

    #PITCH TORQUE
    #from main foil lift, multiplied by distance from foil to CG
    Tq_LW = F_LW*sin(gamma)*G["LM"]
    Tq_LA = F_LA*sin(gamma)*G["LM"]
    Tq_RW = F_RW*sin(gamma)*G["LM"]
    Tq_RA = F_RA*sin(gamma)*G["LM"]
    #from main foil drag, multiplied by changing lengths
    Tq_FD = D_LW*lengths["Lwa"] + D_LA*lengths["Laa"] + D_RW*lengths["Rwa"] + D_RA*lengths["Raa"]
    #from tail foil, multiplied by fixed distances
    Tq_TL = F_TF*G["LT"]
    Tq_TD = D_TF*G["TS"]
    
    #YAW TORQUE
    #from main foil lift, multiplied by length, Lwa*sin(gamma)
    #from main foil drag, multiplied by length, Lwa*sin(gamma)
    #from tail lift, but only with roll angle (and yaw angle?)
    #from tail drag, but only with roll angle (and yaw angle?)

    #ADD transform forces by pitch, roll angle (and yaw angle, if 6DOF)
    Fx = Thrust - (D_LW + D_LA + D_RW + D_RA)
    #Fy = F_LW*cos(gamma) + F_LA*cos(gamma) - (F_RW*cos(gamma) + F_RA*cos(gamma)) #6DOF
    Fz = F_LW*sin(gamma) + F_LA*sin(gamma) + (F_RW*sin(gamma) + F_RA*sin(gamma)) - B["m"]*9.81 + F_TF

    Tp = (Tp_LW + Tp_LA) - (Tp_RW + Tp_RA)
    Tq = (Tq_LW + Tq_LA + Tq_RW + Tq_RA) - (Tq_FD + Tq_TD) + Tq_TL
    #Tr = 0 #6DOF

    d2x_dt2 = Fx/B["m"]
    d2z_dt2 = Fz/B["m"]
    
    d2p_dt2 = Tp/B["Ixx"]
    d2q_dt2 = Tq/B["Iyy"]
    
    return [d2x_dt2, d2z_dt2, d2p_dt2, d2q_dt2]
    
    
def FE_next(U, dt, accels):

    #U = [t, x, dxdt, z, dzdt, p, dpdt, q, dqdt]
    """
    t, time
    x, forward position
    z, height above water
    p, roll
    q, pitch
    """

    d2x_dt2 = accels[0]
    d2z_dt2 = accels[1]
    d2p_dt2 = accels[2]
    d2q_dt2 = accels[3]
    
    t      = U(0, -1)
    x      = U(1, -1)
    dxdt   = U(2, -1)
    z      = U(3, -1)
    dzdt   = U(4, -1)
    p      = U(5, -1)
    dpdt   = U(6, -1)
    q      = U(7, -1)
    dqdt   = U(8, -1)
    
    #Forward Euler
    t_next = t + dt

    dxdt_next = dxdt + d2x_dt2*dt
    x_next = x + dxdt*dt
    
    dzdt_next = dzdt + d2z_dt2*dt
    z_next = z + dzdt*dt
    
    dpdt_next = dpdt + d2p_dt2*dt
    p_next = p + dpdt*dt

    dqdt_next = dqdt + d2q_dt2*dt
    q_next = q + dqdt*dt
    
    U_next = [t_next, x_next, z_next, p_next, q_next, dxdt_next, dzdt_next, dpdt_next, dqdt_next]
    
    return U_next

def steady_state(B, G, v, h):
    c = B["c"]
    cl = flu.foil_cL(G["aoi"])
    Ll = G["Ll"]
    lb = G["lb"]
    gamma = G["gamma"]
    g = 9.81
    m = B["m"]
    pa = 1.225
    pw = 998

    if v == 0:
        v = sqrt((m*g)/(c*cl*((h/cos(gamma)-lb)*pa+(Ll-h/cos(gamma))*pw)))
        return v
    elif h == 0:
        h = (cos(gamma)/(pa-pw))*(lb*pa-Ll*pw+c*cl/(m*g)*(v**2))
        return h
    else:
        return 0

def single_run():

    dt = .1
    U = [0, 0, 0, 0, 0, 0, 0, 0, 0]
    #if starting at steady state and non-zero, use steady_state()
    
    accels = compute_accels(U, B, G)
    
    FE_next(accels, dt)

    #do for 1000 time steps or whatever, check takeoff sim logic
    
#Balance Quantities
m = 2000 #lbs, weighed
Ixx = 1 #kg/m^2, estimated
Iyy = 1 #kg/m^2, estimated
Izz = 1 #kg/m^2, estimated

#Geometric Quantities
L = 2   #m, foil length, measured
lb = 1  #m, from foil to center of gravity...??!
c = .5  #m, chord of foil, measured
aoi = 2 #deg, angle of incidence of main foils, measured
aoi_t = -2 #deg, angle of incidence of tail foil, measured
gamma = 25 #deg, foil anhedral, measured
LM = 0.5 #m, length of main foil cp from boat cg
LT = 3.0 #m, length of tail foil cp from boat cg
TS = 2.0 #m, length of foil strut from longitudinal axis

B = {"mass": m/2.205,
     "Ixx": Ixx,
     "Iyy": Iyy,
     "Izz": Izz}
G = {"Ll": L,
     "Rl": L,
     "lb": lb,
     "c": c,
     "aoi": aoi*pi/180,
     "aoi_t": aoi_t*pi/180,
     "gamma": gamma*pi/180,
     "LM": LM,
     "LT": LT}
 