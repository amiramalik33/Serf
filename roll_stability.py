"""
Roll Stability of a Surface-Piercing Hydrofoil Boat with a single "V-Foil"

Given a symmetric V-foil, length of foil, angle of foil to vertical axis, and
the length of the foil to the CG of the boat, a system response can be made.

Hopefully, I will also be able to inject disturbances to height and angle

TO MATH OUT:
    is left bank angle positive or negative? idk, but make sure you're consistent
    Put all set variables in a dictionary, or global
    All lengths per time step should be in a dictionary
    Forces to Torques (including angles :( )
    Break up forcing function: torques are handled in each axis separately

    Long Term:
        Find max bank angle as function of foil length
        Create proper VDB!
        transform the CG downward (the cg is currently assumed to be at the 
            intersection of the two foils extended above
            the boat, which is waaaay too high)
        able to inject noise into h (height), bank, and pitch, also other variables if it's easy
"""

from math import *

def get_lengths(phi, gamma, Ll, Rl, lb, h):
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
    
    phi - bank angle, in radians, left = positive
    gamma - the foil angle to vertical axis, in radians
    Ll - total length of left foil
    Rl - total length of right foil (= Ll)
    lb - the length of the foil inside the boat to the cg
    """
    
    #Get foil length in air
    if phi >= 0:
        Lal = h/sin(pi/2-phi-gamma) - lb
        Ral = h/cos(phi+gamma) - lb
    elif phi < 0:
        phi = abs(phi)
        Ral = h/sin(pi/2-phi-gamma) - lb
        Lal = h/cos(phi+gamma) - lb   
    else:
        raise Exception('Phi undefined in get_lengths')
    
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

def get_cL(aoa):
    """
    Returns lift coefficient given AOA
    """
    
    """
    #Simple Lookup Aero Database for Modified Airfoil
    
    cLs = {0.1:0.2,
           0.2:0.3,
           0.3:0.4
        }
    
    try:
        cL = cLs[round(aoa,1)]
    except:
        print("AOA out of range of ADB")
    """
    
    return aoa/aoa

def get_lift(b, c, v, aoa):
    """
    Get lift for a rectangular wing
    Uses aero database to look up cL given AOA
    """
    
    rho = 998
    
    cL = get_cL(aoa)
    
    L = 1/2*cL*b*c*rho*(v**2)
    
    return L

def get_thrust(fixed):
    
    force = 100 #Newtons

    prop_eff = .99
    motor_eff = .80

    Thrust = force*prop_eff*motor_eff
    
    return Thrust
    
def compute_accels(I, m, phi, gamma, Ll, Rl, lb, h, c, v, incidence, fixed):
    
    lengths = get_lengths(phi, gamma, Ll, Rl, lb, h)
    
    q = 0 #pitch angle

    aoa = incidence + q

    F_LW = get_lift(lengths["Lwl"], c, v, aoa)
    F_LA = get_lift(lengths["Lal"], c, v, aoa)
    F_RW = get_lift(lengths["Rwl"], c, v, aoa)
    F_RA = get_lift(lengths["Ral"], c, v, aoa)
    
    Thrust = get_thrust(fixed)

    T_LW = F_LW*lengths["Lwa"]
    T_LA = F_LA*lengths["Laa"]
    T_RW = F_RW*lengths["Rwa"]
    T_RA = F_RA*lengths["Raa"]

    Fx = Thrust
    Fy = F_LW*cos(gamma) + F_LA*cos(gamma) - (F_RW*cos(gamma) + F_RA*cos(gamma))
    Fz = F_LW*sin(gamma) + F_LA*sin(gamma) + (F_RW*sin(gamma) + F_RA*sin(gamma))

    Tp = (T_LW + T_LA) - (T_RW + T_RA)
    Tq = 0
    Tr = 0
    
    d2phi_dt2 = Tp/I
    
    d2y_dt2 = Fy/m
    
    return [d2phi_dt2, d2y_dt2]
    
    
def FE_next(U, dt, I, m, phi, gamma, Ll, Rl, lb, h, c, v, aoa):
    
    accels = compute_accels(I, m, phi, gamma, Ll, Rl, lb, h, c, v, aoa)

    #U = [t, x, dxdt, y, dydt, z, dzdt, p, dpdt, q, dqdt, r, drdt]
    """
    t, time
    x, forward position
    dxdt is related to z at steady state
    y, lateral position (maybe not needed)
    z, height above water
    p, roll
    q, pitch
    r, yaw
    """
    
    #THE REST OF THIS IS COPY PASTE FROM TAKEOFF SIM
    t      = U(1, -1);
    y      = U(2, -1);
    phi    = U(3, -1);
    dydt   = U(4, -1);
    dphidt = U(5, -1);

    d2phi_dt2 = accels[0]
    d2y_dt2 = accels[1] - m*9.81
    
    #Forward Euler
    t_next = t + dt
    
    dydt_next = dydt + d2y_dt2*dt
    y_next = y + dydt*dt
    
    dphidt_next = dphidt + d2phi_dt2*dt
    phi_next = phi + dphidt*dt
    
    U_next = [t_next, y_next, phi_next, dydt_next, dphidt_next]
    
    return U_next

    
    
    