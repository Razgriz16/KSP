import math

def delta_v(m0, m1, Isp):
    """
    Calculate the delta-v required to change from mass m0 to mass m1 with a specific impulse Isp.

    Parameters:
    m0 (float): Initial mass (kg)
    m1 (float): Final mass (kg)
    Isp (float): Specific impulse (s)

    Returns:
    float: Delta-v required (m/s)
    """
    g0 = 9.81  # Standard gravity (m/s^2)
    
    if m1 <= 0 or m0 <= 0:
        raise ValueError("Masses must be greater than zero.")
    
    if Isp <= 0:
        raise ValueError("Specific impulse must be greater than zero.")
    
    delta_v = Isp * g0 * math.log(m0 / m1)
    
    return delta_v

# rocket equation for initial mass: m0 = mf * e^(Δv / Isp * g0)
def required_initial_mass(mf, delta_v, Isp):
    """
    Calculate the required initial mass (m0) to achieve a certain delta-v given a final mass (mf) and specific impulse (Isp).

    Parameters:
    mf (float): Final mass (kg)
    delta_v (float): Desired delta-v (m/s)
    Isp (float): Specific impulse (s)

    Returns:
    float: Required initial mass (kg)
    """
    g0 = 9.81  # Standard gravity (m/s^2)
    
    if mf <= 0:
        raise ValueError("Final mass must be greater than zero.")
    
    if Isp <= 0:
        raise ValueError("Specific impulse must be greater than zero.")
    
    m0 = mf * math.exp(delta_v / (Isp * g0))
    
    return m0

def required_final_mass(m0, delta_v, Isp):
    """
    Calculate the required final mass (mf) to achieve a certain delta-v given an initial mass (m0) and specific impulse (Isp).

    Parameters:
    m0 (float): Initial mass (kg)
    delta_v (float): Desired delta-v (m/s)
    Isp (float): Specific impulse (s)

    Returns:
    float: Required final mass (kg)
    """
    g0 = 9.81  # Standard gravity (m/s^2)
    
    if m0 <= 0:
        raise ValueError("Initial mass must be greater than zero.")
    
    if Isp <= 0:
        raise ValueError("Specific impulse must be greater than zero.")
    
    mf = m0 / math.exp(delta_v / (Isp * g0))
    
    return mf

def required_reserve_prop(m_dry, dv_recovery, Isp):
    """Propellant you must leave in the tanks after MECO for landing.
        obtained from the rocket equation: 
            dv = Isp * g0 * ln(m0 / mf) 
            => m0 = mf * e^(dv / Isp * g0) [mp = m0 - mf] (mp = propellant mass)
            => mp = mf * (e^(dv / Isp * g0) - 1)
        Parameters:
        m_dry (float): Dry mass (kg)
        dv_recovery (float): Recovery delta-v (m/s)
        Isp (float): Specific impulse (s)
        Returns:
        float: Required reserve propellant mass (kg)
    """

    g0 = 9.81
    if m_dry <= 0 or Isp <= 0:
        raise ValueError("Invalid input")
    return m_dry * (math.exp(dv_recovery / (Isp * g0)) - 1)

def vis_viva(mu, r, a):
    """Calculate orbital velocity using the vis-viva equation."""
    if r <= 0 or a <= 0:
        raise ValueError("Radius and semi-major axis must be greater than zero.")
    return math.sqrt(mu * ((2./r) - (1./a)))

def meco_mass(vessel) -> float:
    total_mass = vessel.mass
    dry_mass = vessel.dry_mass
    prop_mass = total_mass - dry_mass

    stage1 = vessel.parts.in_decouple_stage(stage=0)
    stage1_dry_mass = sum([x.dry_mass for x in stage1])
    stage1_wet_mass = sum([x.mass for x in stage1]) 
    #stage1_prop_mass = stage1_wet_mass - stage1_dry_mass

    stage2_dry_mass = dry_mass - stage1_dry_mass
    stage2_wet_mass = total_mass - stage1_wet_mass
    #stage2_prop_mass = stage2_wet_mass - stage2_dry_mass

    Isp = vessel.specific_impulse
    dv_recovery = 1500
    reserve_prop = required_reserve_prop(stage1_dry_mass, dv_recovery, Isp)
    meco_mass = (stage1_dry_mass + stage2_wet_mass) + reserve_prop +50  # +50 kg safety margin

    return meco_mass