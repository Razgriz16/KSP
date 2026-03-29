import math
import time
import krpc
from helpers import *

conn = krpc.connect(name='Calculations')
vessel = conn.space_center.active_vessel

# mass for meco

total_mass = vessel.mass
dry_mass = vessel.dry_mass
prop_mass = total_mass - dry_mass

# first stage mass 
stage1 = vessel.parts.in_decouple_stage(stage=0)
stage1_dry_mass = sum([x.dry_mass for x in stage1])
stage1_wet_mass = sum([x.mass for x in stage1]) 
stage1_prop_mass = stage1_wet_mass - stage1_dry_mass

# second stage mass
stage2_dry_mass = dry_mass - stage1_dry_mass
stage2_wet_mass = total_mass - stage1_wet_mass
stage2_prop_mass = stage2_wet_mass - stage2_dry_mass

# calculate reserve propellant
Isp = vessel.specific_impulse
dv_recovery = 1500
reserve_prop = required_reserve_prop(stage1_dry_mass, dv_recovery, Isp)
meco_mass = (stage1_dry_mass + stage2_wet_mass) + reserve_prop +50  # +50 kg safety margin

# print results


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

print(f"meco mass: {meco_mass(vessel)} kg")