import math
import time
import krpc
from helpers import *
from boosterReentry import land_booster

conn = krpc.connect(name='Orbital')
vessel = conn.space_center.active_vessel
control = vessel.control
auto_pilot = vessel.auto_pilot

body_ref = vessel.orbit.body.reference_frame
surface_ref = vessel.surface_reference_frame
surface_speed_frame = conn.space_center.ReferenceFrame.create_hybrid(
    position=body_ref,
    rotation=surface_ref
)
surface_flight = vessel.flight(surface_speed_frame)

# Telemetry streams 
ut = conn.add_stream(getattr, conn.space_center, 'ut')
surface_speed_stream = conn.add_stream(getattr, surface_flight, 'speed')
altitude_stream = conn.add_stream(getattr, surface_flight, 'mean_altitude')  # more stable
apoapsis = conn.add_stream(getattr, vessel.orbit, 'apoapsis_altitude')
mass_stream = conn.add_stream(getattr, vessel, 'mass')   # total mass in kg
current_pitch = conn.add_stream(getattr, surface_flight, 'pitch')
current_heading = conn.add_stream(getattr, surface_flight, 'heading')

stage_1_resources = vessel.resources_in_decouple_stage(stage=1, cumulative=False)  # keep if you want later

# Pre-launch setup
auto_pilot.reference_frame = surface_speed_frame
auto_pilot.sas = True
control.rcs = False
control.throttle = 1.0

# Flags
has_tilted = False
has_staged = False

# Countdown...
conn.ui.message('3...')
time.sleep(1)
conn.ui.message('2...')
time.sleep(1)
conn.ui.message('1...')
time.sleep(1)

conn.ui.message('Launch!')
vessel.control.activate_next_stage()

meco_mass = meco_mass(vessel)

# ====================== MAIN FLIGHT LOOP ======================
while True:
    # Real-time telemetry
    print(f"Surface speed: {surface_speed_stream():.1f} m/s")
    print(f"Altitude:      {altitude_stream():.1f} m")
    print(f"Apoapsis:      {apoapsis():.1f} m")
    print(f"Total mass:    {mass_stream():.0f} kg")
    print(f"Reserve target: cutoff at {meco_mass:.0f} kg")
    print(f"Pitch:         {current_pitch():.1f}°")

    # 2° east kick + PROGRADE LOCK (fixed forever)
    if (not has_tilted and 
        surface_speed_stream() > 50 and 
        surface_speed_stream() < 100):
        
        # Gravity turn initiation: 2° east kick
        conn.ui.message('Engaging autopilot – 2° east kick...')
        auto_pilot.engage()
        auto_pilot.reference_frame = surface_speed_frame
        auto_pilot.target_roll = 0.0
        auto_pilot.target_pitch_and_heading(85, 90)
        auto_pilot.wait()
        auto_pilot.disengage()

        has_tilted = True

    if(has_tilted):
        auto_pilot.sas = True
        auto_pilot.sas_mode = conn.space_center.SASMode.prograde
        print('Prograde locked')

    # Stage separation when mass drops to cutoff (upper stage empty + reserve)
    if (not has_staged and mass_stream() <= meco_mass):
        vessel.control.throttle = 0.0
        time.sleep(1)
        jettisoned = control.activate_next_stage()
        time.sleep(1.25)
        vessel.control.throttle = 1.0
        
        if jettisoned:
            booster = jettisoned[0]
            conn.ui.message(f'Booster separated: {booster.name}')
            print(f'Booster name: {booster.name}')
            
            # Extend aerobrakes on the booster only
            time.sleep(0.5)  # small delay to ensure separation is complete
            booster.control.brakes = True
            booster.control.rcs = True
            #land_booster(conn, booster)
            conn.ui.message('Aerobrakes extended on booster')
            
            # Booster telemetry 
            booster_body_ref = booster.orbit.body.reference_frame
            booster_surface_ref = booster.surface_reference_frame
            booster_frame = conn.space_center.ReferenceFrame.create_hybrid(
                position=booster_body_ref,
                rotation=booster_surface_ref
            )
            booster_flight = booster.flight(booster_frame)
            booster_speed = conn.add_stream(getattr, booster_flight, 'speed')
            booster_alt = conn.add_stream(getattr, booster_flight, 'mean_altitude')
            booster_rho = conn.add_stream(getattr, booster_flight, 'atmosphere_density')
            booster_dyn_pressure = conn.add_stream(getattr, booster_flight, 'dynamic_pressure')
            
            conn.ui.message('Booster telemetry ready (speed, alt, density, Q)')
            
            has_staged = True
        
    if(apoapsis() >= 80000.0):
        control.throttle = 0.0
        conn.ui.message('Target apoapsis reached, cutting throttle')
        print('Target apoapsis reached, cutting throttle')
        break
    time.sleep(0.1)


while altitude_stream() >= 70000:
    pass
# Plan circularization burn (using vis-viva equation)
print('Planning circularization burn')
mu = vessel.orbit.body.gravitational_parameter
r = vessel.orbit.apoapsis
a1 = vessel.orbit.semi_major_axis
a2 = r
v1 = vis_viva(mu, r, a1)
v2 = vis_viva(mu, r, a2)
delta_v = v2 - v1
print(f"delta v calculada: {delta_v}")
node = vessel.control.add_node(
    ut() + vessel.orbit.time_to_apoapsis, prograde=delta_v)

# Calculate burn time (using rocket equation)
F = vessel.available_thrust
Isp = vessel.specific_impulse
m0 = vessel.mass
m1 = required_final_mass(m0, delta_v, Isp)
flow_rate = F / (Isp * 9.81) 
burn_time = (m0 - m1) / flow_rate

print(f"F {F}")
print(f"Isp {Isp}")
print(f"m0 {m0}")
print(f"m1 {m1}")
print(f"flow_rate {flow_rate}")
print(f"burn_time {burn_time}")

# Orientate ship
print('Orientating ship for circularization burn')
vessel.auto_pilot.reference_frame = node.reference_frame
vessel.auto_pilot.target_direction = (0, 1, 0)
vessel.auto_pilot.wait()

# Wait until burn
print('Waiting until circularization burn')
burn_ut = ut() + vessel.orbit.time_to_apoapsis - (burn_time/2.)
print("burn_ut", burn_ut)
lead_time = 10
conn.space_center.warp_to(burn_ut - lead_time)       
# Execute burn
print('Ready to execute burn')
time_to_apoapsis = conn.add_stream(getattr, vessel.orbit, 'time_to_apoapsis')
print("time_to_apoapsis", time_to_apoapsis())

print(f"time_to_apoapsis()-(burn_time/2.): {time_to_apoapsis()-(burn_time/2.)}")
while time_to_apoapsis() - (burn_time/2.) > 0:
    pass


print('Executing burn')
vessel.control.throttle = 1.0
time.sleep(burn_time - 0.1)
print('Fine tuning')
vessel.control.throttle = 0.5
remaining_burn = conn.add_stream(node.remaining_burn_vector, node.reference_frame)
print(f"remaining_burn: {remaining_burn()} remaining_burn[1]: {remaining_burn()[1]}")
while remaining_burn()[2] > 0:
    pass
vessel.control.throttle = 0.0
node.remove()
print('Launch complete')
            



