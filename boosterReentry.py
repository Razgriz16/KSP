import math
import time

def land_booster(conn, booster):
    print(f"--- Initiating landing sequence for {booster.name} ---")
    
    # Switch focus to the booster so it doesn't get deleted by KSP
    conn.space_center.active_vessel = booster
    control = booster.control
    auto_pilot = booster.auto_pilot

    # Telemetry setup for the booster
    body = booster.orbit.body
    surface_ref = booster.surface_reference_frame
    flight = booster.flight(surface_ref)
    
    # Streams
    mass_stream = conn.add_stream(getattr, booster, 'mass')
    available_thrust = conn.add_stream(getattr, booster, 'available_thrust')
    vertical_speed = conn.add_stream(getattr, flight, 'vertical_speed')
    # Use surface_altitude (radar altimeter), NOT mean_altitude (sea level)
    surface_alt = conn.add_stream(getattr, flight, 'surface_altitude')
    
    # Autopilot Setup - Point Retrograde to kill horizontal and vertical speed
    auto_pilot.reference_frame = surface_ref
    auto_pilot.engage()
    auto_pilot.target_direction = (0, -1, 0) # Initially point straight up (relative to surface)
    
    # 1. COASTING PHASE - Wait until we are falling
    print("Coasting to Apogee...")
    while vertical_speed() > 0:
        time.sleep(0.5)
        
    print("Apogee reached. Falling. Pointing Retrograde.")
    # Lock to surface retrograde to let aerodynamics and engines kill horizontal speed
    auto_pilot.sas = True
    auto_pilot.sas_mode = conn.space_center.SASMode.retrograde
    
    # 2. CALCULATION PHASE - The Hoverslam / Suicide Burn loop
    print("Waiting for Suicide Burn altitude...")
    control.brakes = True # Ensure aerobrakes are out
    
    legs_deployed = False
    
    while True:
        # Get current telemetry
        m = mass_stream()
        thrust = available_thrust()
        v = abs(vertical_speed()) # We only care about magnitude for the math
        alt = surface_alt()
        g = body.surface_gravity
        
        # Prevent division by zero if engine is flamed out
        if thrust == 0:
            a_max = 0.001 
        else:
            # Net acceleration upwards = (Thrust / Mass) - Gravity
            a_max = (thrust / m) - g
            
        # Calculate Stopping Distance (d = v^2 / 2a)
        # We add a 10% safety margin (1.1) to account for slight delays
        if a_max > 0:
            stopping_distance = (v ** 2) / (2 * a_max) * 1.1 
        else:
            stopping_distance = 999999 # Can't stop, fall infinitely
            
        # Deploy legs at 2000m
        if alt < 2000 and not legs_deployed:
            control.gear = True
            print("Landing legs deployed!")
            legs_deployed = True
            
        # At ~50m, switch from Retrograde to Radial Out (Straight up)
        # If we stay retrograde at 0m/s, the rocket will flip over!
        if alt < 50 and v < 10:
             auto_pilot.sas_mode = conn.space_center.SASMode.radial
             
        # BURN LOGIC
        if alt <= stopping_distance:
            # We need to burn!
            # Proportional throttle: As speed nears 0, throttle drops to hover
            ideal_throttle = (stopping_distance / alt)
            control.throttle = min(1.0, ideal_throttle)
        else:
            control.throttle = 0.0
            
        # TOUCHDOWN LOGIC
        if booster.situation == conn.space_center.VesselSituation.landed or \
           booster.situation == conn.space_center.VesselSituation.splashed:
            control.throttle = 0.0
            auto_pilot.disengage()
            print("Touchdown confirmed!")
            break
            
        time.sleep(0.05) # Loop runs 20 times a second

    print("Booster landing complete.")