import math
import time
import krpc
from helpers import *
from dataclasses import dataclass


@dataclass
class Telemetry:
    ut: object
    surface_speed: object
    altitude: object
    apoapsis: object
    mass: object
    pitch: object
    heading: object
    surface_speed_frame: object

class KSP:
    def __init__(self, conn_name='Orbital'):
        self.conn = krpc.connect(name=conn_name)
        self.vessel = self.conn.space_center.active_vessel
        self.control = self.vessel.control
        self.auto_pilot = self.vessel.auto_pilot

    def setup_telemetry(self) -> Telemetry:
        body_ref = self.vessel.orbit.body.reference_frame
        surface_ref = self.vessel.surface_reference_frame
        surface_speed_frame = self.conn.space_center.ReferenceFrame.create_hybrid(
            position=body_ref,
            rotation=surface_ref
        )
        surface_flight = self.vessel.flight(surface_speed_frame)

        # Telemetry streams 
        ut = self.conn.add_stream(getattr, self.conn.space_center, 'ut')
        surface_speed_stream = self.conn.add_stream(getattr, surface_flight, 'speed')
        altitude_stream = self.conn.add_stream(getattr, surface_flight, 'mean_altitude')  # more stable
        apoapsis = self.conn.add_stream(getattr, self.vessel.orbit, 'apoapsis_altitude')
        mass_stream = self.conn.add_stream(getattr, self.vessel, 'mass')   # total mass in kg
        current_pitch = self.conn.add_stream(getattr, surface_flight, 'pitch')
        current_heading = self.conn.add_stream(getattr, surface_flight, 'heading')

        telemetry = Telemetry(
            ut=ut,
            surface_speed=surface_speed_stream,
            altitude=altitude_stream,
            apoapsis=apoapsis,
            mass=mass_stream,
            pitch=current_pitch,
            heading=current_heading,
            surface_speed_frame=surface_speed_frame
        )

        return telemetry

    def execute_circularization(self, t: Telemetry):
        print('Planning circularization burn')
        mu = self.vessel.orbit.body.gravitational_parameter
        r = self.vessel.orbit.apoapsis
        a1 = self.vessel.orbit.semi_major_axis
        
        v1 = vis_viva(mu, r, a1)
        v2 = vis_viva(mu, r, r)
        delta_v_req = v2 - v1
        
        node = self.control.add_node(
            t.ut() + self.vessel.orbit.time_to_apoapsis, prograde=delta_v_req)
        
        # Calculate burn time
        F = self.vessel.available_thrust
        Isp = self.vessel.specific_impulse
        m0 = self.vessel.mass
        m1 = required_final_mass(m0, delta_v_req, Isp)
        flow_rate = F / Isp
        burn_time = (m0 - m1) / flow_rate
        
        # Orientate ship
        print('Orientating ship for circularization burn')
        self.auto_pilot.reference_frame = node.reference_frame
        self.auto_pilot.target_direction = (0, 1, 0)
        self.auto_pilot.wait()
        
        # Wait until burn
        print('Waiting until circularization burn')
        burn_ut = t.ut() + self.vessel.orbit.time_to_apoapsis - (burn_time/2.)
        lead_time = 5
        self.conn.space_center.warp_to(burn_ut - lead_time)       
        
        # Execute burn
        print('Ready to execute burn')
        time_to_apoapsis = self.conn.add_stream(getattr, self.vessel.orbit, 'time_to_apoapsis')
        while time_to_apoapsis() - (burn_time/2.) > 0:
            pass
        print('Executing burn')
        self.control.throttle = 1.0
        time.sleep(burn_time - 0.1)
        
        print('Fine tuning')
        self.control.throttle = 0.05
        remaining_burn = self.conn.add_stream(node.remaining_burn_vector, node.reference_frame)
        while remaining_burn()[1] > 0:
            pass
        self.control.throttle = 0.0
        node.remove()
        print('Launch complete')


def main():
    ksp = KSP()
    t = ksp.setup_telemetry()

    # Pre-launch setup
    ksp.auto_pilot.reference_frame = t.surface_speed_frame
    ksp.auto_pilot.sas = True
    ksp.control.rcs = False
    ksp.control.throttle = 1.0

    # Flags
    has_tilted = False
    has_staged = False

    # Countdown...
    ksp.conn.ui.message('3...')
    time.sleep(1)
    ksp.conn.ui.message('2...')
    time.sleep(1)
    ksp.conn.ui.message('1...')
    time.sleep(1)

    ksp.conn.ui.message('Launch!')
    ksp.control.activate_next_stage()

    target_meco_mass = meco_mass(ksp.vessel)

    # ====================== MAIN FLIGHT LOOP ======================
    while True:
        # Real-time telemetry
        print(f"Surface speed: {t.surface_speed():.1f} m/s")
        print(f"Altitude:      {t.altitude():.1f} m")
        print(f"Apoapsis:      {t.apoapsis():.1f} m")
        print(f"Total mass:    {t.mass():.0f} kg")
        print(f"Reserve target: cutoff at {target_meco_mass:.0f} kg")
        print(f"Pitch:         {t.pitch():.1f}°")

        # 2° east kick + PROGRADE LOCK (fixed forever)
        if (not has_tilted and 
            t.surface_speed() > 50 and 
            t.surface_speed() < 100):
            
            # Gravity turn initiation: 2° east kick
            ksp.conn.ui.message('Engaging autopilot – 2° east kick...')
            ksp.auto_pilot.engage()
            ksp.auto_pilot.reference_frame = t.surface_speed_frame
            ksp.auto_pilot.target_roll = 0.0
            ksp.auto_pilot.target_pitch_and_heading(87, 90)
            time.sleep(1.75)

            ksp.auto_pilot.disengage()

            has_tilted = True

        if(has_tilted):
            ksp.auto_pilot.sas = True
            ksp.auto_pilot.sas_mode = ksp.conn.space_center.SASMode.prograde

        # Stage separation when mass drops to cutoff (upper stage empty + reserve)
        if (not has_staged and t.mass() <= target_meco_mass):
            ksp.control.throttle = 0.0
            time.sleep(1)  # small delay to ensure throttle cut is registered
            jettisoned = ksp.control.activate_next_stage()
            ksp.control.throttle = 1.0
            
            if jettisoned:
                booster = jettisoned[0]
                ksp.conn.ui.message(f'Booster separated: {booster.name}')
                print(f'Booster name: {booster.name}')
                
                # Extend aerobrakes on the booster only
                time.sleep(0.5)  # small delay to ensure separation is complete
                booster.control.brakes = True
                ksp.conn.ui.message('Aerobrakes extended on booster')
                
                # Booster telemetry 
                booster_body_ref = booster.orbit.body.reference_frame
                booster_surface_ref = booster.surface_reference_frame
                booster_frame = ksp.conn.space_center.ReferenceFrame.create_hybrid(
                    position=booster_body_ref,
                    rotation=booster_surface_ref
                )
                booster_flight = booster.flight(booster_frame)
                booster_speed = ksp.conn.add_stream(getattr, booster_flight, 'speed')
                booster_alt = ksp.conn.add_stream(getattr, booster_flight, 'mean_altitude')
                booster_rho = ksp.conn.add_stream(getattr, booster_flight, 'atmosphere_density')
                booster_dyn_pressure = ksp.conn.add_stream(getattr, booster_flight, 'dynamic_pressure')
                
                ksp.conn.ui.message('Booster telemetry ready (speed, alt, density, Q)')
                
                has_staged = True
            
        if(t.apoapsis() >= 80000.0):
            ksp.control.throttle = 0.0
            ksp.conn.ui.message('Target apoapsis reached, cutting throttle')
            print('Target apoapsis reached, cutting throttle')
            break
        time.sleep(0.1)


    while t.altitude() < 70000:
        pass
    
    ksp.execute_circularization(t)
                
if __name__ == "__main__":
    main()
