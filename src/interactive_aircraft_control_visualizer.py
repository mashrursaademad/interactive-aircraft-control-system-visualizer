import tkinter as tk
from tkinter import ttk, messagebox
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import threading
import time
import tkinter.font as tkfont

# ----------------------------
# Aircraft control visualizer
# ----------------------------
class AircraftControlVisualizer:
    def __init__(self, root):
        self.root = root
        self.root.title("Aircraft Control System Visualizer")
        self.root.geometry("1200x800")

        # Simulation state
        self.is_running = False
        self.sim_thread = None

        # Simulation parameters
        self.dt = 0.02  # simulation timestep (s)
        self.sim_time = 0.0
        self.max_time = 600.0 # max simulation time 600 seconds

        # Aircraft nominal parameters (tweak these)
        self.V = 70.0        # airspeed (m/s) - used to map pitch -> altitude rate roughly
        self.k_theta = 1.0   # elevator -> pitch gain
        self.wn_theta = 3.0  # pitch natural frequency
        self.zeta_theta = 0.7

        self.k_phi = 1.0     # aileron -> roll gain
        self.wn_phi = 4.0
        self.zeta_phi = 0.8

        # Kinematic coupling constants
        self.k_h = self.V    # altitude rate ~ V * theta (for small angles; units m/s)
        # Increased k_psi for better heading control response
        self.k_psi = 0.1     # heading rate per roll (rad/s per rad) — tuneable

        # Plant state vectors
        # pitch plant: theta, theta_dot
        self.theta = 0.0
        self.theta_dot = 0.0

        # roll plant: phi, phi_dot
        self.phi = 0.0
        self.phi_dot = 0.0

        # integrators for altitude and heading (kinematic)
        self.altitude = 1000.0  # m - initial altitude
        self.heading = 100.0      # degrees

        # controllers' internal state (integrators + derivative memory)
        self.C_alt_int = 0.0
        self.C_pitch_int = 0.0
        self.C_psi_int = 0.0
        self.C_roll_int = 0.0

        self.last_err_alt = 0.0
        self.last_err_pitch = 0.0
        self.last_err_psi = 0.0
        self.last_err_roll = 0.0

        # GUI variables (Set to "optimal" values by default)
        self.desired_alt = tk.DoubleVar(value=1000.0)
        self.desired_psi = tk.DoubleVar(value=100.0)  # degrees
        self.sim_speed = tk.DoubleVar(value=1.0)   # factor to slow/speed animation
        self.fast_sim_mode = tk.BooleanVar(value=False) # New: Fast simulation mode

        # PID gains (outer/inner for altitude/pitch and heading/roll)
        # Optimal values from last discussion
        self.Kp_alt = tk.DoubleVar(value=1.0)
        self.Ki_alt = tk.DoubleVar(value=0.1)
        self.Kd_alt = tk.DoubleVar(value=0.5)

        self.Kp_pitch = tk.DoubleVar(value=5.0)
        self.Ki_pitch = tk.DoubleVar(value=1.0)
        self.Kd_pitch = tk.DoubleVar(value=0.5)

        self.Kp_psi = tk.DoubleVar(value=5.0)
        self.Ki_psi = tk.DoubleVar(value=0.1)  # Reduced Ki_psi to prevent slow oscillations
        self.Kd_psi = tk.DoubleVar(value=0.5)  # Increased Kd_psi for strong damping

        self.Kp_roll = tk.DoubleVar(value=5.0)
        self.Ki_roll = tk.DoubleVar(value=1.0)
        self.Kd_roll = tk.DoubleVar(value=0.5)

        # Disturbance controls (Default to Step)
        self.dist_type = tk.StringVar(value="Step")  # Default to "Step"
        self.dist_time = tk.DoubleVar(value=2.0)
        self.dist_mag_alt = tk.DoubleVar(value=50.0)  # meters (for altitude disturbance)
        self.dist_mag_psi = tk.DoubleVar(value=5.0)  # degrees (for heading disturbance)
        self.dist_duration = tk.DoubleVar(value=0.5)  # disturbance duration

        # Data arrays for plotting
        self.time_history = []
        self.alt_history = []
        self.alt_ref_history = []
        self.psi_history = []
        self.psi_ref_history = []
        self.theta_history = []
        self.phi_history = []
        self.elevator_history = []
        self.aileron_history = []

        # Build GUI
        self.setup_gui()
        self.reset_simulation_state()

    def setup_gui(self):
        default_font = tkfont.nametofont("TkDefaultFont")
        default_font.configure(size=11)

        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=6, pady=6)

        # Add center heading
        heading_frame = ttk.Frame(main_frame)
        heading_frame.pack(fill=tk.X, pady=(0, 10))
        
        heading_label = ttk.Label(heading_frame, 
                                 text="Interactive Aircraft Control System Visualizer",
                                 font=("Arial", 16, "bold"),
                                 anchor=tk.CENTER)
        heading_label.pack(fill=tk.X)

        # Left controls
        control_frame = ttk.Frame(main_frame, width=420)
        control_frame.pack(side=tk.LEFT, fill=tk.Y, padx=(0,8))
        control_frame.pack_propagate(False)

        # Desired setpoints
        sp_frame = ttk.LabelFrame(control_frame, text="Setpoints & Simulation", padding=6)
        sp_frame.pack(fill=tk.X, pady=6)
        ttk.Label(sp_frame, text="Desired Altitude (m):").pack(anchor=tk.W)
        ttk.Entry(sp_frame, textvariable=self.desired_alt).pack(fill=tk.X)
        ttk.Label(sp_frame, text="Desired Heading (deg):").pack(anchor=tk.W, pady=(6,0))
        ttk.Entry(sp_frame, textvariable=self.desired_psi).pack(fill=tk.X)
        ttk.Label(sp_frame, text="Sim Speed (1 = real-time):").pack(anchor=tk.W, pady=(6,0))
        ttk.Scale(sp_frame, from_=0.1, to=5.0, variable=self.sim_speed, orient=tk.HORIZONTAL).pack(fill=tk.X)
        ttk.Checkbutton(sp_frame, text="Fast Simulate (Skip Visual Updates)", variable=self.fast_sim_mode).pack(anchor=tk.W, pady=(6,0))


        # PID gains
        pid_frame = ttk.LabelFrame(control_frame, text="PID Gains (Outer / Inner)", padding=6)
        pid_frame.pack(fill=tk.X, pady=6)

        # Altitude PID (outer)
        ttk.Label(pid_frame, text="Altitude PID (Kp, Ki, Kd)").pack(anchor=tk.W)
        self._add_three_entries(pid_frame, self.Kp_alt, self.Ki_alt, self.Kd_alt)

        ttk.Label(pid_frame, text="Pitch PID (Kp, Ki, Kd)").pack(anchor=tk.W, pady=(6,0))
        self._add_three_entries(pid_frame, self.Kp_pitch, self.Ki_pitch, self.Kd_pitch)

        ttk.Label(pid_frame, text="Heading PID (Kp, Ki, Kd)").pack(anchor=tk.W, pady=(6,0))
        self._add_three_entries(pid_frame, self.Kp_psi, self.Ki_psi, self.Kd_psi)

        ttk.Label(pid_frame, text="Roll PID (Kp, Ki, Kd)").pack(anchor=tk.W, pady=(6,0))
        self._add_three_entries(pid_frame, self.Kp_roll, self.Ki_roll, self.Kd_roll)

        # Disturbance frame
        dist_frame = ttk.LabelFrame(control_frame, text="Disturbance", padding=6)
        dist_frame.pack(fill=tk.X, pady=6)
        ttk.Label(dist_frame, text="Type:").pack(anchor=tk.W)
        ttk.Combobox(dist_frame, textvariable=self.dist_type, values=["None", "Step", "Impulse", "Sinusoid"], state="readonly").pack(fill=tk.X)
        ttk.Label(dist_frame, text="Disturbance Time (s):").pack(anchor=tk.W, pady=(6,0))
        ttk.Entry(dist_frame, textvariable=self.dist_time).pack(fill=tk.X)
        ttk.Label(dist_frame, text="Altitude Magnitude (m):").pack(anchor=tk.W, pady=(6,0))
        ttk.Entry(dist_frame, textvariable=self.dist_mag_alt).pack(fill=tk.X)
        ttk.Label(dist_frame, text="Heading Magnitude (deg):").pack(anchor=tk.W, pady=(6,0))
        ttk.Entry(dist_frame, textvariable=self.dist_mag_psi).pack(fill=tk.X)
        ttk.Label(dist_frame, text="Duration (s):").pack(anchor=tk.W, pady=(6,0))
        ttk.Entry(dist_frame, textvariable=self.dist_duration).pack(fill=tk.X)

        # Buttons
        btn_frame = ttk.Frame(control_frame)
        btn_frame.pack(fill=tk.X, pady=(10,0))
        ttk.Button(btn_frame, text="Start", command=self.start_simulation).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(0,4))
        ttk.Button(btn_frame, text="Stop", command=self.stop_simulation).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=4)
        ttk.Button(btn_frame, text="Reset", command=self.reset_all).pack(side=tk.LEFT, expand=True, fill=tk.X, padx=(4,0))

        # Plot frame on right
        plot_frame = ttk.Frame(main_frame)
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        # Create figure with 3 rows:
        # 1) Altitude reference vs altitude
        # 2) Heading reference vs heading
        # 3) Pitch / Roll angles and actuator commands
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3,1, figsize=(9,8))
        self.fig.subplots_adjust(hspace=0.6)

        self.canvas = FigureCanvasTkAgg(self.fig, plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _add_three_entries(self, parent, var1, var2, var3):
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, pady=(2,2))
        ttk.Entry(frame, textvariable=var1, width=8).pack(side=tk.LEFT, padx=2)
        ttk.Entry(frame, textvariable=var2, width=8).pack(side=tk.LEFT, padx=2)
        ttk.Entry(frame, textvariable=var3, width=8).pack(side=tk.LEFT, padx=2)

    # -------------------
    # Simulation helpers
    # -------------------
    def reset_simulation_state(self):
        # Reset dynamics state
        self.theta = 0.0
        self.theta_dot = 0.0
        self.phi = 0.0
        self.phi_dot = 0.0

        # Altitude & heading
        self.altitude = float(self.desired_alt.get())  # start at desired altitude for quicker demo
        self.heading = float(self.desired_psi.get())

        # Controller internals
        self.C_alt_int = 0.0
        self.C_pitch_int = 0.0
        self.C_psi_int = 0.0
        self.C_roll_int = 0.0
        self.last_err_alt = 0.0
        self.last_err_pitch = 0.0
        self.last_err_psi = 0.0
        self.last_err_roll = 0.0

        # Data histories
        self.time_history = []
        self.alt_history = []
        self.alt_ref_history = []
        self.psi_history = []
        self.psi_ref_history = []
        self.theta_history = []
        self.phi_history = []
        self.elevator_history = []
        self.aileron_history = []

        self.sim_time = 0.0

        self.update_plots()

    def reset_all(self):
        self.stop_simulation()
        self.reset_simulation_state()

    def start_simulation(self):
        if self.is_running:
            messagebox.showinfo("Simulation", "Already running")
            return
        # Reset history and integrators (optionally)
        self.reset_simulation_state()
        self.is_running = True
        self.sim_thread = threading.Thread(target=self.simulation_loop, daemon=True)
        self.sim_thread.start()

    def stop_simulation(self):
        self.is_running = False
        if self.sim_thread:
            self.sim_thread.join(timeout=0.1)
            self.sim_thread = None

    # Simple PID implementation
    def pid_compute(self, setpoint, measurement, Kp, Ki, Kd, integrator, last_err):
        err = setpoint - measurement
        integrator += err * self.dt
        derivative = (err - last_err) / self.dt if self.dt > 0 else 0.0
        output = Kp * err + Ki * integrator + Kd * derivative
        return output, integrator, err

    def apply_disturbance(self, t):
        """Return disturbance contributions to altitude (m) and heading (deg) at time t"""
        typ = self.dist_type.get()
        t0 = float(self.dist_time.get())
        mag_alt = float(self.dist_mag_alt.get())
        mag_psi = float(self.dist_mag_psi.get())
        duration = float(self.dist_duration.get())

        d_alt = 0.0
        d_psi = 0.0

        if typ == "None":
            return d_alt, d_psi

        if typ == "Impulse":
            # short pulse at t0 (modeled as a short step for duration)
            if t0 <= t < t0 + max(0.01, duration):
                d_alt = mag_alt
                d_psi = mag_psi
        elif typ == "Step":
            if t >= t0 and t < t0 + duration:
                d_alt = mag_alt
                d_psi = mag_psi
        elif typ == "Sinusoid":
            # Add duration limit for sinusoidal disturbance
            if t >= t0 and t < t0 + duration:
                # sinusoidal disturbance for specified duration
                d_alt = mag_alt * np.sin(2.0 * np.pi * 0.5 * (t - t0))
                d_psi = mag_psi * np.sin(2.0 * np.pi * 0.4 * (t - t0))
        return d_alt, d_psi

    # -------------------
    # Plant dynamics step
    # -------------------
    def plant_step(self, elevator_cmd, aileron_cmd, disturbance_alt, disturbance_psi):
        """
        Simple continuous-time dynamics integrated with Euler method:
          - Pitch: second-order system driven by elevator_cmd
          - Roll: second-order system driven by aileron_cmd
          - Altitude: integrate vertical speed ~ k_h * theta  (theta in radians)
          - Heading: integrate heading rate ~ k_psi * phi (phi in radians)
        """
        # Pitch (theta) second-order: theta_ddot + 2*zeta*wn*theta_dot + wn^2*theta = k_theta * elevator_cmd
        theta_ddot = (self.k_theta * elevator_cmd - 2.0 * self.zeta_theta * self.wn_theta * self.theta_dot - (self.wn_theta ** 2) * self.theta)
        # Roll (phi) second-order
        phi_ddot = (self.k_phi * aileron_cmd - 2.0 * self.zeta_phi * self.wn_phi * self.phi_dot - (self.wn_phi ** 2) * self.phi)

        # Integrate pitch & roll
        self.theta_dot += theta_ddot * self.dt
        self.theta += self.theta_dot * self.dt

        self.phi_dot += phi_ddot * self.dt
        self.phi += self.phi_dot * self.dt

        # Kinematic update: altitude and heading
        # small angle approximation: vertical speed ~ V * theta (theta in radians)
        self.altitude += (self.k_h * self.theta) * self.dt + disturbance_alt * self.dt
        # heading in degrees: heading rate = k_psi * phi (phi in radians). convert to degrees/sec
        self.heading += (self.k_psi * self.phi) * (180.0 / np.pi) * self.dt + disturbance_psi * self.dt

        # Normalize heading to [-180,180)
        self.heading = ((self.heading + 180) % 360) - 180

    # -------------------
    # Main simulation loop
    # -------------------
    def simulation_loop(self):
        # local aliases for speed
        dt = self.dt
        last_fast_plot_time = 0.0
        fast_plot_interval = 50.0 # Update plots every 50 seconds in fast mode

        while self.is_running and self.sim_time < self.max_time:
            t0_loop_start = time.time() # To measure actual loop execution time

            # references
            alt_ref = float(self.desired_alt.get())
            psi_ref = float(self.desired_psi.get())

            # Disturbance at this sim time
            d_alt, d_psi = self.apply_disturbance(self.sim_time)

            # OUTER ALTITUDE LOOP -> pitch reference (in radians)
            # error in altitude (m)
            err_alt = alt_ref - self.altitude
            alt_cmd, self.C_alt_int, self.last_err_alt = self.pid_compute(
                alt_ref, self.altitude,
                float(self.Kp_alt.get()), float(self.Ki_alt.get()), float(self.Kd_alt.get()),
                self.C_alt_int, self.last_err_alt
            )
            # alt_cmd is desired pitch angle (units arbitrary). We'll clamp.
            pitch_ref = np.clip(alt_cmd, -0.2, 0.2)  # radians (~11.5 deg)

            # INNER PITCH LOOP -> elevator command
            pitch_cmd, self.C_pitch_int, self.last_err_pitch = self.pid_compute(
                pitch_ref, self.theta,
                float(self.Kp_pitch.get()), float(self.Ki_pitch.get()), float(self.Kd_pitch.get()),
                self.C_pitch_int, self.last_err_pitch
            )
            # elevator command (normalized)
            elevator = np.clip(pitch_cmd, -1.0, 1.0)

            # OUTER HEADING LOOP -> roll reference (radians)
            # convert heading units: psi_ref (deg) vs self.heading (deg)
            err_psi = (psi_ref - self.heading + 180) % 360 - 180  # shortest-direction error
            psi_cmd, self.C_psi_int, self.last_err_psi = self.pid_compute(
                psi_ref, self.heading,
                float(self.Kp_psi.get()), float(self.Ki_psi.get()), float(self.Kd_psi.get()),
                self.C_psi_int, self.last_err_psi
            )
            roll_ref = np.clip(np.deg2rad(psi_cmd), -0.35, 0.35)  # radians

            # INNER ROLL LOOP -> aileron command
            roll_cmd, self.C_roll_int, self.last_err_roll = self.pid_compute(
                roll_ref, self.phi,
                float(self.Kp_roll.get()), float(self.Ki_roll.get()), float(self.Kd_roll.get()),
                self.C_roll_int, self.last_err_roll
            )
            aileron = np.clip(roll_cmd, -1.0, 1.0)

            # Apply plant step
            self.plant_step(elevator, aileron, d_alt, d_psi)

            # Save histories
            self.time_history.append(self.sim_time)
            self.alt_history.append(self.altitude)
            self.alt_ref_history.append(alt_ref)
            self.psi_history.append(self.heading)
            self.psi_ref_history.append(psi_ref)

            self.theta_history.append(self.theta)
            self.phi_history.append(self.phi)
            self.elevator_history.append(elevator)
            self.aileron_history.append(aileron)

            # Update plot based on mode
            if self.fast_sim_mode.get():
                # Update periodically in fast mode
                if (self.sim_time - last_fast_plot_time) >= fast_plot_interval or self.sim_time + dt >= self.max_time:
                    self.update_plots()
                    last_fast_plot_time = self.sim_time
            else:
                # Update every N steps in real-time mode
                if len(self.time_history) % 2 == 0:
                    self.update_plots()

            # Advance time
            self.sim_time += dt

            # Control loop to maintain real-time relative to sim_speed (only if not in fast mode)
            if not self.fast_sim_mode.get():
                elapsed = time.time() - t0_loop_start
                sleep_time = max(0.0, (dt / float(self.sim_speed.get())) - elapsed)
                time.sleep(sleep_time)

        # Ensure a final plot update after simulation ends, especially for fast mode
        self.root.after(10, self.update_plots) # Use root.after for GUI update in main thread
        self.is_running = False

    # -------------------
    # Plotting
    # -------------------
    def update_plots(self):
        if len(self.time_history) == 0:
            for ax in (self.ax1, self.ax2, self.ax3):
                ax.clear()
            self.canvas.draw()
            return

        t = np.array(self.time_history)

        # Altitude plotSinusoid
        self.ax1.clear()
        self.ax1.plot(t, self.alt_ref_history, label='Alt Ref (m)')
        self.ax1.plot(t, self.alt_history, label='Altitude (m)')
        self.ax1.set_title('Altitude')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.legend()
        self.ax1.grid(True)

        # Heading plot
        self.ax2.clear()
        self.ax2.plot(t, self.psi_ref_history, label='Heading Ref (deg)')
        self.ax2.plot(t, self.psi_history, label='Heading (deg)')
        self.ax2.set_title('Heading')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.legend()
        self.ax2.grid(True)

        # Inner loops & actuators
        self.ax3.clear()
        self.ax3.plot(t, np.rad2deg(self.theta_history), label='Pitch θ (deg)')
        self.ax3.plot(t, np.rad2deg(self.phi_history), label='Roll φ (deg)')
        self.ax3.plot(t, self.elevator_history, label='Elevator cmd')
        self.ax3.plot(t, self.aileron_history, label='Aileron cmd')
        self.ax3.set_title('Inner-loop angles and actuator commands')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.legend()
        self.ax3.grid(True)

        self.fig.tight_layout()
        self.canvas.draw()

# ----------------------------
# Run the app
# ----------------------------
def main():
    root = tk.Tk()
    app = AircraftControlVisualizer(root)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.stop_simulation(), root.destroy()))
    root.mainloop()

if __name__ == "__main__":
    main()
