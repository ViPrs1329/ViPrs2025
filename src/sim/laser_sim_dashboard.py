# sim/laser_sim_dashboard.py
import tkinter as tk
from tkinter import ttk
import ntcore
import time
import threading

class LaserSimDashboard:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("EndEffector LaserCAN Simulator")
        self.root.geometry("600x500")  # Larger window
        
        # Add style
        self.style = ttk.Style()
        self.style.configure("TButton", font=("Arial", 10))
        self.style.configure("TLabel", font=("Arial", 10))
        self.style.configure("Header.TLabel", font=("Arial", 12, "bold"))
        
        # Initialize NetworkTables
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startClient4("LaserSimDashboard")
        inst.setServer("localhost")  # Connect to localhost for simulation
        
        # Setup tabs
        self.tab_control = ttk.Notebook(self.root)
        self.sensor_tab = ttk.Frame(self.tab_control)
        self.control_tab = ttk.Frame(self.tab_control)
        self.tab_control.add(self.sensor_tab, text="Sensors")
        self.tab_control.add(self.control_tab, text="Controls")
        self.tab_control.pack(expand=1, fill="both")
        
        # Create tables
        self.sim_table = inst.getTable("simulation")
        self.end_effector_table = inst.getTable("EndEffectorTest")
        
        # Setup the sensor tab
        self.setup_sensor_tab()
        
        # Setup the control tab
        self.setup_control_tab()
        
        # Create publishers
        self.entry_pub = self.end_effector_table.getDoubleTopic("sim_entry_sensor").publish()
        self.stop_pub = self.end_effector_table.getDoubleTopic("sim_stop_sensor").publish()
        
        # Subscribe to values
        self.coral_detected_sub = self.end_effector_table.getBooleanTopic("coral_detected").subscribe(False)
        self.coral_positioned_sub = self.end_effector_table.getBooleanTopic("coral_positioned").subscribe(False)
        
        # Initial values
        self.entry_pub.set(1000)
        self.stop_pub.set(1000)
        
        # Start periodic updates
        self.update_values()
        self.root.after(100, self.periodic_update)
        
        # Add a quit button
        quit_button = ttk.Button(self.root, text="Quit", command=self.root.destroy)
        quit_button.pack(pady=10)
        
    def setup_sensor_tab(self):
        """Set up the sensors tab with sliders and displays."""
        sensor_frame = ttk.LabelFrame(self.sensor_tab, text="Sensor Control")
        sensor_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        # Coral Entry Distance slider
        ttk.Label(sensor_frame, text="Coral Entry Distance (mm):", style="Header.TLabel").grid(row=0, column=0, sticky="w", padx=10, pady=5)
        self.entry_slider = ttk.Scale(sensor_frame, from_=0, to=2000, orient=tk.HORIZONTAL, length=400)
        self.entry_slider.set(1000)
        self.entry_slider.grid(row=1, column=0, sticky="ew", padx=10, pady=5)
        
        # Value display for entry sensor
        self.entry_value = tk.StringVar(value="1000 mm")
        ttk.Label(sensor_frame, textvariable=self.entry_value).grid(row=1, column=1, padx=10, pady=5)
        
        # Coral Stop Distance slider
        ttk.Label(sensor_frame, text="Coral Stop Distance (mm):", style="Header.TLabel").grid(row=2, column=0, sticky="w", padx=10, pady=5)
        self.stop_slider = ttk.Scale(sensor_frame, from_=0, to=2000, orient=tk.HORIZONTAL, length=400)
        self.stop_slider.set(1000)
        self.stop_slider.grid(row=3, column=0, sticky="ew", padx=10, pady=5)
        
        # Value display for stop sensor
        self.stop_value = tk.StringVar(value="1000 mm")
        ttk.Label(sensor_frame, textvariable=self.stop_value).grid(row=3, column=1, padx=10, pady=5)
        
        # Preset buttons
        preset_frame = ttk.LabelFrame(sensor_frame, text="Presets")
        preset_frame.grid(row=4, column=0, columnspan=2, sticky="ew", padx=10, pady=10)
        
        # "No coral" preset button
        no_coral_btn = ttk.Button(preset_frame, text="No Coral", command=self.preset_no_coral)
        no_coral_btn.grid(row=0, column=0, padx=5, pady=5)
        
        # "Coral at entrance" preset button
        coral_at_entrance_btn = ttk.Button(preset_frame, text="Coral at Entrance", command=self.preset_coral_at_entrance)
        coral_at_entrance_btn.grid(row=0, column=1, padx=5, pady=5)
        
        # "Coral in position" preset button
        coral_in_position_btn = ttk.Button(preset_frame, text="Coral in Position", command=self.preset_coral_in_position)
        coral_in_position_btn.grid(row=0, column=2, padx=5, pady=5)
        
        # Status indicators
        status_frame = ttk.LabelFrame(sensor_frame, text="Status")
        status_frame.grid(row=5, column=0, columnspan=2, sticky="ew", padx=10, pady=10)
        
        ttk.Label(status_frame, text="Coral Detected:").grid(row=0, column=0, padx=5, pady=5, sticky="w")
        self.coral_detected_indicator = ttk.Label(status_frame, text="No", foreground="red")
        self.coral_detected_indicator.grid(row=0, column=1, padx=5, pady=5, sticky="w")
        
        ttk.Label(status_frame, text="Coral Positioned:").grid(row=1, column=0, padx=5, pady=5, sticky="w")
        self.coral_positioned_indicator = ttk.Label(status_frame, text="No", foreground="red")
        self.coral_positioned_indicator.grid(row=1, column=1, padx=5, pady=5, sticky="w")
        
    def setup_control_tab(self):
        """Set up the controls tab for testing commands."""
        control_frame = ttk.LabelFrame(self.control_tab, text="Command Control")
        control_frame.pack(fill="both", expand=True, padx=10, pady=10)
        
        # Simulate button presses
        ttk.Label(control_frame, text="Simulated Button Presses:", style="Header.TLabel").grid(row=0, column=0, sticky="w", padx=10, pady=5)
        
        # Create a frame for the buttons
        button_frame = ttk.Frame(control_frame)
        button_frame.grid(row=1, column=0, padx=10, pady=5)
        
        # Simulate sequence
        ttk.Label(control_frame, text="Simulation Sequence:", style="Header.TLabel").grid(row=2, column=0, sticky="w", padx=10, pady=5)
        
        # Run a simulated sequence that automatically runs through different states
        self.run_sequence_btn = ttk.Button(control_frame, text="Run Complete Sequence", command=self.run_simulation_sequence)
        self.run_sequence_btn.grid(row=3, column=0, padx=10, pady=5, sticky="w")
        
        # Stop button for the sequence
        self.stop_sequence_btn = ttk.Button(control_frame, text="Stop Sequence", command=self.stop_simulation_sequence)
        self.stop_sequence_btn.grid(row=4, column=0, padx=10, pady=5, sticky="w")
        self.stop_sequence_btn.config(state="disabled")
        
        # Status of the simulation
        ttk.Label(control_frame, text="Sequence Status:").grid(row=5, column=0, padx=10, pady=5, sticky="w")
        self.sequence_status = tk.StringVar(value="Not running")
        ttk.Label(control_frame, textvariable=self.sequence_status).grid(row=5, column=1, padx=10, pady=5, sticky="w")
        
        # Sequence progress
        self.sequence_progress = ttk.Progressbar(control_frame, orient="horizontal", length=300, mode="determinate")
        self.sequence_progress.grid(row=6, column=0, columnspan=2, padx=10, pady=5, sticky="ew")
        
        # Running indication
        self.is_sequence_running = False
        self.sequence_thread = None
        
    def preset_no_coral(self):
        """Set sliders to indicate no coral present."""
        self.entry_slider.set(1000)
        self.stop_slider.set(1000)
        self.update_values()
        
    def preset_coral_at_entrance(self):
        """Set sliders to indicate coral at the entrance."""
        self.entry_slider.set(40)  # Less than CORAL_DETECTION_THRESHOLD
        self.stop_slider.set(1000)  # Still far from stop position
        self.update_values()
        
    def preset_coral_in_position(self):
        """Set sliders to indicate coral fully in position."""
        self.entry_slider.set(40)  # Less than CORAL_DETECTION_THRESHOLD
        self.stop_slider.set(20)   # Less than CORAL_STOP_THRESHOLD
        self.update_values()
        
    def update_values(self):
        """Update NetworkTables values from the sliders."""
        # Get values from sliders
        entry_val = float(self.entry_slider.get())
        stop_val = float(self.stop_slider.get())
        
        # Update text displays
        self.entry_value.set(f"{int(entry_val)} mm")
        self.stop_value.set(f"{int(stop_val)} mm")
        
        # Update NetworkTables
        self.entry_pub.set(entry_val)
        self.stop_pub.set(stop_val)
        
    def update_status_indicators(self):
        """Update status indicators based on NetworkTables values."""
        # Read values from NetworkTables
        is_detected = self.coral_detected_sub.get()
        is_positioned = self.coral_positioned_sub.get()
        
        # Update indicator labels
        if is_detected:
            self.coral_detected_indicator.config(text="Yes", foreground="green")
        else:
            self.coral_detected_indicator.config(text="No", foreground="red")
            
        if is_positioned:
            self.coral_positioned_indicator.config(text="Yes", foreground="green")
        else:
            self.coral_positioned_indicator.config(text="No", foreground="red")
            
    def periodic_update(self):
        """Run periodic updates of the GUI."""
        self.update_values()
        self.update_status_indicators()
        self.root.after(100, self.periodic_update)
        
    def run_simulation_sequence(self):
        """Run a simulation sequence that progresses through all states."""
        if self.is_sequence_running:
            return
            
        self.is_sequence_running = True
        self.run_sequence_btn.config(state="disabled")
        self.stop_sequence_btn.config(state="normal")
        self.sequence_status.set("Running")
        self.sequence_progress["value"] = 0
        
        # Start sequence in a new thread
        self.sequence_thread = threading.Thread(target=self.simulation_sequence_worker)
        self.sequence_thread.daemon = True  # Thread will exit when main program exits
        self.sequence_thread.start()
        
    def stop_simulation_sequence(self):
        """Stop the simulation sequence."""
        self.is_sequence_running = False
        self.run_sequence_btn.config(state="normal")
        self.stop_sequence_btn.config(state="disabled")
        self.sequence_status.set("Stopped")
        
    def simulation_sequence_worker(self):
        """Worker function for the simulation sequence (runs in separate thread)."""
        try:
            # Step 1: No coral (both sensors reading far)
            self.sequence_status.set("Step 1: No coral")
            self.sequence_progress["value"] = 0
            self.preset_no_coral()
            
            if not self.is_sequence_running:
                return
            time.sleep(2)
                
            # Step 2: Coral approaches the intake (entry sensor starts detecting)
            self.sequence_status.set("Step 2: Coral approaching")
            self.sequence_progress["value"] = 20
            for i in range(1000, 40, -50):
                if not self.is_sequence_running:
                    return
                self.entry_slider.set(i)
                time.sleep(0.1)
                
            # Step 3: Coral is at the entrance
            self.sequence_status.set("Step 3: Coral at entrance")
            self.sequence_progress["value"] = 40
            self.preset_coral_at_entrance()
            
            if not self.is_sequence_running:
                return
            time.sleep(2)
                
            # Step 4: Coral moves into the intake (stop sensor detects it approaching)
            self.sequence_status.set("Step 4: Coral moving in")
            self.sequence_progress["value"] = 60
            for i in range(1000, 20, -50):
                if not self.is_sequence_running:
                    return
                self.stop_slider.set(i)
                time.sleep(0.1)
                
            # Step 5: Coral is in position
            self.sequence_status.set("Step 5: Coral in position")
            self.sequence_progress["value"] = 80
            self.preset_coral_in_position()
            
            if not self.is_sequence_running:
                return
            time.sleep(2)
                
            # Step 6: Complete
            self.sequence_status.set("Sequence complete")
            self.sequence_progress["value"] = 100
            
            # Reset state
            self.root.after(0, lambda: self.run_sequence_btn.config(state="normal"))
            self.root.after(0, lambda: self.stop_sequence_btn.config(state="disabled"))
            self.is_sequence_running = False
            
        except Exception as e:
            self.root.after(0, lambda: self.sequence_status.set(f"Error: {e}"))
            self.is_sequence_running = False
            self.root.after(0, lambda: self.run_sequence_btn.config(state="normal"))
            self.root.after(0, lambda: self.stop_sequence_btn.config(state="disabled"))
        
    def run(self):
        """Run the GUI."""
        self.root.mainloop()

if __name__ == "__main__":
    dashboard = LaserSimDashboard()
    dashboard.run()