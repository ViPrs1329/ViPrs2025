# Updates for src/sim/auto_sim_dashboard.py
import tkinter as tk
from tkinter import ttk
import ntcore
import threading
import time
import math

class AutonomousDashboard:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Autonomous Simulation Dashboard")
        self.root.geometry("800x600")
        
        # NetworkTables setup
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self.nt_instance.startClient4("AutoDashboard")
        self.nt_instance.setServer("localhost")
        
        self.auto_table = self.nt_instance.getTable("Autonomous")
        self.field_table = self.nt_instance.getTable("field")
        
        # Add dropdown for autonomous routine selection
        self.auto_routines = [
            "LeaveStartingZoneAuto",
            "ScorePreloadedCoralAutonomous", 
            "ComplexAutonomousRoutine"
        ]
        
        # UI Components
        self.setup_ui()
        
        # Start periodic update thread
        self.update_thread = threading.Thread(target=self.periodic_update, daemon=True)
        self.update_thread.start()
        
    def setup_ui(self):
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Left side - controls
        ctrl_frame = ttk.LabelFrame(main_frame, text="Controls", padding="10")
        ctrl_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Autonomous selection
        ttk.Label(ctrl_frame, text="Select Autonomous Routine:").pack(anchor="w", pady=5)
        self.routine_var = tk.StringVar(value=self.auto_routines[0])
        routine_dropdown = ttk.Combobox(ctrl_frame, textvariable=self.routine_var, values=self.auto_routines)
        routine_dropdown.pack(fill=tk.X, pady=5)
        
        # Command buttons
        btn_frame = ttk.Frame(ctrl_frame)
        btn_frame.pack(fill=tk.X, pady=10)
        
        ttk.Button(btn_frame, text="Select Routine", command=self.select_routine).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Start", command=self.start_autonomous).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Stop", command=self.stop_autonomous).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="Reset Field", command=self.reset_field).pack(side=tk.LEFT, padx=5)
        
        # Current status frame
        status_frame = ttk.LabelFrame(ctrl_frame, text="Current Status", padding="10")
        status_frame.pack(fill=tk.X, pady=10)
        
        # Current Command Display
        ttk.Label(status_frame, text="Command:").grid(row=0, column=0, sticky="w", pady=2)
        self.command_var = tk.StringVar(value="Not Started")
        ttk.Label(status_frame, textvariable=self.command_var).grid(row=0, column=1, sticky="w", pady=2)
        
        # Robot Position Display
        ttk.Label(status_frame, text="Position:").grid(row=1, column=0, sticky="w", pady=2)
        self.position_var = tk.StringVar(value="X: 0.00, Y: 0.00, θ: 0.00°")
        ttk.Label(status_frame, textvariable=self.position_var).grid(row=1, column=1, sticky="w", pady=2)
        
        # Progress Display
        ttk.Label(status_frame, text="Progress:").grid(row=2, column=0, sticky="w", pady=2)
        self.progress_var = tk.StringVar(value="0%")
        ttk.Label(status_frame, textvariable=self.progress_var).grid(row=2, column=1, sticky="w", pady=2)
        
        # Right side - field visualization (simple)
        field_frame = ttk.LabelFrame(main_frame, text="Field", padding="10")
        field_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Simple field visualization using canvas
        self.field_canvas = tk.Canvas(field_frame, bg="white", width=400, height=400)
        self.field_canvas.pack(fill=tk.BOTH, expand=True)
        
        # Draw field boundaries (16.5 x 8.2 meters)
        field_color = "#0C7C59"  # Dark green
        self.field_canvas.create_rectangle(50, 50, 350, 350, outline=field_color, width=2)
        
        # Create robot marker
        self.robot_marker = self.field_canvas.create_oval(195, 195, 205, 205, fill="blue")
        self.robot_heading = self.field_canvas.create_line(200, 200, 210, 200, fill="red", width=2)
        
    # Updates for the periodic_update method in auto_sim_dashboard.py

    # Update the position reading in auto_sim_dashboard.py's periodic_update method

    def periodic_update(self):
        while True:
            try:
                # Update command name
                command_name = self.auto_table.getStringTopic("current_command").subscribe("None").get()
                self.command_var.set(command_name)
                
                # Debug all available NetworkTables values
                print("Checking available NetworkTables entries:")
                field_entries = self.field_table.getEntries()
                for entry in field_entries:
                    print(f"  Field table entry: {entry.getName()} = {entry.getValue()}")
                
                # Try multiple ways to get the robot position
                robot_x = self.field_table.getDoubleTopic("robot_x").subscribe(0.0).get()
                robot_y = self.field_table.getDoubleTopic("robot_y").subscribe(0.0).get()
                robot_rotation = self.field_table.getDoubleTopic("robot_rotation").subscribe(0.0).get()
                
                print(f"Read from NT - x: {robot_x:.2f}, y: {robot_y:.2f}, rot: {robot_rotation:.2f}")
                
                # Update position text
                self.position_var.set(f"X: {robot_x:.2f}, Y: {robot_y:.2f}, θ: {robot_rotation:.1f}°")
                
                # Update visualization
                x_canvas = 50 + (robot_x / 16.5) * 300
                y_canvas = 50 + (robot_y / 8.2) * 300
                angle_rad = math.radians(robot_rotation)
                
                # Update robot marker position
                self.field_canvas.coords(self.robot_marker, x_canvas-5, y_canvas-5, x_canvas+5, y_canvas+5)
                
                # Update robot heading line
                heading_x = x_canvas + 10 * math.cos(angle_rad)
                heading_y = y_canvas + 10 * math.sin(angle_rad)
                self.field_canvas.coords(self.robot_heading, x_canvas, y_canvas, heading_x, heading_y)
                
                # Update progress
                progress = self.auto_table.getDoubleTopic("progress").subscribe(0).get()
                self.progress_var.set(f"{progress:.0f}%")
            
            except Exception as e:
                print(f"Update error: {e}")
            
            time.sleep(0.1)
    
    def select_routine(self):
        # Send the selected routine to NetworkTables
        selected = self.routine_var.get()
        selection_pub = self.auto_table.getStringTopic("selected_routine").publish()
        selection_pub.set(selected)
        print(f"Selected routine: {selected}")
        
    def start_autonomous(self):
        # Signal to start autonomous via NetworkTables
        start_pub = self.auto_table.getStringTopic("start_command").publish()
        start_pub.set("start")
        print("Start command sent")
        
    def stop_autonomous(self):
        # Signal to stop autonomous via NetworkTables
        stop_pub = self.auto_table.getStringTopic("stop_command").publish()
        stop_pub.set("stop")
        print("Stop command sent")
        
    # Updated reset_field method in auto_sim_dashboard.py

    def reset_field(self):
        # Reset robot position to starting position
        reset_pub = self.auto_table.getStringTopic("reset_field").publish()
        reset_pub.set("reset")
        print("Field reset command sent")
        
        # Immediately update visualization to give user feedback
        try:
            # Reset visualization to center position
            x_canvas, y_canvas = 200, 200
            self.field_canvas.coords(self.robot_marker, x_canvas-5, y_canvas-5, x_canvas+5, y_canvas+5)
            self.field_canvas.coords(self.robot_heading, x_canvas, y_canvas, x_canvas+10, y_canvas)
            self.position_var.set("X: 2.00, Y: 2.00, θ: 0.00°")
        except Exception as e:
            print(f"Error updating reset visualization: {e}")
        
    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    dashboard = AutonomousDashboard()
    dashboard.run()