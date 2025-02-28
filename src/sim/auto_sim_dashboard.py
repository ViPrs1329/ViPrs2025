import tkinter as tk
import ntcore
import threading
import time

class AutonomousDashboard:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Autonomous Simulation Dashboard")
        
        # NetworkTables setup
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self.nt_instance.startClient4("AutoDashboard")
        self.nt_instance.setServer("localhost")
        
        self.auto_table = self.nt_instance.getTable("Autonomous")
        self.field_table = self.nt_instance.getTable("field")
        
        # UI Components
        self.setup_ui()
        
        # Start periodic update thread
        self.update_thread = threading.Thread(target=self.periodic_update, daemon=True)
        self.update_thread.start()
        
    def setup_ui(self):
        # Current Command Display
        tk.Label(self.root, text="Current Command:", font=("Arial", 12, "bold")).pack()
        self.command_var = tk.StringVar(value="Not Started")
        tk.Label(self.root, textvariable=self.command_var, font=("Arial", 10)).pack()
        
        # Robot Position Display
        tk.Label(self.root, text="Robot Position:", font=("Arial", 12, "bold")).pack()
        self.position_var = tk.StringVar(value="X: N/A, Y: N/A")
        tk.Label(self.root, textvariable=self.position_var, font=("Arial", 10)).pack()
        
        # Autonomous Progress
        tk.Label(self.root, text="Progress:", font=("Arial", 12, "bold")).pack()
        self.progress_var = tk.StringVar(value="0%")
        tk.Label(self.root, textvariable=self.progress_var, font=("Arial", 10)).pack()
        
        # Simulation Control Buttons
        tk.Button(self.root, text="Start Autonomous", command=self.start_autonomous).pack(pady=10)
        tk.Button(self.root, text="Stop Autonomous", command=self.stop_autonomous).pack(pady=10)
        
    def periodic_update(self):
        while True:
            try:
                # Update command name
                command_name = self.auto_table.getStringTopic("current_command").subscribe("None").get()
                self.command_var.set(command_name)
                
                # Update robot position
                robot_pose = self.field_table.getStructTopic("robot_pose", None).subscribe(None).get()
                if robot_pose:
                    position_str = f"X: {robot_pose.x:.2f}, Y: {robot_pose.y:.2f}"
                    self.position_var.set(position_str)
                
                # You could add progress tracking here
                
            except Exception as e:
                print(f"Update error: {e}")
            
            time.sleep(0.1)
        
    def start_autonomous(self):
        # Signal to start autonomous via NetworkTables
        start_pub = self.auto_table.getStringTopic("start_command").publish()
        start_pub.set("start")
        
    def stop_autonomous(self):
        # Signal to stop autonomous via NetworkTables
        stop_pub = self.auto_table.getStringTopic("stop_command").publish()
        stop_pub.set("stop")
        
    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    dashboard = AutonomousDashboard()
    dashboard.run()