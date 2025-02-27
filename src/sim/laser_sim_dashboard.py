import tkinter as tk
import ntcore

class LaserSimDashboard:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("LaserCAN Simulator")
        
        # Initialize NetworkTables
        inst = ntcore.NetworkTableInstance.getDefault()
        inst.startClient4("LaserSimDashboard")
        inst.setServer("localhost")  # Connect to localhost for simulation
        self.sim_table = inst.getTable("simulation")
        
        # Create sliders for distances
        tk.Label(self.root, text="Coral Entry Distance (mm):").pack()
        self.entry_slider = tk.Scale(self.root, from_=0, to=2000, orient=tk.HORIZONTAL, length=300)
        self.entry_slider.set(1000)
        self.entry_slider.pack()
        
        tk.Label(self.root, text="Coral Stop Distance (mm):").pack()
        self.stop_slider = tk.Scale(self.root, from_=0, to=2000, orient=tk.HORIZONTAL, length=300)
        self.stop_slider.set(1000)
        self.stop_slider.pack()
        
        # Update button
        update_button = tk.Button(self.root, text="Update Values", command=self.update_values)
        update_button.pack(pady=10)
        
        # Create publishers
        self.entry_pub = self.sim_table.getDoubleTopic("coral_entry_distance").publish()
        self.stop_pub = self.sim_table.getDoubleTopic("coral_stop_distance").publish()
        
        # Start periodic updates
        self.update_values()
        self.root.after(100, self.periodic_update)
        
    def update_values(self):
        self.entry_pub.set(float(self.entry_slider.get()))
        self.stop_pub.set(float(self.stop_slider.get()))
        
    def periodic_update(self):
        self.update_values()
        self.root.after(100, self.periodic_update)
        
    def run(self):
        self.root.mainloop()

if __name__ == "__main__":
    dashboard = LaserSimDashboard()
    dashboard.run()