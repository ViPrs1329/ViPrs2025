# src/tests/test_end_effector.py
import wpilib
import commands2
from subsystems.EndEffector import EndEffector
import ntcore

def main():
    # Initialize as simulation
    wpilib.run(EndEffectorTestRobot)

class EndEffectorTestRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.end_effector = EndEffector()
        
        # Create controls
        inst = ntcore.NetworkTableInstance.getDefault()
        self.test_table = inst.getTable("end_effector_test")
        self.test_intake = self.test_table.getBooleanTopic("start_intake").publish()
        self.test_intake.set(False)
        
    def teleopPeriodic(self):
        # Read control from NetworkTables
        should_intake = self.test_table.getBooleanTopic("start_intake").subscribe(False).get()
        
        if should_intake:
            intake_done = self.end_effector.intakeCoral()
            if intake_done:
                self.test_intake.set(False)  # Reset when done
        else:
            self.end_effector.stopCoralIntake()
            
        # Print sensor values
        entry_measurement = self.end_effector.coral_intake_LC.get_measurement()
        stop_measurement = self.end_effector.coral_stop_LC.get_measurement()
        
        if entry_measurement and stop_measurement:
            print(f"Entry: {entry_measurement[0]}mm, Stop: {stop_measurement[0]}mm")

if __name__ == "__main__":
    main()