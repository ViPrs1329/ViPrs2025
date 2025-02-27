# test_endeffector.py
import wpilib
import commands2
import ntcore
from subsystems.EndEffector import EndEffector
from commands.IntakeCommands import IntakeCoralCommand

class EndEffectorTestRobot(wpilib.TimedRobot):
    """
    This is a simple test program for the EndEffector subsystem.
    It can be run in simulation mode to test the EndEffector functionality.
    """
    
    def robotInit(self):
        """Robot initialization code."""
        print("Initializing EndEffector Test Robot")
        
        # Create the subsystem and command
        self.endEffector = EndEffector()
        self.intakeCoralCommand = IntakeCoralCommand(self.endEffector)
        
        # Create a joystick for control
        self.controller = wpilib.XboxController(0)
        
        # Create NetworkTables for simulation control and monitoring
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self.table = self.nt_instance.getTable("EndEffectorTest")
        
        # Create publishers for sensor values
        self.entry_sensor_pub = self.table.getDoubleTopic("entry_sensor_mm").publish()
        self.stop_sensor_pub = self.table.getDoubleTopic("stop_sensor_mm").publish()
        self.is_coral_detected_pub = self.table.getBooleanTopic("coral_detected").publish()
        self.is_coral_positioned_pub = self.table.getBooleanTopic("coral_positioned").publish()
        
        # Create subscribers for simulation control
        self.entry_sensor_sub = self.table.getDoubleTopic("sim_entry_sensor").subscribe(8000)
        self.stop_sensor_sub = self.table.getDoubleTopic("sim_stop_sensor").subscribe(8000)
        
        # Initialize values
        self.entry_sensor_pub.set(8000)
        self.stop_sensor_pub.set(8000)
        self.is_coral_detected_pub.set(False)
        self.is_coral_positioned_pub.set(False)
        
        # Command scheduler
        self.scheduler = commands2.CommandScheduler.getInstance()
        
        print("EndEffector Test Robot initialized")

    def robotPeriodic(self):
        """Periodic code for all robot modes."""
        # Run the command scheduler
        self.scheduler.run()
        
        # Update NetworkTables with sensor values
        if hasattr(self.endEffector, 'coral_intake_LC') and hasattr(self.endEffector.coral_intake_LC, 'get_measurement'):
            measurement = self.endEffector.coral_intake_LC.get_measurement()
            if measurement:
                distance, status = measurement
                self.entry_sensor_pub.set(distance)
                
        if hasattr(self.endEffector, 'coral_stop_LC') and hasattr(self.endEffector.coral_stop_LC, 'get_measurement'):
            measurement = self.endEffector.coral_stop_LC.get_measurement()
            if measurement:
                distance, status = measurement
                self.stop_sensor_pub.set(distance)
        
        # Update detection status
        self.is_coral_detected_pub.set(self.endEffector.isCoralDetected())
        self.is_coral_positioned_pub.set(self.endEffector.isCoralPositioned())
        
        # In simulation, update the simulated sensors
        if wpilib.RobotBase.isSimulation():
            self.simulationPeriodic()

    def teleopInit(self):
        """Initialization code for teleop mode."""
        print("Teleop initialized")

    def teleopPeriodic(self):
        """Periodic code for teleop mode."""
        # Check if button A is pressed to start the intake command
        if self.controller.getAButtonPressed():
            print("A button pressed - scheduling IntakeCoralCommand")
            self.scheduler.schedule(self.intakeCoralCommand)
        
        # Check if button B is pressed to cancel the intake command
        if self.controller.getBButtonPressed():
            print("B button pressed - canceling IntakeCoralCommand")
            self.scheduler.cancel(self.intakeCoralCommand)

    def simulationInit(self):
        """Initialization code for simulation mode."""
        print("Simulation initialized")

    def simulationPeriodic(self):
        """Periodic code for simulation mode."""
        # Update simulated sensors from NetworkTables
        if hasattr(self.endEffector, 'coral_intake_LC') and hasattr(self.endEffector.coral_intake_LC, 'set_simulated_distance'):
            entry_distance = self.entry_sensor_sub.get()
            self.endEffector.coral_intake_LC.set_simulated_distance(entry_distance)
            
        if hasattr(self.endEffector, 'coral_stop_LC') and hasattr(self.endEffector.coral_stop_LC, 'set_simulated_distance'):
            stop_distance = self.stop_sensor_sub.get()
            self.endEffector.coral_stop_LC.set_simulated_distance(stop_distance)

if __name__ == "__main__":
    wpilib.run(EndEffectorTestRobot)