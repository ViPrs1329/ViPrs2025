# src/subsystems/BaseSubsystem.py
import commands2
import wpilib

class BaseSubsystem(commands2.Subsystem):
    """
    Base class for all robot subsystems that implements common functionality.
    
    This class provides:
    - Standard initialization pattern
    - Sensor caching to reduce CAN traffic
    - Consistent error handling
    - Simulation detection and support
    """
    
    def __init__(self, name: str) -> None:
        """
        Initialize the base subsystem.
        
        Args:
            name (str): The name of the subsystem for logging and dashboard
        """
        super().__init__()
        self.subsystem_name = name
        
        # Check if we're in simulation mode
        self.is_simulation = wpilib.RobotBase.isSimulation()
        
        # Initialize dashboard
        wpilib.SmartDashboard.putString(f"{name}/Status", "Initializing")
        
        # Create the sensor cache container
        self.sensor_cache = {}
        
        # Initialize periodic counter for less frequent operations
        self.periodic_counter = 0
        
        # Last error logged to prevent error flooding
        self.last_error = None
        
    def periodic(self) -> None:
        """Common periodic method called for all subsystems."""
        try:
            # Increment periodic counter
            self.periodic_counter = (self.periodic_counter + 1) % 100
            
            # Cache sensor values
            self.cacheSensors()
            
            # Call the subsystem-specific implementation
            self.subsystemPeriodic()
            
            # Update dashboard with status
            wpilib.SmartDashboard.putString(f"{self.subsystem_name}/Status", "OK")
        except Exception as e:
            self.handleError("periodic", e)
    
    def cacheSensors(self) -> None:
        """Cache sensor values to reduce CAN traffic. Override in subclasses."""
        pass
    
    def subsystemPeriodic(self) -> None:
        """Subsystem-specific periodic code. Override in subclasses."""
        pass
    
    def handleError(self, method_name: str, error: Exception) -> None:
        """
        Common error handling method.
        
        Args:
            method_name (str): The name of the method where the error occurred
            error (Exception): The exception that was thrown
        """
        # Avoid flooding the logs with the same error
        error_msg = f"{type(error).__name__}: {str(error)}"
        if error_msg != self.last_error:
            print(f"ERROR in {self.subsystem_name}.{method_name}: {error_msg}")
            self.last_error = error_msg
        
        # Update dashboard with error status
        wpilib.SmartDashboard.putString(f"{self.subsystem_name}/Status", f"ERROR: {type(error).__name__}")
        
        # Additional error handling can be added here (e.g., emergency stop)
        
    def simulationPeriodic(self) -> None:
        """Periodic simulation code. Override in subclasses if needed."""
        if self.is_simulation:
            try:
                self.subsystemSimulationPeriodic()
            except Exception as e:
                self.handleError("simulationPeriodic", e)
    
    def subsystemSimulationPeriodic(self) -> None:
        """Subsystem-specific simulation periodic code. Override in subclasses."""
        pass
    
    def resetSensors(self) -> None:
        """Reset subsystem sensors to initial state. Override in subclasses."""
        pass
    
    def stopMotors(self) -> None:
        """Stop all motors in the subsystem. Override in subclasses."""
        pass
    
    def getSubsystemName(self) -> str:
        """Get the name of the subsystem."""
        return self.subsystem_name