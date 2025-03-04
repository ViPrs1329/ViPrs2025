from wpilib import SubsystemBase
from typing import Dict, Any, Optional
from utils.telemetry import TelemetryManager

class TelemetrySubsystemBase(SubsystemBase):
    """
    Base class for subsystems that integrates telemetry functionality.
    Combines the caching system with telemetry logging.
    """
    
    def __init__(self, telemetry: TelemetryManager, subsystem_name: str):
        """
        Initialize the telemetry-enabled subsystem.
        
        Args:
            telemetry: The telemetry manager instance
            subsystem_name: Name of this subsystem
        """
        super().__init__()
        self.telemetry = telemetry
        self.subsystem_name = subsystem_name
        self.telemetry_table = telemetry.register_subsystem(subsystem_name)
        
        # Dictionary to store cached values
        self.cache: Dict[str, Any] = {}
        
        # Initialize health monitoring
        self._setup_health_monitoring()
    
    def _setup_health_monitoring(self) -> None:
        """
        Set up health monitoring thresholds for this subsystem.
        Override this in derived classes to add specific thresholds.
        """
        pass
    
    def cache_sensors(self) -> None:
        """
        Cache sensor readings and log them to telemetry.
        Override this in derived classes.
        """
        pass
    
    def periodic_logic(self) -> None:
        """
        Perform periodic computations and updates.
        Override this in derived classes.
        """
        pass
    
    def update_hardware(self) -> None:
        """
        Update hardware with cached values.
        Override this in derived classes.
        """
        pass
    
    def periodic(self) -> None:
        """
        Periodic update method that runs every robot loop.
        Handles caching, telemetry, and hardware updates.
        """
        # Cache sensor values
        self.cache_sensors()
        
        # Run periodic logic
        self.periodic_logic()
        
        # Update hardware
        self.update_hardware()
        
        # Log cached values to telemetry
        self.telemetry.log_subsystem_state(self.subsystem_name, self.cache)
    
    def log_event(self, event_name: str, details: str = "") -> None:
        """
        Log an event for this subsystem.
        
        Args:
            event_name: Name of the event
            details: Additional event details
        """
        self.telemetry.log_event(self.subsystem_name, event_name, details)
    
    def register_health_threshold(self, key: str, 
                                min_value: Optional[float] = None,
                                max_value: Optional[float] = None,
                                warning_min: Optional[float] = None,
                                warning_max: Optional[float] = None) -> None:
        """
        Register a health monitoring threshold for this subsystem.
        
        Args:
            key: Data key to monitor
            min_value: Minimum acceptable value
            max_value: Maximum acceptable value
            warning_min: Warning threshold minimum
            warning_max: Warning threshold maximum
        """
        self.telemetry.register_health_threshold(
            self.subsystem_name, key,
            min_value, max_value,
            warning_min, warning_max
        )
    
    def get_cached(self, key: str, default: Any = None) -> Any:
        """
        Get a cached value.
        
        Args:
            key: Cache key
            default: Default value if key not found
            
        Returns:
            The cached value or default
        """
        return self.cache.get(key, default)
    
    def set_cached(self, key: str, value: Any) -> None:
        """
        Set a cached value.
        
        Args:
            key: Cache key
            value: Value to cache
        """
        self.cache[key] = value 