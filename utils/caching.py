"""
Utility classes for implementing sensor and actuator caching in subsystems.
This helps reduce CAN bus traffic by batching reads and writes.
"""

from commands2 import SubsystemBase
from typing import Any, Dict

class CachingSubsystemBase(SubsystemBase):
    """
    Base class for subsystems that implement sensor and actuator caching.
    Reduces CAN bus traffic by batching reads and writes.
    """
    
    class Cache:
        """Base cache class that can be extended by specific subsystems"""
        def __init__(self):
            self._cached_values: Dict[str, Any] = {}
            self._setpoints: Dict[str, Any] = {}
        
        def get_cached(self, key: str, default: Any = None) -> Any:
            """Get a cached value"""
            return self._cached_values.get(key, default)
            
        def set_cached(self, key: str, value: Any) -> None:
            """Set a cached value"""
            self._cached_values[key] = value
            
        def get_setpoint(self, key: str, default: Any = None) -> Any:
            """Get a setpoint value"""
            return self._setpoints.get(key, default)
            
        def set_setpoint(self, key: str, value: Any) -> None:
            """Set a setpoint value"""
            self._setpoints[key] = value

    def __init__(self):
        """Initialize the caching subsystem"""
        super().__init__()
        self.cache = self.Cache()
        
    def periodic(self) -> None:
        """
        Override of SubsystemBase.periodic()
        Implements the caching pattern:
        1. Cache all sensor values
        2. Run normal periodic logic
        3. Update all hardware setpoints
        """
        self.cache_sensors()
        self.periodic_logic()
        self.update_hardware()
        
    def cache_sensors(self) -> None:
        """Cache all sensor values - Override in subclass"""
        pass
        
    def periodic_logic(self) -> None:
        """Normal periodic logic - Override in subclass"""
        pass
        
    def update_hardware(self) -> None:
        """Update hardware with cached setpoints - Override in subclass"""
        pass 