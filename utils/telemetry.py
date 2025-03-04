import wpilib
import ntcore
from wpilib import DataLogManager, DriverStation
from wpimath.geometry import Pose2d, Rotation2d
import time
import math
from typing import Dict, Any, List, Optional, Callable, TypeVar, Generic, Union

T = TypeVar('T')

class TelemetryManager:
    """
    Manages telemetry data collection, logging, and publishing to NetworkTables.
    Integrates with AdvantageScope for visualization and debugging.
    """
    
    def __init__(self, robot_name: str = "Robot"):
        """
        Initialize the telemetry manager.
        
        Args:
            robot_name: Name of the robot for logging purposes
        """
        self.robot_name = robot_name
        self.start_time = time.time()
        
        # Initialize NetworkTables
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        self.telemetry_table = self.nt_instance.getTable("Telemetry")
        
        # Initialize subsystem tables
        self.subsystem_tables: Dict[str, ntcore.NetworkTable] = {}
        
        # Initialize data logging
        DataLogManager.start()
        
        # Log both DS control and joystick data
        DriverStation.startDataLog(DataLogManager.getLog())
        
        # Statistics tracking
        self.stats_tracking: Dict[str, Dict[str, Any]] = {}
        
        # Health monitoring thresholds
        self.health_thresholds: Dict[str, Dict[str, Dict[str, float]]] = {}
        
        # Log startup
        self._log_event("TelemetryManager", "Initialized")
        print(f"Telemetry Manager initialized for {robot_name}")
    
    def register_subsystem(self, subsystem_name: str) -> ntcore.NetworkTable:
        """
        Register a subsystem for telemetry.
        
        Args:
            subsystem_name: Name of the subsystem
            
        Returns:
            NetworkTable for the subsystem
        """
        if subsystem_name not in self.subsystem_tables:
            self.subsystem_tables[subsystem_name] = self.telemetry_table.getSubTable(subsystem_name)
            self._log_event("TelemetryManager", f"Registered subsystem: {subsystem_name}")
        
        return self.subsystem_tables[subsystem_name]
    
    def log_data(self, subsystem_name: str, key: str, value: Any) -> None:
        """
        Log data to both NetworkTables and the data log.
        
        Args:
            subsystem_name: Name of the subsystem
            key: Data key
            value: Data value
        """
        # Ensure subsystem is registered
        if subsystem_name not in self.subsystem_tables:
            self.register_subsystem(subsystem_name)
        
        table = self.subsystem_tables[subsystem_name]
        
        # Publish to NetworkTables based on type
        if isinstance(value, bool):
            table.putBoolean(key, value)
        elif isinstance(value, (int, float)):
            table.putNumber(key, value)
            # Update statistics for numerical values
            self._update_statistics(subsystem_name, key, value)
            # Check health thresholds
            self._check_health_threshold(subsystem_name, key, value)
        elif isinstance(value, str):
            table.putString(key, value)
        elif isinstance(value, Pose2d):
            # Handle Pose2d for odometry
            pose_table = table.getSubTable(key)
            pose_table.putNumber("x", value.X())
            pose_table.putNumber("y", value.Y())
            pose_table.putNumber("rotation", value.rotation().degrees())
        elif isinstance(value, list) and all(isinstance(x, (int, float)) for x in value):
            # Handle numeric arrays
            table.putNumberArray(key, value)
        else:
            # Convert to string for unsupported types
            table.putString(key, str(value))
    
    def log_event(self, subsystem_name: str, event_name: str, details: str = "") -> None:
        """
        Log an event with optional details.
        
        Args:
            subsystem_name: Name of the subsystem
            event_name: Name of the event
            details: Additional details about the event
        """
        self._log_event(subsystem_name, event_name, details)
        
        # Also log to NetworkTables events subtable
        if subsystem_name not in self.subsystem_tables:
            self.register_subsystem(subsystem_name)
        
        events_table = self.subsystem_tables[subsystem_name].getSubTable("Events")
        events_table.putString(event_name, f"{time.time() - self.start_time:.2f}s: {details}")
    
    def _log_event(self, subsystem_name: str, event_name: str, details: str = "") -> None:
        """Internal method to log events to the data log."""
        timestamp = time.time() - self.start_time
        log_message = f"[{timestamp:.2f}s] [{subsystem_name}] {event_name}"
        if details:
            log_message += f": {details}"
        
        # Log to both console and data log
        print(log_message)
        DataLogManager.log(log_message)
    
    def register_health_threshold(self, subsystem_name: str, key: str, 
                                 min_value: Optional[float] = None, 
                                 max_value: Optional[float] = None,
                                 warning_min: Optional[float] = None,
                                 warning_max: Optional[float] = None) -> None:
        """
        Register health monitoring thresholds for a value.
        
        Args:
            subsystem_name: Name of the subsystem
            key: Data key to monitor
            min_value: Minimum acceptable value (error if below)
            max_value: Maximum acceptable value (error if above)
            warning_min: Warning threshold minimum
            warning_max: Warning threshold maximum
        """
        if subsystem_name not in self.health_thresholds:
            self.health_thresholds[subsystem_name] = {}
        
        self.health_thresholds[subsystem_name][key] = {
            "min": min_value if min_value is not None else float('-inf'),
            "max": max_value if max_value is not None else float('inf'),
            "warning_min": warning_min if warning_min is not None else float('-inf'),
            "warning_max": warning_max if warning_max is not None else float('inf')
        }
        
        self._log_event("TelemetryManager", 
                      f"Registered health threshold for {subsystem_name}.{key}")
    
    def _check_health_threshold(self, subsystem_name: str, key: str, value: float) -> None:
        """Check if a value is within health thresholds and log warnings/errors if not."""
        if (subsystem_name in self.health_thresholds and 
            key in self.health_thresholds[subsystem_name]):
            
            thresholds = self.health_thresholds[subsystem_name][key]
            
            # Check error conditions
            if value < thresholds["min"]:
                self._log_event(subsystem_name, f"ERROR: {key} below minimum",
                              f"Value: {value}, Minimum: {thresholds['min']}")
            elif value > thresholds["max"]:
                self._log_event(subsystem_name, f"ERROR: {key} above maximum",
                              f"Value: {value}, Maximum: {thresholds['max']}")
            
            # Check warning conditions
            elif value < thresholds["warning_min"]:
                self._log_event(subsystem_name, f"WARNING: {key} below warning threshold",
                              f"Value: {value}, Warning Min: {thresholds['warning_min']}")
            elif value > thresholds["warning_max"]:
                self._log_event(subsystem_name, f"WARNING: {key} above warning threshold",
                              f"Value: {value}, Warning Max: {thresholds['warning_max']}")
    
    def _update_statistics(self, subsystem_name: str, key: str, value: float) -> None:
        """Update running statistics for a numerical value."""
        stats_key = f"{subsystem_name}.{key}"
        
        if stats_key not in self.stats_tracking:
            self.stats_tracking[stats_key] = {
                "count": 0,
                "sum": 0,
                "sum_squared": 0,
                "min": float('inf'),
                "max": float('-inf'),
                "last_value": value
            }
        
        stats = self.stats_tracking[stats_key]
        stats["count"] += 1
        stats["sum"] += value
        stats["sum_squared"] += value * value
        stats["min"] = min(stats["min"], value)
        stats["max"] = max(stats["max"], value)
        stats["last_value"] = value
        
        # Calculate and publish statistics periodically
        if stats["count"] % 50 == 0:  # Update stats every 50 samples
            self._publish_statistics(subsystem_name, key, stats)
    
    def _publish_statistics(self, subsystem_name: str, key: str, stats: Dict[str, Any]) -> None:
        """Publish statistics to NetworkTables."""
        if subsystem_name not in self.subsystem_tables:
            self.register_subsystem(subsystem_name)
        
        stats_table = self.subsystem_tables[subsystem_name].getSubTable("Statistics").getSubTable(key)
        
        # Calculate statistics
        mean = stats["sum"] / stats["count"] if stats["count"] > 0 else 0
        variance = (stats["sum_squared"] / stats["count"] - mean * mean) if stats["count"] > 0 else 0
        std_dev = math.sqrt(max(0, variance))
        
        # Publish to NetworkTables
        stats_table.putNumber("mean", mean)
        stats_table.putNumber("min", stats["min"])
        stats_table.putNumber("max", stats["max"])
        stats_table.putNumber("stdDev", std_dev)
        stats_table.putNumber("count", stats["count"])
        
        # Calculate rate of change
        if "last_time" in stats and "last_value" in stats:
            current_time = time.time()
            time_diff = current_time - stats["last_time"]
            if time_diff > 0:
                value_diff = stats["last_value"] - stats.get("prev_value", stats["last_value"])
                rate = value_diff / time_diff
                stats_table.putNumber("rate", rate)
            
            stats["prev_value"] = stats["last_value"]
            stats["last_time"] = current_time
        else:
            stats["last_time"] = time.time()
            stats["prev_value"] = stats["last_value"]
    
    def log_subsystem_state(self, subsystem_name: str, state_dict: Dict[str, Any]) -> None:
        """
        Log multiple data points for a subsystem at once.
        
        Args:
            subsystem_name: Name of the subsystem
            state_dict: Dictionary of key-value pairs to log
        """
        for key, value in state_dict.items():
            self.log_data(subsystem_name, key, value)
    
    def log_mechanism_state(self, mechanism_name: str, mechanism_data: Dict[str, Any]) -> None:
        """
        Log mechanism state for AdvantageScope's mechanism tab.
        
        Args:
            mechanism_name: Name of the mechanism
            mechanism_data: Dictionary with position data for joints/components
        """
        mechanism_table = self.telemetry_table.getSubTable("Mechanisms").getSubTable(mechanism_name)
        
        for component, value in mechanism_data.items():
            mechanism_table.putNumber(component, value)
    
    def log_swerve_state(self, module_states: Dict[str, Dict[str, float]]) -> None:
        """
        Log swerve module states for AdvantageScope's swerve tab.
        
        Args:
            module_states: Dictionary mapping module names to their states
                           (drive_speed, turn_angle, etc.)
        """
        swerve_table = self.telemetry_table.getSubTable("Swerve")
        
        for module_name, state in module_states.items():
            module_table = swerve_table.getSubTable(module_name)
            for key, value in state.items():
                module_table.putNumber(key, value)
    
    def log_odometry(self, pose: Pose2d) -> None:
        """
        Log robot odometry for AdvantageScope's odometry tab.
        
        Args:
            pose: Current robot pose
        """
        odometry_table = self.telemetry_table.getSubTable("Odometry")
        odometry_table.putNumber("x", pose.X())
        odometry_table.putNumber("y", pose.Y())
        odometry_table.putNumber("rotation", pose.rotation().degrees())
    
    def log_scoring_state(self, 
                          speaker_ready: bool = False,
                          speaker_distance: float = 0.0,
                          speaker_angle: float = 0.0,
                          amp_ready: bool = False,
                          amp_distance: float = 0.0,
                          stage_ready: bool = False,
                          stage_position: str = "NONE") -> None:
        """
        Log scoring-related state for Crescendo game elements.
        
        Args:
            speaker_ready: Whether the robot is ready to score in speaker
            speaker_distance: Distance to speaker in meters
            speaker_angle: Angle to speaker in degrees
            amp_ready: Whether the robot is ready to score in amp
            amp_distance: Distance to amp in meters
            stage_ready: Whether the robot is ready to climb
            stage_position: Current stage position ("NONE", "PARKED", "SPOTLIT", "TRAP")
        """
        scoring_table = self.telemetry_table.getSubTable("Scoring")
        
        # Speaker state
        scoring_table.putBoolean("speaker_ready", speaker_ready)
        scoring_table.putNumber("speaker_distance", speaker_distance)
        scoring_table.putNumber("speaker_angle", speaker_angle)
        
        # Amp state
        scoring_table.putBoolean("amp_ready", amp_ready)
        scoring_table.putNumber("amp_distance", amp_distance)
        
        # Stage state
        scoring_table.putBoolean("stage_ready", stage_ready)
        scoring_table.putString("stage_position", stage_position)
    
    def log_note_detection(self, 
                          note_detected: bool = False,
                          note_distance: float = 0.0,
                          note_angle: float = 0.0) -> None:
        """
        Log game piece (note) detection state.
        
        Args:
            note_detected: Whether a note is detected
            note_distance: Distance to the note in meters
            note_angle: Angle to the note in degrees
        """
        vision_table = self.telemetry_table.getSubTable("Vision")
        vision_table.putBoolean("note_detected", note_detected)
        vision_table.putNumber("note_distance", note_distance)
        vision_table.putNumber("note_angle", note_angle)
    
    def log_scoring_success(self, 
                           location: str,
                           success: bool,
                           distance: float = 0.0,
                           angle: float = 0.0) -> None:
        """
        Log a scoring attempt for statistics tracking.
        
        Args:
            location: Where the attempt was made ("SPEAKER", "AMP", "TRAP")
            success: Whether the scoring attempt was successful
            distance: Distance from which the attempt was made
            angle: Angle from which the attempt was made
        """
        stats_key = f"{location.lower()}_success_rate"
        if stats_key not in self.stats_tracking:
            self.stats_tracking[stats_key] = {
                "attempts": 0,
                "successes": 0,
                "total_distance": 0.0,
                "total_angle": 0.0
            }
        
        stats = self.stats_tracking[stats_key]
        stats["attempts"] += 1
        if success:
            stats["successes"] += 1
        stats["total_distance"] += distance
        stats["total_angle"] += angle
        
        # Calculate and publish statistics
        success_rate = (stats["successes"] / stats["attempts"]) * 100
        avg_distance = stats["total_distance"] / stats["attempts"]
        avg_angle = stats["total_angle"] / stats["attempts"]
        
        stats_table = self.telemetry_table.getSubTable("Statistics")
        stats_table.putNumber(f"{stats_key}", success_rate)
        stats_table.putNumber(f"{location.lower()}_avg_distance", avg_distance)
        stats_table.putNumber(f"{location.lower()}_avg_angle", avg_angle)
        
        # Log the event
        self.log_event("Scoring", 
                      f"{location} {'SUCCESS' if success else 'MISS'} at {distance:.2f}m, {angle:.1f}°")
    
    def log_auto_path(self, path_poses: List[Pose2d]) -> None:
        """
        Log autonomous path poses for visualization.
        
        Args:
            path_poses: List of poses in the path
        """
        auto_table = self.telemetry_table.getSubTable("Auto")
        path_table = auto_table.getSubTable("Path")
        
        # Convert poses to arrays for NetworkTables
        x_points = []
        y_points = []
        rotations = []
        
        for pose in path_poses:
            x_points.append(pose.X())
            y_points.append(pose.Y())
            rotations.append(pose.rotation().degrees())
        
        path_table.putNumberArray("x", x_points)
        path_table.putNumberArray("y", y_points)
        path_table.putNumberArray("rotation", rotations)
    
    def log_system_health(self) -> None:
        """Log system health metrics."""
        import psutil
        
        system_table = self.telemetry_table.getSubTable("System")
        
        # CPU and memory usage
        cpu_percent = psutil.cpu_percent()
        memory = psutil.virtual_memory()
        
        system_table.putNumber("cpu_usage", cpu_percent)
        system_table.putNumber("memory_usage", memory.percent)
        
        # Check system health thresholds
        if cpu_percent > 80:
            self.log_event("System", "WARNING: High CPU usage", f"{cpu_percent}%")
        if memory.percent > 80:
            self.log_event("System", "WARNING: High memory usage", f"{memory.percent}%")
    
    def periodic(self) -> None:
        """
        Periodic update method - call this from robotPeriodic.
        Updates timestamps, system health, and flushes data.
        """
        # Update global timestamp
        self.telemetry_table.putNumber("timestamp", time.time() - self.start_time)
        
        # Update system health
        self.log_system_health()
        
        # Flush NetworkTables updates
        self.nt_instance.flush() 