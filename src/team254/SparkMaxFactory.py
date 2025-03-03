# src/team254/SparkMaxFactory.py
from team254.LazySparkMax import LazySparkMax, LazySparkFlex
import rev
import wpilib

class SparkMaxFactory:
    """
    Factory for creating pre-configured LazySparkMax controllers.
    """
    
    class Configuration:
        """Configuration for SparkMax or SparkFlex controllers."""
        def __init__(self):
            self.inverted = False
            # Handle different idle mode enums for simulation vs hardware
            if wpilib.RobotBase.isSimulation():
                # In simulation, use SparkMax.IdleMode
                self.idle_mode = rev.SparkMax.IdleMode.kCoast
            else:
                # In hardware, use CANSparkMax.IdleMode
                self.idle_mode = rev.SparkMax.IdleMode.kCoast
            self.voltage_comp_enabled = False
            self.voltage_comp_saturation = 12.0
            self.current_limit = 80
            self.follow_leader = None
            self.follow_invert = False
            
            # PID configuration (if needed)
            self.kP = 0.0
            self.kI = 0.0
            self.kD = 0.0
            self.kF = 0.0
            self.position_conversion_factor = 1.0
            self.velocity_conversion_factor = 1.0

    @staticmethod
    def createSparkMax(device_id: int, config=None):
        """
        Create and configure a LazySparkMax.
        
        Args:
            device_id: CAN ID of the SparkMax
            config: Configuration object or None to use defaults
            
        Returns:
            A configured LazySparkMax instance
        """
        spark = LazySparkMax(device_id)
        
        # Apply factory defaults
        try:
            spark.restoreFactoryDefaults()
        except Exception as e:
            print(f"Warning: Could not restore factory defaults: {e}")
        
        if config:
            try:
                # Apply configuration
                spark.setInverted(config.inverted)
                spark.setIdleMode(config.idle_mode)
                
                if config.voltage_comp_enabled:
                    spark.enableVoltageCompensation(config.voltage_comp_saturation)
                else:
                    spark.disableVoltageCompensation()
                    
                spark.setSmartCurrentLimit(config.current_limit)
                
                if config.follow_leader is not None:
                    spark.follow(config.follow_leader, config.follow_invert)
                    
                # Configure PID if needed
                try:
                    encoder = spark.getEncoder()
                    encoder.setPositionConversionFactor(config.position_conversion_factor)
                    encoder.setVelocityConversionFactor(config.velocity_conversion_factor)
                    
                    pid_controller = spark.getPIDController()
                    pid_controller.setP(config.kP)
                    pid_controller.setI(config.kI)
                    pid_controller.setD(config.kD)
                    pid_controller.setFF(config.kF)
                except Exception as e:
                    print(f"Warning: Could not configure PID: {e}")
            except Exception as e:
                print(f"Warning: Error configuring SparkMax {device_id}: {e}")
        
        # Burn flash to ensure settings persist
        try:
            spark.burnFlash()
        except Exception as e:
            print(f"Warning: Could not burn flash: {e}")
        
        return spark
    
    @staticmethod
    def createSparkFlex(device_id: int, config=None):
        """
        Create and configure a LazySparkFlex.
        
        Args:
            device_id: CAN ID of the SparkFlex
            config: Configuration object or None to use defaults
            
        Returns:
            A configured LazySparkFlex instance
        """
        spark = LazySparkFlex(device_id)
        
        # Apply factory defaults
        try:
            # TODO: restoreFactoryDefaults() doesn't exist in the new rev module
            # Check here: https://robotpy.readthedocs.io/projects/rev/en/stable/rev/
            spark.restoreFactoryDefaults()
        except Exception as e:
            print(f"Warning: Could not restore factory defaults: {e}")
        
        if config:
            try:
                # Apply configuration
                spark.setInverted(config.inverted)
                spark.setIdleMode(config.idle_mode)
                
                if config.voltage_comp_enabled:
                    spark.enableVoltageCompensation(config.voltage_comp_saturation)
                else:
                    spark.disableVoltageCompensation()
                    
                spark.setSmartCurrentLimit(config.current_limit)
                
                if config.follow_leader is not None:
                    spark.follow(config.follow_leader, config.follow_invert)
                    
                # Configure PID if needed
                try:
                    encoder = spark.getEncoder()
                    encoder.setPositionConversionFactor(config.position_conversion_factor)
                    encoder.setVelocityConversionFactor(config.velocity_conversion_factor)
                    
                    pid_controller = spark.getPIDController()
                    pid_controller.setP(config.kP)
                    pid_controller.setI(config.kI)
                    pid_controller.setD(config.kD)
                    pid_controller.setFF(config.kF)
                except Exception as e:
                    print(f"Warning: Could not configure PID: {e}")
            except Exception as e:
                print(f"Warning: Error configuring SparkFlex {device_id}: {e}")
        
        # Burn flash to ensure settings persist
        try:
            spark.burnFlash()
        except Exception as e:
            print(f"Warning: Could not burn flash: {e}")
        
        return spark