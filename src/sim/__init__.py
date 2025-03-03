# sim/__init__.py
"""
Simulation support module for the robot.

This module provides simulation replacements for hardware components 
and other simulation utilities.
"""

import wpilib
import builtins
import importlib.util
import sys
import ntcore

# Only execute simulation setup in simulation mode
if wpilib.RobotBase.isSimulation():
    print("==== Initializing Simulation Mode ====")
    
    # Store the original import function
    original_import = builtins.__import__
    
    def simulation_import_hook(name, globals=None, locals=None, fromlist=(), level=0):
        """
        Import hook that replaces hardware modules with simulation versions.
        
        This allows the robot code to use the same imports in both real and simulation mode.
        """
        
        # Handle specific hardware libraries
        if name == 'rev':
            try:
                # Try to import our simulation version
                from sim.rev import (
                    SparkMax, SparkFlex, CANSparkMax, SparkMaxAbsoluteEncoder,
                    SparkRelativeEncoder, SparkPIDController
                )
                
                # Create a fake module to return
                rev_module = type('rev', (), {})()
                
                # Add simulation classes to the module
                rev_module.SparkMax = SparkMax
                rev_module.SparkFlex = SparkFlex
                rev_module.CANSparkMax = CANSparkMax
                rev_module.SparkMaxAbsoluteEncoder = SparkMaxAbsoluteEncoder
                rev_module.SparkRelativeEncoder = SparkRelativeEncoder
                rev_module.SparkPIDController = SparkPIDController
                
                # Add enum types and other necessary components
                rev_module.IdleMode = type('IdleMode', (), {
                    'kCoast': 0,
                    'kBrake': 1
                })
                
                rev_module.MotorType = type('MotorType', (), {
                    'kBrushless': 0,
                    'kBrushed': 1
                })
                
                rev_module.ControlType = type('ControlType', (), {
                    'kDutyCycle': 0,
                    'kVelocity': 1,
                    'kPosition': 2,
                    'kVoltage': 3,
                    'kCurrent': 4,
                    'kSmartMotion': 5,
                    'kSmartVelocity': 6,
                    'kSmartVoltage': 7
                })
                
                print("Using simulated REV library")
                return rev_module
                
            except ImportError as e:
                print(f"Error importing simulation REV library: {e}")
                # If our simulation version fails, try to import the real library
                try:
                    import rev
                    return rev
                except ImportError:
                    # If that also fails, use original import
                    pass
        
        elif name == 'phoenix6' or name.startswith('phoenix6.'):
            # Handle Phoenix 6 imports
            if name == 'phoenix6':
                # Import the main phoenix6 module
                try:
                    from sim.phoenix6 import hardware
                    
                    # Create phoenix6 module with hardware submodule
                    phoenix6_module = type('phoenix6', (), {})()
                    phoenix6_module.hardware = hardware
                    
                    print("Using simulated Phoenix 6 library")
                    return phoenix6_module
                    
                except ImportError as e:
                    print(f"Error importing simulation Phoenix 6 library: {e}")
                    # Fall back to original import
                    pass
                    
            elif name == 'phoenix6.hardware':
                # Import specific hardware submodule
                try:
                    from sim.phoenix6 import hardware
                    print("Using simulated Phoenix 6 hardware library")
                    return hardware
                    
                except ImportError as e:
                    print(f"Error importing simulation Phoenix 6 hardware library: {e}")
                    # Fall back to original import
                    pass
        
        # For any other imports, use the original import function
        return original_import(name, globals, locals, fromlist, level)
    
    # Replace the built-in import with our custom import hook
    builtins.__import__ = simulation_import_hook
    
    # Initialize NetworkTables for simulation
    print("Initializing NetworkTables for simulation")
    instance = ntcore.NetworkTableInstance.getDefault()
    instance.startServer()
    
    # Create a simulation dashboard table
    sim_table = instance.getTable("Simulation")
    sim_table.getStringTopic("status").publish().set("Running")
    
    print("==== Simulation Initialized ====")