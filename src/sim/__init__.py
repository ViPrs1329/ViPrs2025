import wpilib
import sys
import builtins
import importlib.util

# Store the original import
original_import = builtins.__import__

def simulation_import_hook(name, globals=None, locals=None, fromlist=(), level=0):
    """Import hook to override hardware modules with simulation versions."""
    
    # Check if we're running in simulation mode
    if wpilib.RobotBase.isSimulation():
        # Override hardware libraries with simulation versions
        if name == 'rev':
            # First, try to import our simulation version
            try:
                # Import the specific simulation module directly
                from sim.revlib.sim_sparkmax import SimSparkMax, SimSparkFlex, SparkBaseConfig
                
                # Create a fake module to return
                rev_module = type('rev', (), {})()
                
                # Add our simulation classes
                # Add SparkRelativeEncoder to the rev_module
                rev_module.SparkRelativeEncoder = type('SparkRelativeEncoder', (), {
                    'Type': type('Type', (), {
                        'kHallSensor': 0,
                        'kQuadrature': 1
                    })
                })
                rev_module.SparkMax = SimSparkMax
                rev_module.SparkFlex = SimSparkFlex
                rev_module.SparkBaseConfig = SparkBaseConfig
                rev_module.SparkBase = type('SparkBase', (), {
                    'ResetMode': type('ResetMode', (), {
                        'kResetSafeParameters': 0
                    }),
                    'PersistMode': type('PersistMode', (), {
                        'kPersistParameters': 0
                    })
                })()
                
                # Add the motor types and other necessary attributes
                rev_module.CANSparkMax = SimSparkMax
                
                return rev_module
            except ImportError as e:
                print(f"Error importing simulation modules: {e}")
                # If our simulation version fails, try to import from rev.py
                try:
                    import rev
                    return rev
                except ImportError:
                    # If that also fails, try the original
                    pass
        
        # ... rest of the existing import hook code for phoenix6, etc.
    
    # For any other imports, use the original import function
    return original_import(name, globals, locals, fromlist, level)

# Replace the built-in import with our custom import hook
builtins.__import__ = simulation_import_hook

# Make sure this module is loaded early
print("Simulation import hooks installed")