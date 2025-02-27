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
            sim_path = "sim.revlib.sim_sparkmax"
            try:
                sim_module = original_import(sim_path, globals, locals, fromlist, level)
                
                # Create a fake module to return
                rev_module = type('rev', (), {})()
                
                # Add our simulation classes
                rev_module.SparkMax = sim_module.SimSparkMax
                rev_module.SparkFlex = sim_module.SimSparkFlex
                rev_module.SparkBaseConfig = sim_module.SparkBaseConfig
                rev_module.SparkBase = type('SparkBase', (), {
                    'ResetMode': type('ResetMode', (), {
                        'kResetSafeParameters': 0
                    }),
                    'PersistMode': type('PersistMode', (), {
                        'kPersistParameters': 0
                    })
                })()
                
                # Add the motor types and other necessary attributes
                rev_module.CANSparkMax = sim_module.SimSparkMax
                
                return rev_module
            except ImportError:
                # If our simulation version fails, try the original
                pass
        
        elif name == 'phoenix6.hardware':
            # Import our simulation version
            sim_path = "sim.phoenix6.sim_phoenix6"
            try:
                sim_module = original_import(sim_path, globals, locals, fromlist, level)
                
                # Create a fake module to return
                phoenix_module = type('phoenix6.hardware', (), {})()
                
                # Add our simulation classes
                phoenix_module.Pigeon2 = sim_module.Pigeon2
                phoenix_module.CANcoder = sim_module.CANcoder
                
                return phoenix_module
            except ImportError:
                # If our simulation version fails, try the original
                pass
    
    # For any other imports, use the original import function
    return original_import(name, globals, locals, fromlist, level)

# Replace the built-in import with our custom import hook
builtins.__import__ = simulation_import_hook

# Make sure this module is loaded early
print("Simulation import hooks installed")