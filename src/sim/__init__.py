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
import commands2
from commands2.button import CommandXboxController
import wpilib.simulation

# Only execute simulation setup in simulation mode
if wpilib.RobotBase.isSimulation():
    print("==== Initializing Simulation Mode ====")
    
    class KeyboardState:
        """Tracks keyboard state for simulation controls."""
        def __init__(self):
            self.keys = {}
            self.axes = {
                'left_x': 0.0,
                'left_y': 0.0,
                'right_x': 0.0,
                'right_y': 0.0,
                'left_trigger': 0.0,
                'right_trigger': 0.0
            }
            self.pressed_keys = set()
            
        def set_key(self, key, value):
            """Set a key's state and update pressed keys set."""
            self.keys[key] = value
            if value:
                self.pressed_keys.add(key)
            else:
                self.pressed_keys.discard(key)
            
        def get_key(self, key):
            """Get a key's state."""
            return self.keys.get(key, False)
            
        def set_axis(self, axis, value):
            """Set an axis value."""
            if axis in self.axes:
                self.axes[axis] = value
                
        def get_axis(self, axis):
            """Get an axis value."""
            return self.axes.get(axis, 0.0)
        
        def is_key_pressed(self, key):
            """Check if a key is currently pressed."""
            return key in self.pressed_keys

    # Create a global keyboard state instance
    keyboard_state = KeyboardState()
    
    # Create a mock controller class for simulation
    class MockXboxController(CommandXboxController):
        """Mock Xbox controller for simulation with keyboard input support."""
        
        # Key bindings for Xbox controller buttons
        KEY_BINDINGS = {
            'a': 'A',           # A button
            's': 'B',           # B button
            'd': 'X',           # X button
            'f': 'Y',           # Y button
            'q': 'LB',          # Left bumper
            'e': 'RB',          # Right bumper
            'tab': 'Back',      # Back button
            'enter': 'Start',   # Start button
            'space': 'LSB',     # Left stick button
            'rshift': 'RSB'     # Right stick button
        }
        
        # Axis mappings
        AXIS_MAPPINGS = {
            'w': ('left_y', 1.0),    # Forward
            'x': ('left_y', -1.0),   # Backward
            'a': ('left_x', -1.0),   # Left
            'd': ('left_x', 1.0),    # Right
            'i': ('right_y', 1.0),   # Look up
            'k': ('right_y', -1.0),  # Look down
            'j': ('right_x', -1.0),  # Look left
            'l': ('right_x', 1.0),   # Look right
            'r': ('right_trigger', 1.0),  # Right trigger
            't': ('left_trigger', 1.0)    # Left trigger
        }
        
        def __init__(self, port):
            """Initialize the mock controller."""
            super().__init__(port)
            print(f"Created MockXboxController on port {port} with keyboard mapping")
            print("Keyboard Controls:")
            print("  Movement: WASD")
            print("  Camera: IJKL")
            print("  Buttons: A(a) B(s) X(d) Y(f)")
            print("  Bumpers: Q(LB) E(RB)")
            print("  Triggers: T(LT) R(RT)")
            print("  Special: Tab(Back) Enter(Start) Space(LSB) RShift(RSB)")
            
        def getRawButton(self, button):
            """Map keyboard input to controller buttons."""
            return keyboard_state.get_key(button)
            
        def getRawAxis(self, axis):
            """Map keyboard input to controller axes."""
            # Map axis numbers to our named axes
            axis_map = {
                0: 'left_x',
                1: 'left_y',
                2: 'left_trigger',
                3: 'right_trigger',
                4: 'right_x',
                5: 'right_y'
            }
            
            # Get the axis name
            axis_name = axis_map.get(axis, 'unknown')
            
            # Get the current axis value
            value = keyboard_state.get_axis(axis_name)
            
            # Print debug info
            print(f"getRawAxis({axis}) -> {axis_name} = {value}")
            
            return value
            
        def getBackButton(self):
            return keyboard_state.get_key('Back')
            
        def getAButton(self):
            return keyboard_state.get_key('A')
            
        def getBButton(self):
            return keyboard_state.get_key('B')
            
        def getXButton(self):
            return keyboard_state.get_key('X')
            
        def getYButton(self):
            return keyboard_state.get_key('Y')
            
        def getLeftBumper(self):
            return keyboard_state.get_key('LB')
            
        def getRightBumper(self):
            return keyboard_state.get_key('RB')
            
        def getLeftStickButton(self):
            return keyboard_state.get_key('LSB')
            
        def getRightStickButton(self):
            return keyboard_state.get_key('RSB')
            
        def getLeftTriggerAxis(self):
            return keyboard_state.get_axis('left_trigger')
            
        def getRightTriggerAxis(self):
            return keyboard_state.get_axis('right_trigger')
            
        def getLeftX(self):
            return keyboard_state.get_axis('left_x')
            
        def getLeftY(self):
            return keyboard_state.get_axis('left_y')
            
        def getRightX(self):
            return keyboard_state.get_axis('right_x')
            
        def getRightY(self):
            return keyboard_state.get_axis('right_y')
            
        def getStartButton(self):
            """Get the Start button state (mapped to Enter key)."""
            return keyboard_state.get_key('Start')
            
        def setRumble(self, type, value):
            pass  # Ignore rumble in simulation

    # Create a keyboard input handler for the simulation GUI
    def handle_keyboard_input(window, key, scancode, action, mods):
        """Handle keyboard input in the simulation GUI."""
        key = str(key)
        print(f"Keyboard input: key={key}, action={action}, scancode={scancode}, mods={mods}")
        
        # Handle button presses
        if action == wpilib.simulation.Action.kPress:
            if key in MockXboxController.KEY_BINDINGS:
                button = MockXboxController.KEY_BINDINGS[key]
                keyboard_state.set_key(button, True)
                print(f"Button pressed: {button}")
                
            # Handle axis inputs
            if key in MockXboxController.AXIS_MAPPINGS:
                axis, value = MockXboxController.AXIS_MAPPINGS[key]
                keyboard_state.set_axis(axis, value)
                print(f"Axis set: {axis} = {value}")
                
        # Handle button releases
        elif action == wpilib.simulation.Action.kRelease:
            if key in MockXboxController.KEY_BINDINGS:
                button = MockXboxController.KEY_BINDINGS[key]
                keyboard_state.set_key(button, False)
                print(f"Button released: {button}")
                
            # Handle axis releases
            if key in MockXboxController.AXIS_MAPPINGS:
                axis, _ = MockXboxController.AXIS_MAPPINGS[key]
                # Check if the opposite key is pressed
                if axis == "left_y":
                    if keyboard_state.is_key_pressed("w"):
                        keyboard_state.set_axis(axis, -1.0)
                    elif keyboard_state.is_key_pressed("x"):
                        keyboard_state.set_axis(axis, 1.0)
                    else:
                        keyboard_state.set_axis(axis, 0.0)
                elif axis == "left_x":
                    if keyboard_state.is_key_pressed("a"):
                        keyboard_state.set_axis(axis, -1.0)
                    elif keyboard_state.is_key_pressed("d"):
                        keyboard_state.set_axis(axis, 1.0)
                    else:
                        keyboard_state.set_axis(axis, 0.0)
                elif axis == "right_y":
                    if keyboard_state.is_key_pressed("i"):
                        keyboard_state.set_axis(axis, -1.0)
                    elif keyboard_state.is_key_pressed("k"):
                        keyboard_state.set_axis(axis, 1.0)
                    else:
                        keyboard_state.set_axis(axis, 0.0)
                elif axis == "right_x":
                    if keyboard_state.is_key_pressed("j"):
                        keyboard_state.set_axis(axis, -1.0)
                    elif keyboard_state.is_key_pressed("l"):
                        keyboard_state.set_axis(axis, 1.0)
                    else:
                        keyboard_state.set_axis(axis, 0.0)
                print(f"Axis released: {axis}")
        
        print(f"Current keyboard state: pressed_keys={keyboard_state.pressed_keys}, axes={keyboard_state.axes}")

    # Create a mock controller instance for simulation
    try:
        driver_controller = MockXboxController(0)
        operator_controller = MockXboxController(1)
        print("Successfully created mock controllers")
    except Exception as e:
        print(f"Warning: Could not create mock controllers: {e}")

    # Register keyboard input handler with the simulation window
    try:
        if hasattr(wpilib.simulation, "keyboard_callback"):
            wpilib.simulation.keyboard_callback = handle_keyboard_input
            print("Successfully registered keyboard input handler")
        else:
            print("Warning: keyboard_callback not available in wpilib.simulation")
    except Exception as e:
        print(f"Warning: Could not register keyboard input handler: {e}")

    print("==== Simulation Mode Initialized ====")