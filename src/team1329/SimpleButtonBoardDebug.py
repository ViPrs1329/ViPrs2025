# SimpleButtonBoardDebug.py
#
# A simple utility class for identifying button numbers on a GP2040 button board

import wpilib
import time

class SimpleButtonBoardDebug:
    """
    A minimal utility for identifying which buttons are which on a controller.
    Simply prints the button number when a button is pressed or released.
    """
    
    def __init__(self, port=None, show_continuous_axis=False):
        """
        Initialize the button board debugger.
        
        Args:
            port (int, optional): The joystick port to monitor. If None, will
                                try to auto-detect the 'Inno GamePad' controller.
            show_continuous_axis (bool): If True, will continuously print axis values.
                                        If False, will only print significant changes.
        """
        # Auto-detect the button board if port not specified
        if port is None:
            port = self.find_button_board()
            
        self.port = port
        self.joystick = wpilib.Joystick(port)
        self.show_continuous_axis = show_continuous_axis
        
        # Track button states to detect changes
        self.previous_button_states = {}
        
        # Track axis values to detect changes
        self.previous_axis_values = {}
        
        # Track POV values
        self.previous_pov_values = {}
        
        # Timing for throttling continuous output
        self.last_axis_print_time = 0
        
        # Report initialization
        joystick_name = wpilib.DriverStation.getJoystickName(port)
        button_count = wpilib.DriverStation.getStickButtonCount(port)
        axis_count = wpilib.DriverStation.getStickAxisCount(port)
        pov_count = wpilib.DriverStation.getStickPOVCount(port)
        
        print(f"Button board debugger initialized on port {port}")
        print(f"Controller name: {joystick_name}")
        print(f"Buttons: {button_count}, Axes: {axis_count}, POVs: {pov_count}")
        print("Press buttons to see their index numbers")
        print("Move joysticks/axes to see their values")
        
    def find_button_board(self):
        """Find the port with the Inno GamePad button board"""
        for port in range(wpilib.DriverStation.kJoystickPorts):
            if wpilib.DriverStation.isJoystickConnected(port):
                name = wpilib.DriverStation.getJoystickName(port)
                if "Inno GamePad" in name:
                    print(f"Found Button Board on port {port}: {name}")
                    return port
        
        # Default to port 2 if not found
        print("Button Board not found. Defaulting to port 2")
        return 2
    
    def update(self):
        """
        Check for button presses and axis movements and print them.
        Call this method periodically from your robot code.
        """
        # Get the number of buttons on the controller
        button_count = wpilib.DriverStation.getStickButtonCount(self.port)
        
        # Check each button
        for button_index in range(1, button_count + 1):  # Buttons are 1-indexed
            # Get current button state
            current_state = self.joystick.getRawButton(button_index)
            
            # Initialize previous state if this is the first check
            if button_index not in self.previous_button_states:
                self.previous_button_states[button_index] = False
            
            # Get previous state
            previous_state = self.previous_button_states[button_index]
            
            # Detect state changes
            if current_state != previous_state:
                if current_state:
                    print(f"Button {button_index} PRESSED")
                else:
                    print(f"Button {button_index} released")
                
                # Update state
                self.previous_button_states[button_index] = current_state
        
        # Check axis values
        axis_count = wpilib.DriverStation.getStickAxisCount(self.port)
        current_time = time.time()
        
        for axis_index in range(axis_count):
            # Get current axis value
            axis_value = self.joystick.getRawAxis(axis_index)
            
            # Initialize previous value if this is the first check
            if axis_index not in self.previous_axis_values:
                self.previous_axis_values[axis_index] = 0.0
            
            # Get previous value
            previous_value = self.previous_axis_values[axis_index]
            
            # Detect significant changes in axis values
            if self.show_continuous_axis:
                # Show continuous updates but throttle to avoid flooding
                if abs(axis_value) > 0.1 and (current_time - self.last_axis_print_time) > 0.5:
                    print(f"Axis {axis_index} value: {axis_value:.2f}")
                    self.last_axis_print_time = current_time
            else:
                # Only show significant changes from previous value
                if abs(axis_value) > 0.1 and abs(axis_value - previous_value) > 0.1:
                    print(f"Axis {axis_index} value: {axis_value:.2f}")
                    self.previous_axis_values[axis_index] = axis_value
        
        # Check POV (D-pad) values
        pov_count = wpilib.DriverStation.getStickPOVCount(self.port)
        
        for pov_index in range(pov_count):
            pov_value = self.joystick.getPOV(pov_index)
            
            # Initialize previous value if this is the first check
            if pov_index not in self.previous_pov_values:
                self.previous_pov_values[pov_index] = -1
            
            # Get previous value
            previous_value = self.previous_pov_values[pov_index]
            
            # Only report changes in POV value
            if pov_value != previous_value:
                if pov_value == -1:
                    print(f"POV {pov_index} released")
                else:
                    # Translate degree values to directional names for common angles
                    direction = ""
                    if pov_value == 0:
                        direction = " (Up)"
                    elif pov_value == 90:
                        direction = " (Right)"
                    elif pov_value == 180:
                        direction = " (Down)"
                    elif pov_value == 270:
                        direction = " (Left)"
                    elif 0 < pov_value < 90:
                        direction = " (Up-Right)"
                    elif 90 < pov_value < 180:
                        direction = " (Down-Right)"
                    elif 180 < pov_value < 270:
                        direction = " (Down-Left)"
                    elif 270 < pov_value < 360:
                        direction = " (Up-Left)"
                        
                    print(f"POV {pov_index} position: {pov_value}°{direction}")
                
                # Update state
                self.previous_pov_values[pov_index] = pov_value

# Simple example of usage
if __name__ == "__main__":
    # This section only runs if you execute this file directly
    print("Starting button board debug utility")
    
    # Create the debugger
    debugger = SimpleButtonBoardDebug()
    
    # Main loop
    try:
        while True:
            # Update button states
            debugger.update()
            
            # Small delay to prevent CPU hogging
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Button board debug utility stopped")