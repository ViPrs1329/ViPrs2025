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
    
    def __init__(self, port=None):
        """
        Initialize the button board debugger.
        
        Args:
            port (int, optional): The joystick port to monitor. If None, will
                                try to auto-detect the 'Inno GamePad' controller.
        """
        # Auto-detect the button board if port not specified
        if port is None:
            port = self.find_button_board()
            
        self.port = port
        self.joystick = wpilib.Joystick(port)
        
        # Track button states to detect changes
        self.previous_button_states = {}
        
        # Report initialization
        joystick_name = wpilib.DriverStation.getJoystickName(port)
        print(f"Button board debugger initialized on port {port}")
        print(f"Controller name: {joystick_name}")
        print("Press buttons to see their index numbers")
        
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
        Check for button presses and print them.
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