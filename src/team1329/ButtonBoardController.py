# ButtonBoardController.py
#
# This file defines a controller class to interface with a GP 2040-based
# button board for robot control functions

import wpilib
import commands2
import commands2.button
from typing import List, Dict, Callable, Optional

class ButtonBoardController:
    """
    Controller class for a GP 2040-based button board.
    
    The GP 2040 button board shows up as a standard joystick/controller to the robot,
    so we can use the WPILib joystick interface to read its state.
    
    This class provides methods for configuring button bindings and implements
    debouncing to avoid accidental triggers.
    """
    
    def __init__(self, port: int = None, name: str = "GP2040 Button Board"):
        """
        Initialize the button board controller.
        
        Args:
            port (int, optional): The USB port number for the button board. If None,
                                 will attempt to auto-detect the button board.
            name (str): A friendly name for this button board
        """
        # Auto-detect port if not specified
        if port is None:
            port = self.detect_button_board_port()
            if port >= 0:
                print(f"Auto-detected button board on port {port}")
            else:
                # Default to port 2 if detection fails
                port = 2
                print(f"Warning: Could not auto-detect button board, defaulting to port {port}")
        
        self.port = port
        self.name = name
        
        # Create the joystick interface for the button board
        self.joystick = wpilib.Joystick(port)
        
        # Create a command Xbox controller wrapper for button bindings
        self.command_joystick = commands2.button.CommandJoystick(port)
        
        # Button state tracking for custom debouncing
        self.button_states = {}
        self.button_timestamps = {}
        self.debounce_time = 0.1  # seconds
        
        # Mapping of button indices to function names for readability
        self.button_mapping = {}
        
        # Status LED if available on the button board
        self._has_leds = False
        
        # Joystick detection
        self._is_connected = False
        self._check_connection()
        
        print(f"Initialized {self.name} on port {self.port}")
        
        # Display controller info if connected
        if self._is_connected:
            self.print_controller_info()
    
    def _check_connection(self) -> bool:
        """
        Check if the button board is connected.
        
        Returns:
            bool: True if connected, False otherwise
        """
        self._is_connected = wpilib.DriverStation.isJoystickConnected(self.port)
        return self._is_connected
    
    def is_connected(self) -> bool:
        """
        Get the connection status of the button board.
        
        Returns:
            bool: True if connected, False otherwise
        """
        return self._check_connection()
    
    def setup_button_mapping(self, mapping: Dict[int, str]) -> None:
        """
        Set up a mapping of button indices to descriptive names.
        
        Args:
            mapping (Dict[int, str]): Dictionary mapping button indices to names
        """
        self.button_mapping = mapping
        print(f"Button mapping configured for {self.name}:")
        for idx, name in mapping.items():
            print(f"  Button {idx}: {name}")
    
    def get_button_name(self, button_idx: int) -> str:
        """
        Get the configured name for a button, or a default name if not configured.
        
        Args:
            button_idx (int): The button index
            
        Returns:
            str: The name of the button
        """
        return self.button_mapping.get(button_idx, f"Button {button_idx}")
    
    def get_button(self, button_idx: int) -> bool:
        """
        Get the current state of a button.
        
        Args:
            button_idx (int): The button index
            
        Returns:
            bool: True if the button is pressed, False otherwise
        """
        if not self._is_connected:
            return False
        
        return self.joystick.getRawButton(button_idx)
    
    def get_debounced_button(self, button_idx: int) -> bool:
        """
        Get the debounced state of a button to avoid rapid toggling.
        
        Args:
            button_idx (int): The button index
            
        Returns:
            bool: True if the button is pressed (after debouncing), False otherwise
        """
        if not self._is_connected:
            return False
        
        current_time = wpilib.Timer.getFPGATimestamp()
        current_state = self.get_button(button_idx)
        
        # Initialize button state if not already tracked
        if button_idx not in self.button_states:
            self.button_states[button_idx] = False
            self.button_timestamps[button_idx] = 0.0
        
        # Only update state if enough time has passed since last change
        if current_state != self.button_states[button_idx]:
            if current_time - self.button_timestamps[button_idx] > self.debounce_time:
                self.button_states[button_idx] = current_state
                self.button_timestamps[button_idx] = current_time
        
        return self.button_states[button_idx]
    
    def button(self, button_idx: int) -> commands2.button.Trigger:
        """
        Get a CommandButton object for the specified button.
        
        Args:
            button_idx (int): The button index
            
        Returns:
            commands2.button.Trigger: A trigger for the button
        """
        return self.command_joystick.button(button_idx)
    
    def configureButtonBindings(self, 
                               drivetrain=None, 
                               elevator=None, 
                               endEffector=None, 
                               vision=None) -> None:
        """
        Configure button bindings for robot functions.
        This method establishes the relationship between button presses and robot commands.
        
        Args:
            drivetrain: The drive subsystem (optional)
            elevator: The elevator subsystem (optional)
            endEffector: The end effector subsystem (optional)
            vision: The vision subsystem (optional)
        """
        if not self._is_connected:
            print(f"Warning: {self.name} not connected, skipping button binding configuration")
            return
        
        # Example button configurations - customize based on your button board layout
        # Each command is wrapped with a check for subsystem existence
        
        # Reset gyro (Button 1)
        if drivetrain:
            self.button(1).onTrue(
                commands2.InstantCommand(lambda: drivetrain.gyro.set_yaw(0))
            )
            print(f"Configured {self.get_button_name(1)} to reset gyro")
        
        # Toggle debug mode (Button 2)
        # Assuming is_debug_mode is a reference to the debug flag: [is_debug_mode]
        if hasattr(self, "is_debug_mode") and self.is_debug_mode is not None:
            self.button(2).onTrue(
                commands2.InstantCommand(lambda: self._toggle_debug_mode())
            )
            print(f"Configured {self.get_button_name(2)} to toggle debug mode")
        
        # Elevator levels
        if elevator:
            # Go to level 1 (Button 3)
            self.button(3).onTrue(
                commands2.InstantCommand(lambda: self._set_elevator_level(elevator, 1))
            )
            print(f"Configured {self.get_button_name(3)} for elevator level 1")
            
            # Go to level 2 (Button 4)
            self.button(4).onTrue(
                commands2.InstantCommand(lambda: self._set_elevator_level(elevator, 2))
            )
            print(f"Configured {self.get_button_name(4)} for elevator level 2")
            
            # Go to level 3 (Button 5)
            self.button(5).onTrue(
                commands2.InstantCommand(lambda: self._set_elevator_level(elevator, 3))
            )
            print(f"Configured {self.get_button_name(5)} for elevator level 3")
            
            # Go to level 4 (Button 6)
            self.button(6).onTrue(
                commands2.InstantCommand(lambda: self._set_elevator_level(elevator, 4))
            )
            print(f"Configured {self.get_button_name(6)} for elevator level 4")
        
        # End effector controls
        if endEffector:
            # Coral intake (Button 7)
            self.button(7).onTrue(
                commands2.InstantCommand(lambda: endEffector.startCoralMotors())
            )
            self.button(7).onFalse(
                commands2.InstantCommand(lambda: endEffector.stopCoralMotors())
            )
            print(f"Configured {self.get_button_name(7)} for coral intake")
            
            # Algae intake (Button 8)
            self.button(8).onTrue(
                commands2.InstantCommand(lambda: endEffector.algae_intake_motor.set(-0.6))
            )
            self.button(8).onFalse(
                commands2.InstantCommand(lambda: endEffector.algae_intake_motor.set(0))
            )
            print(f"Configured {self.get_button_name(8)} for algae intake")
            
            # Algae eject (Button 9)
            self.button(9).onTrue(
                commands2.InstantCommand(lambda: endEffector.algae_intake_motor.set(0.6))
            )
            self.button(9).onFalse(
                commands2.InstantCommand(lambda: endEffector.algae_intake_motor.set(0))
            )
            print(f"Configured {self.get_button_name(9)} for algae eject")
        
        # Arm position presets (if endEffector available)
        if endEffector:
            # Algae arm floor position (Button 10)
            from commands.MoveAlgaeArmToPosition import MoveAlgaeArmToPosition
            import constants
            
            self.button(10).onTrue(
                MoveAlgaeArmToPosition(endEffector, constants.intakeConsts.algaeArmFloorIntakeAngle)
            )
            print(f"Configured {self.get_button_name(10)} for algae arm floor position")
            
            # Algae arm reef position (Button 11)
            self.button(11).onTrue(
                MoveAlgaeArmToPosition(endEffector, constants.intakeConsts.algaeArmReefIntakeAngle)
            )
            print(f"Configured {self.get_button_name(11)} for algae arm reef position")
            
            # Algae arm stowed position (Button 12)
            self.button(12).onTrue(
                MoveAlgaeArmToPosition(endEffector, 3.1)  # ~180 degrees in radians
            )
            print(f"Configured {self.get_button_name(12)} for algae arm stowed position")
    
    def _set_elevator_level(self, elevator, level: int) -> None:
        """Helper method to set elevator level"""
        if not elevator:
            return
        
        import constants
        
        # Validate level
        level = max(1, min(level, 4))
        
        # Set level
        elevator.currentLevel = level
        level_index = level - 1
        
        # Calculate position from constants
        target_height = constants.reefConsts.reefLevels[level_index][1] + constants.elevatorConsts.verticalOffset
        target_position = constants.convert.in2rot(target_height) / 2
        
        # Move elevator
        elevator.gotoPosition(target_position)
        print(f"Setting elevator to level {level}")
    
    def _toggle_debug_mode(self) -> None:
        """Helper method to toggle debug mode"""
        if hasattr(self, "is_debug_mode") and self.is_debug_mode is not None:
            self.is_debug_mode[0] = not self.is_debug_mode[0]
            mode_str = "ENABLED" if self.is_debug_mode[0] else "DISABLED"
            print(f"Debug mode {mode_str}")
    
    @staticmethod
    def detect_button_board_port():
        """
        Find which port the button board is connected to by checking all joysticks.
        
        Returns:
            int: The port number (0-5) if found, or -1 if not found
        """
        for port in range(wpilib.DriverStation.kJoystickPorts):
            if wpilib.DriverStation.isJoystickConnected(port):
                # Check joystick name
                joystick_name = wpilib.DriverStation.getJoystickName(port)
                
                # Check for the specific button board name
                if "Inno GamePad" in joystick_name:
                    print(f"Found GP2040 button board on port {port}: {joystick_name}")
                    return port
                
                # Common GP2040 device identifiers (backup checks)
                gp2040_identifiers = ["GP2040", "RP2040", "Pico", "Button Board", "DIY"]
                
                for identifier in gp2040_identifiers:
                    if identifier.lower() in joystick_name.lower():
                        return port
                
                # Check if it has a button-board-like profile
                # (many buttons, few axes, few POVs)
                button_count = wpilib.DriverStation.getStickButtonCount(port)
                axis_count = wpilib.DriverStation.getStickAxisCount(port)
                pov_count = wpilib.DriverStation.getStickPOVCount(port)
                
                if button_count >= 10 and axis_count <= 2 and pov_count <= 1:
                    # Likely a button board and not a standard controller
                    return port
        
        return -1  # Not found
    
    def print_controller_info(self):
        """Print detailed information about this controller for debugging"""
        if not self._is_connected:
            print(f"{self.name} is not connected")
            return
        
        joystick_name = wpilib.DriverStation.getJoystickName(self.port)
        button_count = wpilib.DriverStation.getStickButtonCount(self.port)
        axis_count = wpilib.DriverStation.getStickAxisCount(self.port)
        pov_count = wpilib.DriverStation.getStickPOVCount(self.port)
        
        print(f"Controller Info for {self.name} on port {self.port}:")
        print(f"  Name: {joystick_name}")
        print(f"  Button count: {button_count}")
        print(f"  Axis count: {axis_count}")
        print(f"  POV count: {pov_count}")
    
    def periodic(self) -> None:
        """
        Periodic method that can be called from the robot's periodic methods.
        Can be used to check connection status or update button states.
        """
        was_connected = self._is_connected
        self._check_connection()
        
        # Report connection changes
        if was_connected and not self._is_connected:
            print(f"Warning: {self.name} disconnected from port {self.port}")
        elif not was_connected and self._is_connected:
            print(f"{self.name} connected on port {self.port}")
            self.print_controller_info()
        
        # Example of printing disconnection warnings
        if not self._is_connected:
            # Only print occasionally to avoid flooding
            if wpilib.Timer.getFPGATimestamp() % 10 < 0.1:  # Every 10 seconds
                print(f"Warning: {self.name} not connected on port {self.port}")