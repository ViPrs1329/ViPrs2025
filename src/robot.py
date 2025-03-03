#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import time
import wpilib
import commands2
import ntcore

# If in simulation mode, initialize simulation hooks
if wpilib.RobotBase.isSimulation():
    import sim

# Import the robot container class
from robotContainer import RobotContainer


class Robot(commands2.TimedCommandRobot):
    """
    Main robot class that handles core robot functionality.
    
    This class handles robot lifecycle events (init, periodic, etc.) and
    delegates specific functionality to the RobotContainer.
    """
    
    def robotInit(self):
        """
        Initialize the robot.
        Called once when the robot starts up.
        """
        print("==== Robot Initializing ====")
        
        # Create the robot container
        self.container = RobotContainer()
        
        # Initialize autonomous command - will be set in autonomousInit
        self.autonomous_command = None
        
        # Initialize robot status for dashboard
        wpilib.SmartDashboard.putString("Robot/Status", "Initialized")
        wpilib.SmartDashboard.putString("Robot/Mode", "Disabled")
        
        print("==== Robot Initialized ====")
    
    def robotPeriodic(self):
        """
        Periodic code for all robot modes.
        Called every scheduler run.
        """
        # Run the command scheduler
        commands2.CommandScheduler.getInstance().run()
    
    def disabledInit(self):
        """Initialize disabled mode."""
        print("==== Disabled Mode ====")
        wpilib.SmartDashboard.putString("Robot/Mode", "Disabled")
        
        # Stop any rumble on controllers
        self.stopRumble()
    
    def disabledPeriodic(self):
        """Periodic code for disabled mode."""
        pass
    
    def autonomousInit(self):
        """Initialize autonomous mode."""
        print("==== Autonomous Mode ====")
        wpilib.SmartDashboard.putString("Robot/Mode", "Autonomous")
        
        # Record start time for timeout handling
        self.autonomous_start_time = time.time()
        
        # Get the autonomous command from the container
        self.autonomous_command = self.container.getAutonomousCommand()
        
        if self.autonomous_command is not None:
            # Log the command that will run
            print(f"Starting autonomous command: {self.autonomous_command.getName()}")
            
            # Send command name to dashboard
            wpilib.SmartDashboard.putString("Autonomous/Command", self.autonomous_command.getName())
            
            # Schedule the command
            self.autonomous_command.schedule()
    
    def autonomousPeriodic(self):
        """Periodic code for autonomous mode."""
        # Check for autonomous timeout (15 seconds max)
        if hasattr(self, 'autonomous_start_time'):
            elapsed_time = time.time() - self.autonomous_start_time
            
            # Update dashboard with progress
            wpilib.SmartDashboard.putNumber("Autonomous/ElapsedTime", elapsed_time)
            
            # Optional timeout handling
            if elapsed_time > 15.0 and self.autonomous_command is not None:
                print("Autonomous period timed out (15 seconds)")
                self.autonomous_command.cancel()
                self.autonomous_command = None
    
    def teleopInit(self):
        """Initialize teleop mode."""
        print("==== Teleop Mode ====")
        wpilib.SmartDashboard.putString("Robot/Mode", "Teleop")
        
        # Cancel the autonomous command when teleop starts
        if self.autonomous_command is not None:
            self.autonomous_command.cancel()
            self.autonomous_command = None
        
        # Reset systems for teleop
        if hasattr(self.container, 'drivetrain'):
            self.container.drivetrain.resetGyro()
        
        # Check system temperatures
        if self.container.systemTempCheck():
            self.startRumble()
            print("WARNING: System temperature check failed!")
    
    def teleopPeriodic(self):
        """Periodic code for teleop mode."""
        # Command scheduler runs in robotPeriodic
        pass
    
    def testInit(self):
        """Initialize test mode."""
        print("==== Test Mode ====")
        wpilib.SmartDashboard.putString("Robot/Mode", "Test")
        
        # Cancel all running commands
        commands2.CommandScheduler.getInstance().cancelAll()
    
    def testPeriodic(self):
        """Periodic code for test mode."""
        pass
    
    def simulationInit(self):
        """Initialize simulation mode."""
        print("==== Simulation Initialized ====")
        wpilib.SmartDashboard.putString("Robot/Mode", "Simulation")
        
        # Initialize simulation table
        self.sim_table = ntcore.NetworkTableInstance.getDefault().getTable("Simulation")
    
    def simulationPeriodic(self):
        """Periodic code for simulation mode."""
        # Update simulation data
        pass
    
    def startRumble(self):
        """Start rumble on controllers for feedback."""
        if hasattr(self.container, 'driver_controller'):
            self.container.driver_controller.setRumble(
                wpilib.GenericHID.RumbleType.kLeftRumble, 0.5
            )
            self.container.driver_controller.setRumble(
                wpilib.GenericHID.RumbleType.kRightRumble, 0.5
            )
            
        if hasattr(self.container, 'operator_controller'):
            self.container.operator_controller.setRumble(
                wpilib.GenericHID.RumbleType.kLeftRumble, 0.5
            )
            self.container.operator_controller.setRumble(
                wpilib.GenericHID.RumbleType.kRightRumble, 0.5
            )
    
    def stopRumble(self):
        """Stop rumble on all controllers."""
        if hasattr(self.container, 'driver_controller'):
            self.container.driver_controller.setRumble(
                wpilib.GenericHID.RumbleType.kLeftRumble, 0
            )
            self.container.driver_controller.setRumble(
                wpilib.GenericHID.RumbleType.kRightRumble, 0
            )
            
        if hasattr(self.container, 'operator_controller'):
            self.container.operator_controller.setRumble(
                wpilib.GenericHID.RumbleType.kLeftRumble, 0
            )
            self.container.operator_controller.setRumble(
                wpilib.GenericHID.RumbleType.kRightRumble, 0
            )


if __name__ == "__main__":
    wpilib.run(Robot)