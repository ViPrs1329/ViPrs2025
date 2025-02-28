#!/usr/bin/env python3
#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#
import time
import wpilib

# If in simulation mode, load simulation hooks
if wpilib.RobotBase.isSimulation():
    import sim

import commands2
import ntcore
# Import the RobotContainer class from the local module
from robotContainer import RobotContainer

class MyRobot(commands2.TimedCommandRobot):
    """
    The main robot class in a command-based structure. 
    This class handles robot lifecycle events and delegates 
    functionality to the RobotContainer.
    """
    
    def robotInit(self):
        """
        This function is called upon program startup and
        should be used for any initialization code.
        """
        print("robotInit()")
        
        # Create the robot container which holds all subsystems and controllers
        self.container = RobotContainer()
        
        # Initialize autonomous command
        self.autonomousCommand = None

    def robotPeriodic(self):
        """
        This function is called periodically in all robot modes.
        Use it to run the command scheduler.
        """
        # Run the command scheduler
        commands2.CommandScheduler.getInstance().run()
        
    def autonomousInit(self):
        print("Autonomous Starting")
        self.autonomous_start_time = time.time()
        
        # Get the selected autonomous command
        self.autonomousCommand = self.container.getAutonomousCommand()
        
        # Check for NetworkTables start command
        nt_inst = ntcore.NetworkTableInstance.getDefault()
        auto_table = nt_inst.getTable("Autonomous")
        
        # Reset field if requested
        reset_field = auto_table.getStringTopic("reset_field").subscribe("").get()
        if reset_field == "reset":
            # Reset robot position in simulation
            if self.isSimulation():
                start_pose = wpimath.geometry.Pose2d(2, 2, 0)
                self.container.drivetrain.resetHarder(start_pose)

        # Log autonomous details
        print(f"Running Autonomous: {self.autonomousCommand.getName()}")
        
        # Send autonomous start event to NetworkTables
        auto_status_pub = auto_table.getStringTopic("status").publish()
        auto_status_pub.set("Running")
        
        # Schedule the command
        self.autonomousCommand.schedule()

    def autonomousPeriodic(self):
        # Check for stop command from dashboard
        nt_inst = ntcore.NetworkTableInstance.getDefault()
        auto_table = nt_inst.getTable("Autonomous")
        stop_command = auto_table.getStringTopic("stop_command").subscribe("").get()
        
        if stop_command == "stop":
            if self.autonomousCommand is not None:
                print("Stopping autonomous command from dashboard")
                self.autonomousCommand.cancel()
                self.autonomousCommand = None
                
                # Clear the stop command to prevent continuous cancellation
                stop_pub = auto_table.getStringTopic("stop_command").publish()
                stop_pub.set("")
                
                # Update status
                auto_status_pub = auto_table.getStringTopic("status").publish()
                auto_status_pub.set("Stopped")
        
        # Optional: Add progress tracking with a more controlled approach
        if hasattr(self, 'autonomous_start_time'):
            elapsed_time = time.time() - self.autonomous_start_time
            
            # Optionally stop autonomous if it runs too long
            if elapsed_time > 15.0:  # 15 seconds max
                print("Autonomous period timed out")
                if self.autonomousCommand is not None:
                    self.autonomousCommand.cancel()
                    self.autonomousCommand = None
                    
                    # Update status
                    auto_status_pub = auto_table.getStringTopic("status").publish()
                    auto_status_pub.set("Timed Out")

        # Check if the autonomous command is finished
        if self.autonomousCommand is not None and self.autonomousCommand.isFinished():
            print("Autonomous command completed")
            self.autonomousCommand.end(False)
            self.autonomousCommand = None
            
            # Update status
            auto_status_pub = auto_table.getStringTopic("status").publish()
            auto_status_pub.set("Completed")

    def teleopInit(self): 
        """This function is called once each time the robot enters teleoperated mode."""
        print("teleopInit()")
        
        # Cancel the autonomous command when teleop starts
        if self.autonomousCommand is not None:
            self.autonomousCommand.cancel()
        
        # Stop rumble (if active)
        self.stopRumble()
        
        # Reset drivetrain
        self.container.drivetrain.resetHarder()
        
        # Check motor temperatures 
        if self.container.systemTempCheck():
            self.startRumble()
            for i in range(100):
                print("!!! ---------------- MOTORS TOO HOT ------------------- !!!")

    def teleopPeriodic(self):
        """This function is called periodically during teleoperated mode."""
        # No need to add code here - command scheduler is run in robotPeriodic
        # and default commands are set up in the RobotContainer
        pass

    def disabledInit(self):
        """This function is called initially when disabled."""
        print("disabledInit()")
        self.stopRumble()

    def disabledPeriodic(self):
        """This function is called periodically while disabled."""
        pass

    def testInit(self): 
        """This function is called once each time the robot enters test mode."""
        print("testInit()")
        # Cancel all running commands
        commands2.CommandScheduler.getInstance().cancelAll()
        
    def testPeriodic(self): 
        """This function is called periodically during test mode."""
        pass

    def simulationInit(self):
        """Initialization code for simulation mode."""
        print("Simulation init...")
        # Initialize any simulation-specific components
        self.simulation_table = ntcore.NetworkTableInstance.getDefault().getTable("simulation")
        
        # If you have LaserCAN sensors in EndEffector subsystem
        if hasattr(self.container, 'endEffector'):
            # Create network table entries for sensor values
            self.coral_entry_distance = self.simulation_table.getDoubleTopic("coral_entry_distance").publish()
            self.coral_stop_distance = self.simulation_table.getDoubleTopic("coral_stop_distance").publish()
            
            # Set initial values
            self.coral_entry_distance.set(1000)  # 1000mm (nothing detected)
            self.coral_stop_distance.set(1000)   # 1000mm (nothing detected)

    # Updates for robot.py - add this to simulationPeriodic or create a new method

    def simulationPeriodic(self):
        """Periodic simulation code."""
        # Existing code...
        
        # Handle reset field command
        nt_inst = ntcore.NetworkTableInstance.getDefault()
        auto_table = nt_inst.getTable("Autonomous")
        reset_field = auto_table.getStringTopic("reset_field").subscribe("").get()
        
        if reset_field == "reset":
            print("Resetting field position")
            # Reset robot position in simulation
            if hasattr(self, 'container') and hasattr(self.container, 'drivetrain'):
                start_pose = wpimath.geometry.Pose2d(2, 2, wpimath.geometry.Rotation2d(0))
                self.container.drivetrain.resetHarder(start_pose)
                
                # Clear the reset command
                reset_pub = auto_table.getStringTopic("reset_field").publish()
                reset_pub.set("")
    
    def startRumble(self):
        """Start controller rumble."""
        if hasattr(self.container.drivingController, 'getRawRumble'):
            self.container.drivingController.setRumble(
                self.container.drivingController.RumbleType.kRightRumble, 1
            )
            self.container.drivingController.setRumble(
                self.container.drivingController.RumbleType.kLeftRumble, 1
            )

    def stopRumble(self):
        """Stop controller rumble."""
        if hasattr(self.container.drivingController, 'getRawRumble'):
            self.container.drivingController.setRumble(
                self.container.drivingController.RumbleType.kRightRumble, 0
            )
            self.container.drivingController.setRumble(
                self.container.drivingController.RumbleType.kLeftRumble, 0
            )

if __name__ == "__main__":
    wpilib.run(MyRobot)