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
        
        # Log autonomous details
        print(f"Running Autonomous: {self.autonomousCommand.getName()}")
        
        # Optional: Send autonomous start event to NetworkTables
        nt_inst = ntcore.NetworkTableInstance.getDefault()
        auto_table = nt_inst.getTable("Autonomous")
        auto_start_pub = auto_table.getStringTopic("status").publish()
        auto_start_pub.set("Running")

    def autonomousPeriodic(self):
        # Optionally add progress tracking
        if hasattr(self, 'autonomous_start_time'):
            elapsed_time = time.time() - self.autonomous_start_time
            print(f"Autonomous Progress: {elapsed_time:.2f} seconds elapsed")

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

    def simulationPeriodic(self):
        """Periodic simulation code."""
        # Update simulation values periodically
        if hasattr(self.container, 'endEffector'):
            # Get values from Network Tables that could be set by simulator GUI
            entry_distance = self.simulation_table.getDoubleTopic("coral_entry_distance").subscribe(1000).get()
            stop_distance = self.simulation_table.getDoubleTopic("coral_stop_distance").subscribe(1000).get()
            
            # Update the simulated sensors
            if hasattr(self.container.endEffector.coral_intake_LC, 'sim_device'):
                self.container.endEffector.coral_intake_LC.sim_device.set_simulated_distance(entry_distance)
            if hasattr(self.container.endEffector.coral_stop_LC, 'sim_device'):
                self.container.endEffector.coral_stop_LC.sim_device.set_simulated_distance(stop_distance)
    
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