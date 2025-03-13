# SimpleVisionSubsystem.py
#
# This file handles camera setup for driver vision:
# - Two Limelight cameras used as driver cameras
# - One USB webcam

import commands2
import wpilib
from cscore import CameraServer, UsbCamera
import ntcore

class SimpleVisionSubsystem(commands2.Subsystem):
    """
    Manages driver vision cameras:
    - 2 Limelight cameras in driver mode
    - 1 USB webcam for additional perspective
    """
    
    def __init__(self) -> None:
        super().__init__()
        
        # Network Tables instance for communication
        self.nt_instance = ntcore.NetworkTableInstance.getDefault()
        
        # Setup USB webcam on CameraServer
        self.driver_camera = self.setupDriverCamera()
        
        # Setup Limelights in driver mode
        # Assuming Limelights have hostnames "limelight-front" and "limelight-rear"
        self.limelight_front = self.setupLimelight("limelight-left")
        self.limelight_rear = self.setupLimelight("limelight-right")
        
        print("Vision subsystem initialized in driver mode")
        
    def setupDriverCamera(self):
        """Set up the USB driver camera"""
        try:
            camera = CameraServer.startAutomaticCapture()
            # Set resolution (lower for better network performance)
            camera.setResolution(320, 240)
            # Lower FPS to save bandwidth
            camera.setFPS(15)
            print("Driver USB camera initialized successfully")
            return camera
        except Exception as e:
            print(f"Error initializing driver USB camera: {e}")
            return None
    
    def setupLimelight(self, hostname):
        """
        Set up a Limelight camera in driver mode
        
        Args:
            hostname (str): Network hostname of the Limelight
        
        Returns:
            NetworkTable: The NetworkTable for the specified Limelight
        """
        # Get NetworkTable for Limelight
        limelight_table = self.nt_instance.getTable(hostname)
        
        # Configure for driver mode
        self.setDriverMode(limelight_table, True)
        
        print(f"Initialized Limelight in driver mode: {hostname}")
        return limelight_table
    
    def setDriverMode(self, limelight_table, enabled=True):
        """
        Set a Limelight to driver mode
        
        Args:
            limelight_table: The NetworkTable for the Limelight
            enabled (bool): True for driver mode, False for vision processing mode
        """
        if enabled:
            # Driver mode: LEDs off, driver camera
            limelight_table.putNumber("ledMode", 1)  # Force LEDs off
            limelight_table.putNumber("camMode", 1)  # Driver camera (no vision processing)
        else:
            # Vision mode: Use pipeline setting for LEDs, vision processing
            limelight_table.putNumber("ledMode", 0)  # Use pipeline setting
            limelight_table.putNumber("camMode", 0)  # Vision processing
    
    def setPipeline(self, limelight_table, pipeline):
        """
        Set the current pipeline on a Limelight
        
        Args:
            limelight_table: The NetworkTable for the Limelight
            pipeline (int): Pipeline number (0-9)
        """
        limelight_table.putNumber("pipeline", pipeline)
    
    def toggleFrontLimelightMode(self):
        """Toggle the front Limelight between driver and vision modes"""
        current_mode = self.limelight_front.getNumber("camMode", 0)
        new_driver_mode = current_mode == 0  # If currently in vision mode (0), switch to driver mode (1)
        self.setDriverMode(self.limelight_front, new_driver_mode)
        mode = "driver" if new_driver_mode else "vision processing"
        print(f"Front Limelight switched to {mode} mode")
    
    def toggleRearLimelightMode(self):
        """Toggle the rear Limelight between driver and vision modes"""
        current_mode = self.limelight_rear.getNumber("camMode", 0)
        new_driver_mode = current_mode == 0  # If currently in vision mode (0), switch to driver mode (1)
        self.setDriverMode(self.limelight_rear, new_driver_mode)
        mode = "driver" if new_driver_mode else "vision processing"
        print(f"Rear Limelight switched to {mode} mode")