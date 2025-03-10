# Elevator
* heights to go to ( I already added a print statement that outputs the current calculated height of the elevator )
* not high priority: tune the pid and ff more
* clamp setpoint for pids ( already have a lower setpoint bound. just need to add an upper one by dragging the elevator all the way up and putting the print statement into robot periodic so it prints even if the robot is disabled so we can jsut drag the elevator all the way up and record that value )

# End Effector
* intake coral ( this is top priority )
  * use sensors ( the order is "x" is pressed on the controller and the intake motors start up. then when a coral is detected by the EE side canrange then detected by the funnel side canrange, only then do the intake motors stop. Maybe also program in a timeout so if "x" is pressed unintentionally, stop the motors after ~10 seconds or so )
* dispense coral ( just have this be a button that is held down since that is much easier than having it be pressed and the motors always run for 0.5 seconds. )
* intake coral is now working, we just need to work on the timing of the sensor detection. Right now it when it stops detecting the inside sensor the coral is out too far. We will need to slowly reverse the intake until the inside sensor is tripped again. 
* intake algae ( sense the current in the motors to know when the algae is gripped. maybe have the motor try and hold that position or current with a pid controller once an algae has been detected. )
* dispense algae ( the way that the motor turns will be different for the different heights that the algae was taken on so there needs to ba a variable that keeps track of what height the elevator was at when it was picked up. )
* algae arm pid ( maybe ff arm ) ( simple. just needs tuning )
* clamp setpoint for pids ( need to print out the current measurement of angle in rotations then cap it on boht ends. )