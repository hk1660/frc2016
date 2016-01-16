package org.usfirst.frc.team1660.robot;


import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;

/**
 * This is a demo program showing how to use Mecanum control with the tinkoDrive class.
 */
public class Robot extends SampleRobot {
	
    RobotDrive tinkoDrive;
    Joystick xDrive = new Joystick(0);// uses/assigns ports 
    Joystick xMan = new Joystick(1);

    // Channels for the wheels
    final int frontLeftChannel	= 2;
    final int rearLeftChannel	= 3;
    final int frontRightChannel	= 1;
    final int rearRightChannel	= 0;
    

    public Robot() {
        tinkoDrive = new RobotDrive(frontLeftChannel, rearLeftChannel, frontRightChannel, rearRightChannel);
    	tinkoDrive.setInvertedMotor(MotorType.kFrontLeft, true);	// invert the left side motors
    	tinkoDrive.setInvertedMotor(MotorType.kRearLeft, true);		// you may need to change or remove this to match your robot
        tinkoDrive.setExpiration(0.1);

        //stick = new Joystick(joystickChannel);
        
    }
        

    /**
     * Runs the motors with tank drive.
     */
    public void operatorControl() {
        tinkoDrive.setSafetyEnabled(true);
        while (isOperatorControl() && isEnabled()) {
        	
        	// Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        	// This sample does not use field-oriented drive, so the gyro input is set to zero.
            tinkoDrive.tankDrive(xDrive, 1, xDrive, 3, true );
            
            
            
 
            
            Timer.delay(0.005);	// wait 5ms to avoid hogging CPU cycles
        }
    }
    
}
