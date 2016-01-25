package org.usfirst.frc.team1660.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

import org.usfirst.frc.team1660.robot.CamImage;


public class Robot extends SampleRobot {
	
	CamImage tinkoCam = new CamImage();
	
    RobotDrive tinkoDrive;
    Joystick xDrive = new Joystick(0);// uses/assigns ports 
    Joystick xMan = new Joystick(1);
    int session;
	DigitalInput testLimit = new DigitalInput(0);

	// Channels for the wheels
    CANTalon left1	= new CANTalon(1);
    CANTalon left2	= new CANTalon(2);
    CANTalon left3	= new CANTalon(3);
    CANTalon right1	= new CANTalon(4);
    CANTalon right2	= new CANTalon(5);
    CANTalon right3	= new CANTalon(6);
    
    Encoder leftEnc = new Encoder(null, null);
    Encoder rightEnc = new Encoder(null, null);
    
    

    public void RobotInit() {
        tinkoDrive = new RobotDrive(left1, right1);
    	tinkoDrive.setInvertedMotor(MotorType.kFrontLeft, true);	// invert the left side motors
        tinkoDrive.setExpiration(0.1);

        tinkoCam.camInit();
        
    }
        
    public void operatorControl() {
        
    	tinkoDrive.setSafetyEnabled(true);
        
        while (isOperatorControl() && isEnabled()) {
        	
        	tinkoCam.camProcessing();
        	
        	SmartDashboard.putNumber("Left Encoder", leftEnc.getRaw());
        	SmartDashboard.putNumber("Right Encoder", rightEnc.getRaw());
        	SmartDashboard.putBoolean("Limit Test", testLimit.get());
        	
        	
        	//Driving Method
            right1.set(xDrive.getRawAxis(0));
            right2.set(xDrive.getRawAxis(0)); 
            right3.set(xDrive.getRawAxis(0)); 
            left1.set(xDrive.getRawAxis(-0));
            left2.set(xDrive.getRawAxis(-0)); 
            left3.set(xDrive.getRawAxis(-0)); 

            //Manipulator Methods
            Timer.delay(0.005);	// wait 5ms to avoid hogging CPU cycles
        }
        
        tinkoCam.camKill();
        
    }
    
   
    
    
    
    
}






