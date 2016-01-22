package org.usfirst.frc.team1660.robot;


import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ni.vision.NIVision;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ShapeMode;

import edu.wpi.first.wpilibj.CameraServer;
//import org.usfirst.frc.team1660.robot.Image;

/**
 * This is a demo program showing how to use Mecanum control with the tinkoDrive class.
 */
public class Robot extends SampleRobot {
	
    RobotDrive tinkoDrive;
    Joystick xDrive = new Joystick(0);// uses/assigns ports 
    Joystick xMan = new Joystick(1);
    int session;
    Image frame;

    
    // Channels for the wheels
    CANTalon frontLeftChannel	= new CANTalon(2);
    CANTalon rearLeftChannel	= new CANTalon(3);
    CANTalon frontRightChannel	= new CANTalon(1);
    CANTalon rearRightChannel	= new CANTalon(0);
    

    public void RobotInit() {
        tinkoDrive = new RobotDrive(frontLeftChannel, rearLeftChannel, frontRightChannel, rearRightChannel);
    	tinkoDrive.setInvertedMotor(MotorType.kFrontLeft, true);	// invert the left side motors
    	tinkoDrive.setInvertedMotor(MotorType.kRearLeft, true);		// you may need to change or remove this to match your robot
        tinkoDrive.setExpiration(0.1);
        frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);

        // the camera name (ex "cam0") can be found through the roborio web interface
        session = NIVision.IMAQdxOpenCamera("cam0",
                NIVision.IMAQdxCameraControlMode.CameraControlModeController);
        NIVision.IMAQdxConfigureGrab(session);
        //stick = new Joystick(joystickChannel);
        
        

        
    }
        

    /**
     * Runs the motors with tank drive.
     */
    public void operatorControl() {
        tinkoDrive.setSafetyEnabled(true);
        while (isOperatorControl() && isEnabled()) {
        	
        	
        	//SmartDashboard.putString("Cam",);
        	
        	// Use the joystick X axis for lateral movement, Y axis for forward movement, and Z axis for rotation.
        	// This sample does not use field-oriented drive, so the gyro input is set to zero.
            rearRightChannel.set(xDrive.getRawAxis(0));
            
            
            
 
            
            Timer.delay(0.005);	// wait 5ms to avoid hogging CPU cycles
        }
    }
    
}
