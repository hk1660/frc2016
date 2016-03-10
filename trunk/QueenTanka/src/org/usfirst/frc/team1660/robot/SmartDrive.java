package org.usfirst.frc.team1660.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDrive {

	final int A_BUTTON = 1;
	final int B_BUTTON = 2;
	final int X_BUTTON = 3;
	final int Y_BUTTON = 4;
	final int LB_BUTTON = 5;
	final int RB_BUTTON = 6;
	final int BACK_BUTTON = 7;
	final int START_BUTTON = 8;
	final int LEFT_JOY_BUTTON = 9;
	final int RIGHT_JOY_BUTTON = 10;
	final int LEFT_SIDEWAYS_AXIS = 0;
	final int LEFT_UP_AXIS = 1;
	final int LT_AXIS = 2;
	final int RT_AXIS = 3;
	final int RIGHT_SIDEWAYS_AXIS = 4;
	final int RIGHT_UP_AXIS = 5;
	final int POV_UP = 0;
	final int POV_LEFT = 90;
	final int POV_DOWN = 180;
	final int POV_RIGHT = 270;
	
	public CANTalon left1 = new CANTalon(1);
	public CANTalon left2 = new CANTalon(2);
	public CANTalon left3 = new CANTalon(3);
	public CANTalon right1 = new CANTalon(4);
	public CANTalon right2 = new CANTalon(5);
	public CANTalon right3 = new CANTalon(6);

	Joystick xDrive = new Joystick(0);
	double leftAxis = xDrive.getRawAxis(1);
    double rightAxis = xDrive.getRawAxis(5);
    
    int encThreshold = 1400;


	public SmartDrive() { // constructor
		//basicTinkDrivePID();			
		basicTinkDriveFollowers();			

	}

	/* Driving the robot with Joysticks */
	public void joyTinkDrive() {

		double leftSpeed = leftAxis;
		double rightSpeed = rightAxis;

		tinkDrive(leftSpeed, rightSpeed);

		// Sends the values from the joysticks axis to the dashboard
		SmartDashboard.putDouble("Left Axis " + leftAxis, leftSpeed);
		SmartDashboard.putDouble("Right Axis" + rightAxis, rightSpeed);

	}

	/*
	 * Driving the robot using encoders to adjust for different belt tensioning
	 */
	public void tinkDrive(double desiredLeftSpeed, double desiredRightSpeed) {

		// set initial scaling values at 1.0
		double leftScaleFactor = 1.0;
		double rightScaleFactor = 1.0;

		double checkLeftScale = getScaleFactor("left", desiredLeftSpeed, desiredRightSpeed);
		double checkRightScale = getScaleFactor("right", desiredLeftSpeed, desiredRightSpeed);

		// scale one of the sides down to match the other side
		if (desiredRightSpeed < desiredLeftSpeed) {
			leftScaleFactor = 1 - checkLeftScale;
		}
		if (desiredLeftSpeed < desiredRightSpeed) {
			rightScaleFactor = 1 - checkRightScale;
		}
		

		// set the speeds of the motors
		left1.set(-desiredLeftSpeed * leftScaleFactor);
		right1.set(desiredRightSpeed * rightScaleFactor);

	} 

	/*
	 * How much should we adjust the motor values based on the joysticks and
	 * encoder values? ("ERROR")
	 * 
	 */
	public double getScaleFactor(String side, double leftSpeed, double rightSpeed) {

		double scaleFactor = getActualMotorRatio(side) - getExpectedRatio(side, leftSpeed, rightSpeed);
		return scaleFactor;
	}
	
	
	/*
	 * How different are the desired speeds (from joystick or Auto) between left and
	 * right? ("SETPOINT")
	 */
	public double getExpectedRatio(String side, double leftSpeed, double rightSpeed) { 
		double expectedRatio;

		if (side.equals("left")) {
			expectedRatio = leftSpeed - rightSpeed / leftSpeed;
		} else {
			expectedRatio = rightSpeed - leftSpeed / rightSpeed;
		}

		SmartDashboard.putDouble("Expected Ratio", expectedRatio);
		return expectedRatio;
	}

	/*
	 * How different are the motors actually moving between left and right? ("PROCESS VALUE")
	 */
	public double getActualMotorRatio(String side) {

		double actualRatio;
		if (side.equals("left")) {
			actualRatio = (left1.getEncVelocity() - right1.getEncVelocity()) / left1.getEncVelocity();
		} else {
			actualRatio = (right1.getEncVelocity() - left1.getEncVelocity()) / right1.getEncVelocity();
		}

		SmartDashboard.putDouble("ActualMotorRatio", actualRatio);
		return actualRatio;

	}



	/* Set P values of control loop */
	public void encValue() {
		
		int leftEnc = left1.getEncPosition();
		int rightEnc = right1.getEncPosition();
		
		if (leftEnc - rightEnc >= encThreshold) {
			left1.setP(0.6);
		}

		if (rightEnc - leftEnc >= encThreshold) {
			right1.setP(0.6);

		}		
		SmartDashboard.putInt("leftEnc", leftEnc);
		SmartDashboard.putInt("rightEnc", rightEnc);

	}


private double squareInputWithThreshold(double value){
	if(value < 0.05 && value > -0.05 ){
		return 0;
	}
	
	if (value > 0){
		return 0.2 + (Math.pow(value, 2) * 0.8);
	}
	return -0.2 + (Math.pow(value, 2) * -0.8);
}

	

	/* Basic drivetrain movement */
	/* basic PID control for left & right side of drivetrain
	 * 
	 */
public void basicTinkDrivePID(){
	left1.changeControlMode(CANTalon.TalonControlMode.Speed);
	right1.changeControlMode(CANTalon.TalonControlMode.Speed);
	left1.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
	right1.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
	left1.setPID(1.0, 0.0, 0.0);
	right1.setPID(1.0, 0.0, 0.0);


		
}

public void basicTinkDriveFollowers(){
	left2.changeControlMode(TalonControlMode.Follower);
	left2.set(1);
	left3.changeControlMode(TalonControlMode.Follower);
	left3.set(1);
	right2.changeControlMode(TalonControlMode.Follower);
	right2.set(4);
	right3.changeControlMode(TalonControlMode.Follower);
	right3.set(4);	
}

	public void basicTinkDrive() { 

		double leftAxis = xDrive.getRawAxis(LEFT_UP_AXIS);
		double rightAxis = xDrive.getRawAxis(RIGHT_UP_AXIS);

		//basicTinkDrivePID();
		basicTinkDriveFollowers();
		
		left1.set(-1 * squareInputWithThreshold(leftAxis));
		right1.set(squareInputWithThreshold(rightAxis) * 0.8);

		SmartDashboard.putDouble("Left Motor Enc", left1.getEncPosition());
		SmartDashboard.putDouble("Right Motor Enc", right1.getEncPosition());
		SmartDashboard.putDouble("Left Motor Enc Speed", left1.getEncVelocity());
		SmartDashboard.putDouble("Right Motor Enc Speed", right1.getEncVelocity());

	}
	
	//accessor method for right encoder
	public double rightEncPosition(){
		SmartDashboard.putDouble("Right Motor Enc", right1.getEncPosition());
		SmartDashboard.putDouble("Right Motor Enc Speed", right1.getEncVelocity());

		return right1.getEncPosition();
	}
	
	//zero the drive encoders
	public void zeroRightEnc(){
		right1.setEncPosition(0);
	}
	public void zeroLeftEnc(){
		left1.setEncPosition(0);
	}
	
	
	public void autoTinkDrive(double leftSpeed, double rightSpeed) { 

		//basicTinkDrivePID();
		basicTinkDriveFollowers();
		
		double leftDriveScale = 1.0;
		double rightDriveScale = 1.0;
		
		left1.set(squareInputWithThreshold(leftSpeed) * -leftDriveScale);
		right1.set(squareInputWithThreshold(rightSpeed) * rightDriveScale);

		encDash();
	}

	//Encoder on smartdash
	public void encDash(){
		SmartDashboard.putDouble("Left Motor Enc", left1.getEncPosition());
		SmartDashboard.putDouble("Right Motor Enc", right1.getEncPosition());
		SmartDashboard.putDouble("Left Motor Enc Speed", left1.getEncVelocity());
		SmartDashboard.putDouble("Right Motor Enc Speed", right1.getEncVelocity());

	}
	
	
	// combo sides Encoder
	public void autoEncDrive(int desiredInches, double speed) {

		left1.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		right1.changeControlMode(CANTalon.TalonControlMode.PercentVbus);

		int autoEncoderFoot = 514;
		int autoEncoderInch = autoEncoderFoot / 12;

		zeroRightEnc();
		zeroLeftEnc();

		boolean rightFlag = true;
		boolean leftFlag = true;

		while (rightFlag || leftFlag) {
			if (right1.getEncPosition() < desiredInches * autoEncoderInch) {
				right1.set(speed);
				encDash();
			} else {
				right1.set(0);
				rightFlag = false;
			}

			if (left1.getEncPosition() < desiredInches * autoEncoderInch) {
				left1.set(-speed);
				encDash();
			} else {
				left1.set(0);
				leftFlag = false;
			}

			encDash();
		}
		
	}
		/* Driving the robot with Joysticks arcade Drive */
		public void arcadeDrive() {
		
			double up = xDrive.getRawAxis(RIGHT_UP_AXIS);
			double side = xDrive.getRawAxis(RIGHT_SIDEWAYS_AXIS);
			
			double leftSpeed = squareInputWithThreshold(-up  + side);
			double rightSpeed =squareInputWithThreshold(-up  - side);

			basicTinkDriveFollowers();
			
			left1.set(leftSpeed);
			right1.set(-rightSpeed);

			encDash();		
	}	
	
}
