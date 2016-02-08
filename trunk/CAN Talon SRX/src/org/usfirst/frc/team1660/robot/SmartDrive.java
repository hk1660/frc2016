package org.usfirst.frc.team1660.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDrive {

	CANTalon left1 = new CANTalon(1);
	CANTalon left2 = new CANTalon(2);
	CANTalon left3 = new CANTalon(3);
	CANTalon right1 = new CANTalon(4);
	CANTalon right2 = new CANTalon(5);
	CANTalon right3 = new CANTalon(6);

	Joystick xDrive;
	double leftAxis = xDrive.getRawAxis(1);
    double rightAxis = xDrive.getRawAxis(5);

	private int encThreshold = 1400;
	private int maxEncSpeed = 680;

	public SmartDrive() { // constructor
		left2.changeControlMode(TalonControlMode.Follower);
		left2.set(1);
		left3.changeControlMode(TalonControlMode.Follower);
		left3.set(1);
		right2.changeControlMode(TalonControlMode.Follower);
		right2.set(4);
		right3.changeControlMode(TalonControlMode.Follower);
		right3.set(4);

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
		SmartDashboard.putInt("leftEnc", left1.getEncPosition());
		SmartDashboard.putInt("rightEnc", right1.getEncPosition());
		if (left1.getEncPosition() - right1.getEncPosition() >= encThreshold) {
			left1.setP(0.6);
		}

		if (right1.getEncPosition() - left1.getEncPosition() >= encThreshold) {
			right1.setP(0.6);

		}
	}


	/* Basic drivetrain movement */
	public void basicTinkDrive() {

		left1.changeControlMode(CANTalon.TalonControlMode.Speed);
		right1.changeControlMode(CANTalon.TalonControlMode.Speed);
		left1.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		right1.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		left1.setPID(1.0, 0.0, 0.0);
		right1.setPID(1.0, 0.0, 0.0);

		left1.set(leftAxis);
		right1.set(rightAxis);

	}

}
