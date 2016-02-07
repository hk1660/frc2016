package org.usfirst.frc.team1660.robot;

import java.awt.Image;
import java.nio.ByteBuffer;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
//import edu.wpi.first.wpilibj.RobotDrive;
//import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.USBCamera;
import edu.wpi.first.wpilibj.DigitalInput;

import java.util.logging.*;

import org.usfirst.frc.team1660.robot.CamImage;

public class HkBot extends SampleRobot {

	SmartDrive smartDrive;
	NetworkTablesBridge ourTable = new NetworkTablesBridge();

	/* Joystick Setup */
	// Joystick xDrive = new Joystick(0); // created in the SmartMotor class
	Joystick xMan = new Joystick(1);

	/* Camera Setup */
	// CamImage tinkoCam = new CamImage();

	int session;
	// USBCamera cam0 = new USBCamera();
	// NetworkTablesBridge table;
	// ByteBuffer image;
	/* Channels for the Motors */

	CANTalon armMotor = new CANTalon(7);
	CANTalon spitLeft = new CANTalon(8);
	CANTalon spitRight = new CANTalon(9);
	CANTalon dart = new CANTalon(10);

	// RobotDrive tinkoDrive = new RobotDrive(left1, right1);

	/* Pistons */
	Relay pusher = new Relay(1);
	Relay compressor = new Relay(2);
	Relay extra = new Relay(3);

	/* Sensor Setup */

	DigitalInput armLimiter = new DigitalInput(0);

	/* ArmStrong Angles (DONASHIA) */
	double startAngle = 0.0;
	double drawbridgeAngle = 45.0;
	double collectorAngle = 90.0;
	double portcullisAngle = 115.0;
	double climbAngle = -15.0;
	double currentArmAngle = startAngle;

	/* Auto Fields */
	Timer timerAuto = new Timer();
	double timerA = timerAuto.get();


	public void RobotInit() {

		armMotor.changeControlMode(TalonControlMode.Position);

		/* Initialize values for the Armstrong */
		double currentArmAngle = startAngle;
		// tinkoDrive.setInvertedMotor(MotorType.kFrontLeft, true); // invert
		// the left side motors
		// tinkoDrive.setExpiration(0.1);
		// tinkoCam.camInit();

	}

	public void autonomous() {

		while (isAutonomous() && isEnabled()) {

			// reachBreachScore();

		}
	}

	public void operatorControl() {

		while (isOperatorControl() && isEnabled()) {

			smartDrive.joyTinkDrive();
			// smartDrive.encValue();
			armMove();
			collectLaunch();

			ourTable.run();

			// tinkoCam.camProcessing();
			// SmartDashboard.pnjutData(cam0.getImageData(image));

			Timer.delay(0.005); // wait 5ms to avoid hogging CPU cycles
		}

		// tinkoCam.camKill();

	}
    	
	/* TELEOP METHODS */

	/* Arm limit switch */
	public void armLimit() {
		if (armLimiter.equals(true)) {
			armMotor.set(0);
		}
		SmartDashboard.putBoolean("Arm Limiter", armLimiter.get());
	}

    /*Move ArmStrong with Joystick (DONASHIA)		*/

	public void armMove() {

		// Decide which angle to use based on buttons
		if (xMan.getRawButton(1) == true) {
			currentArmAngle = startAngle;
		} else if (xMan.getRawButton(2) == true) {
			currentArmAngle = drawbridgeAngle;
		} else if (xMan.getRawButton(3) == true) {
			currentArmAngle = collectorAngle;
		} else if (xMan.getRawButton(4) == true) {
			currentArmAngle = portcullisAngle;
		}

		armMotor.set(currentArmAngle);

		// Move armstrong up and down manually

		// missing code

		SmartDashboard.putNumber("Arm Encoder", armMotor.getEncPosition());
	}


	/* Joystick Method to Collect Boulders */
	public void eaterSpitter() {

	}

	/* Joystick Method to Spit Boulders into Low Goal */

	/* Joystick Method to Launch Boulders into High Goal */

	/* Joystick method to adjust angle of Launcher */

	/* Boulder collector Joystick Method */
	public void collectLaunch() {
		double speed = xMan.getRawAxis(1);
		spitLeft.set(speed);
		spitRight.set(speed);
		SmartDashboard.putDouble("xMan Axis 1", xMan.getRawAxis(1));
	}

	/* Puts Armstong encoder values on SmartDashboard */
	public int encConvert(double userAngle) {

		int encAngle = 0;
		int ninety = 1000; // Get value from a test
		encAngle = (int) ((ninety / 90) * userAngle);
		SmartDashboard.putInt("Arm Value", armMotor.getEncPosition());
		return encAngle;

	}

	/* AUTO METHODS */

	/* shoots ball at given speed */
	public void shootBall(double speed) {
		spitLeft.set(speed);
		spitRight.set(-speed);
	}

	/* Raise launcher at given speed */
	public void turnOnActuator(double speed) {
		dart.set(speed);
	}

	/* Aim robot yaw based on camera image (JAMESEY, AHMED) */
	public void aimRobotYaw(CamImage image) {
		// determine how to move robot based on image

		// move robot

	}

	/*
	 * Method to reach the D, Breach a Drivetrain Def, & Score on Low Goal
	 * (ADONIS)
	 */
	private void reachBreachScore() {

		// reach defense(based on time /LS)
		if (timerA < 2) {
			goForward(1.0);

		}

		// breach rough terrain(based on time ?)
		if (timerA < 3) {
			goForward(1.0);
		}

		// aim generally towards goal (based on gyro)

		// aim precisely at goal (based on camera)

		// drive fwd to goal (based on time)

		// score boulder (based on time)

		// go back to "reach"

	}

	/* AUTO Go forward (ADONIS) */
	public void goForward(double speed) {

		smartDrive.tinkDrive(speed, speed);
	}

	/* AUTO Turn right (ADONIS) */
	public void turnRight(double speed) {
		smartDrive.tinkDrive(speed, -speed);
	}

	/* AUTO Turn left (ADONIS) */
	public void turnLeft(double speed) {
		smartDrive.tinkDrive(-speed, speed);
	}

	/* AUTO go backwards (ADONIS) */
	public void goBackward(double speed) {
		smartDrive.tinkDrive(-speed, -speed);
	}

	/* AUTO Aim launcher angle based on camera image (JAMESEY, AHMED) */

}