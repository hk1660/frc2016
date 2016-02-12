package org.usfirst.frc.team1660.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

public class HkBot extends SampleRobot {

	SmartDrive smartDrive;
	NetworkTablesBridge ourTable = new NetworkTablesBridge();

	/* Joystick Setup */
	// Joystick xDrive = new Joystick(0); // created in the SmartMotor class
	Joystick xMan = new Joystick(1);

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

	/* Camera Setup */
	// CamImage tinkoCam = new CamImage();

	int session;
	// USBCamera cam0 = new USBCamera();
	// NetworkTablesBridge table;
	// ByteBuffer image;
	/* Channels for the Motors */

	CANTalon armMotor = new CANTalon(7);
	CANTalon launcherLeft = new CANTalon(8);
	CANTalon launcherRight = new CANTalon(9);
	Talon strongCollector = new Talon(0);

	/* Pistons */
	Relay compressor = new Relay(0);
	Relay angler = new Relay(1);
	Relay pusher = new Relay(2);
	Relay extra = new Relay(3);

	/* Sensor Setup */
	DigitalInput armLimiter = new DigitalInput(0);

	/* ArmStrong Angles (DONASHIA) */
	double startAngle = 0.0;
	double drawbridgeAngle = 45.0;
	double collectorAngle = 100.0;
	double portcullisAngle = 115.0;
	double climbAngle = -15.0;
	double currentArmAngle = startAngle;
	int armEncoderRotation = 1400;

	/* Auto Fields */
	Timer timerAuto = new Timer();
	double timerA = timerAuto.get();

	public void RobotInit() {

		armMotor.changeControlMode(TalonControlMode.Position);
		armMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		armMotor.setPID(1.0, 0.0, 0.0);
		double currentArmAngle = startAngle; // Initialize values for the
												// Armstrong

		CamImage exime = new CamImage();
		exime.camInit();
		
		// tinkoCam.camInit();
	}

	public void autonomous() {

		while (isAutonomous() && isEnabled()) {

			// reachBreachScore();

		}
	}

	public void operatorControl() {

		while (isOperatorControl() && isEnabled()) {

			// smartDrive.joyTinkDrive();
			// smartDrive.encValue();
			smartDrive.basicTinkDrive();
			simpleArmstrongMove();
			simpleCollector();
			simpleLauncher();
			// simpleLauncherAngle();

			ourTable.run();
			// armMove();
			// tinkoCam.camProcessing();
			// SmartDashboard.pnjutData(cam0.getImageData(image));

			Timer.delay(0.005); // wait 5ms to avoid hogging CPU cycles
		}

		// tinkoCam.camKill();

	}

	/* SIMPLE JOYSTICK METHODS */

	/* Joystick Method to Collect & Spit out Boulders */
	public void simpleCollector() {
		double speed = xMan.getRawAxis(LEFT_UP_AXIS);
		strongCollector.set(speed);
		SmartDashboard.putDouble("xMan LeftUpAxis",
				xMan.getRawAxis(LEFT_UP_AXIS));

	}

	/* Joystick method to move Armstrong up & down manually */
	public void simpleArmstrongMove() {
		double speed = xMan.getRawAxis(RIGHT_UP_AXIS);
		armMotor.set(speed);
	}

	/* Joystick method to spin the launcher wheels */
	public void simpleLauncher() {
		if (xMan.getRawButton(Y_BUTTON) == true) {
			launcherLeft.set(-1.0);
			launcherRight.set(1.0);
		} else if (xMan.getRawButton(X_BUTTON) == true) {
			launcherLeft.set(1.0);
			launcherRight.set(-1.0);
		} else {
			launcherLeft.set(0.0);
			launcherRight.set(0.0);
		}
	}

	/* Joystick method to adjust angle of Launcher */
	public void simpleLauncherAngle() {
		if (xMan.getRawButton(2)) {

		}

	}

	/* COMBO JOYSTICK METHODS */

	/* Move ArmStrong with Joystick (DONASHIA) */
	public void armMove() {

		armZero();
		
		// Decide which angle to use based on buttons
		if (xMan.getPOV() == POV_UP) {
			currentArmAngle = startAngle;
		} else if (xMan.getPOV() == POV_RIGHT) {
			currentArmAngle = drawbridgeAngle;
		} else if (xMan.getPOV() == POV_LEFT) {
			currentArmAngle = collectorAngle;
		} else if (xMan.getPOV() == POV_DOWN) {
			currentArmAngle = portcullisAngle;
		}

		armMotor.set(currentArmAngle * armEncoderRotation / 360);

		SmartDashboard.putNumber("Arm Encoder", armMotor.getEncPosition());
	}

	/* Zero Armstrong to starting position with limit switch */
	public void armZero() {
		if (armLimiter.get() == true) {
			armMotor.setPosition(0.0);
			currentArmAngle = startAngle;
		}
	}

	/* Joystick Method to Spit Boulders into Low Goal */

	/* Joystick Method to Launch Boulders into High Goal */

	/* OTHER TELEOP METHODS */

	/* Arm limit switch */
	public void armLimit() {
		if (armLimiter.equals(true)) {
			armMotor.set(0);
		}
		SmartDashboard.putBoolean("Arm Limiter", armLimiter.get());
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
		launcherLeft.set(speed);
		launcherRight.set(-speed);
	}

	/* Raise launcher at given speed */
	/*
	 * public void turnOnActuator(double speed) { dart.set(speed); }
	 * 
	 * /* Aim robot yaw based on camera image (JAMESEY, AHMED)
	 */
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