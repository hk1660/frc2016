package org.usfirst.frc.team1660.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

public class HkBot extends SampleRobot {

	/* Camera Setup */
	CamImage exime = new CamImage();
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

	/* Channels for the Motors */
	SmartDrive smartDrive;
	CANTalon armMotor = new CANTalon(7);
	CANTalon launcherLeft = new CANTalon(8);
	CANTalon launcherRight = new CANTalon(9);
	Talon strongCollector = new Talon(0);

	/* Pneumatics */
	Compressor c = new Compressor();
	DoubleSolenoid angler = new DoubleSolenoid(0,1);
	DoubleSolenoid pusher = new DoubleSolenoid(2,3);
	DoubleSolenoid sallyPortHook = new DoubleSolenoid(4,5);
	
	//Relay compressor = new Relay(0);
	//Relay angler = new Relay(1);
	//Relay pusher = new Relay(2);
	//Relay sallyPortHook = new Relay(3);

	/* Sensor Setup */
	DigitalInput armLimiterFloor = new DigitalInput(0);
	DigitalInput armLimiterBack = new DigitalInput(1);
	
	/* ArmStrong Angles (DONASHIA) */
	int ENC_SCALE = 100; 					//scale from degrees to armstrong encoder bips
	double startAngleValue = 0.0 * ENC_SCALE;
	double drawbridgeAngleValue = 45.0 * ENC_SCALE;
	double collectorAngleValue = 100.0 * ENC_SCALE;
	double portcullisAngleValue = 115.0 * ENC_SCALE;
	double desiredAngleValue = startAngleValue;
	int armEncoderRotation = 1400;

	/* Timers */
	Timer timerAuto = new Timer();
	double timerA = timerAuto.get();
	Timer timerSpit = new Timer();


	
/* 3 MAIN ROBOT METHODS */
	public void RobotInit() {

		armMotor.changeControlMode(TalonControlMode.Position);
		armMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		armMotor.setPID(1.0, 0.0, 0.0);
		double desiredAngleValue = startAngleValue; // Initialize values for the Armstrong
		
		exime.camInit();
	}

	public void autonomous() {

		while (isAutonomous() && isEnabled()) {

			// reachBreachScore();

		}
	}

	public void operatorControl() {

		while (isOperatorControl() && isEnabled()) {

			checkCompressor();
			
			basicTinkDrive();
			// smartDrive.joyTinkDrive();
			// smartDrive.basicTinkDrive();
			
			simpleArmstrongMove();
			// armMove();

			//simpleCollector();
			comboCollector();
			
			simpleLauncherWheels();
			lowGoalSpit();
			
			// simpleLauncherAngle();
			//highGoalLaunch();
			
			ourTable.run();
			
			Timer.delay(0.005); // wait 5ms to avoid hogging CPU cycles
		}

	}


	
/* COMBO JOYSTICK METHODS */

	/*Collector method that spins collector and motor wheels in simultaneously
	 */
	public void comboCollector() {
		double speed = xMan.getRawAxis(LEFT_UP_AXIS);
		strongCollector.set(speed);
		launcherLeft.set(speed);
		launcherRight.set(speed);
		SmartDashboard.putDouble("Collecting Boulder Axis",	speed);
	}
	
	/* Move ArmStrong with Joystick (DONASHIA) */
	public void armMove() {

		boolean armLimitFloor = armLimiterFloor.get();
		boolean armLimitBack = armLimiterBack.get();
		
		//Check if a limit swith is hit before moving
		if (armLimitFloor == true) {
			armMotor.setPosition(0.0);
			armMotor.set(0.0);
			SmartDashboard.putString("Arm Moving?", "Zeroed at Floor!");
			
		} else if(armLimitBack == true){
			armMotor.set(armMotor.getPosition());
			SmartDashboard.putString("Arm Moving?", "STOP at Back!");
			
		} else {

			// Decide which angle to use based on buttons
			if (xMan.getPOV() == POV_UP) {
				desiredAngleValue = startAngleValue;
			} else if (xMan.getPOV() == POV_RIGHT) {
				desiredAngleValue = drawbridgeAngleValue;
			} else if (xMan.getPOV() == POV_LEFT) {
				desiredAngleValue = collectorAngleValue;
			} else if (xMan.getPOV() == POV_DOWN) {
				desiredAngleValue = portcullisAngleValue;
			}

			//move arm
			armMotor.set(desiredAngleValue);
		}
		
		SmartDashboard.putBoolean("Arm Limiter Floor", armLimitFloor);
		SmartDashboard.putBoolean("Arm Limiter Back", armLimitBack);
		SmartDashboard.putNumber("Arm Encoder", armMotor.getEncPosition());
		SmartDashboard.putNumber("desired enc angle Value", desiredAngleValue);
	}
	
	
	/* Joystick Method to Spit Boulders into Low Goal */
	public void lowGoalSpit() {

		//timerSpit.start();		//needed?
		boolean timeFlag = false;
		
		if (xMan.getRawAxis(LT_AXIS) > 0.5) {
			
			//run right away
			if(timeFlag == false){
				armMotor.set(drawbridgeAngleValue);		// raise the armstrong out of the way
				lowerLauncher();						// angle launcher down
				launchWheels(1.0);						// start spinning the launcher wheels out
				timeFlag = true;						//flip the flag				
				timerSpit.reset();						//reset the clock to 0				
			}
			//wait for wheels to speed up to push ball
			if(timerSpit.get() > 0.5){				
				launchTrigger();						// trig boulder forward	
				timeFlag = false;						//flip the flag back
			}
		}
	}

	
	/* Joystick Method to Launch Boulders into High Goal */
	public void highGoalLaunch(){
		
	}

	
	
/* UTILITY METHODS    */
	
	/* Puts Armstrong encoder values on SmartDashboard */
	public int encConvert(double userAngle) {

		int encAngle = 0;
		int ninety = 1000; // Get value from a test
		encAngle = (int) ((ninety / 90) * userAngle);
		SmartDashboard.putInt("Arm Value", encAngle);
		return encAngle;

	}

	/* Turn Compressor on & off with Pressure Switch */
	public void checkCompressor(){
		
		//c.getPressureSwitchValue();
		

		
		
	}

		
	
/* SIMPLE JOYSTICK METHODS */

	/* basic PID control for left & right side of drivetrain
	 * 
	 */
	public void basicTinkDrive() {
		Joystick xDrive = new Joystick(0); // created in the SmartMotor class
		
		CANTalon left1 = new CANTalon(1);
		CANTalon left2 = new CANTalon(2);
		CANTalon left3 = new CANTalon(3);
		CANTalon right1 = new CANTalon(4);
		CANTalon right2 = new CANTalon(5);
		CANTalon right3 = new CANTalon(6);
		
		left1.changeControlMode(CANTalon.TalonControlMode.Speed);
		right1.changeControlMode(CANTalon.TalonControlMode.Speed);
		left1.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		right1.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		left1.setPID(1.0, 0.0, 0.0);
		right1.setPID(1.0, 0.0, 0.0);

		left2.changeControlMode(TalonControlMode.Follower);
		left2.set(1);
		left3.changeControlMode(TalonControlMode.Follower);
		left3.set(1);
		right2.changeControlMode(TalonControlMode.Follower);
		right2.set(4);
		right3.changeControlMode(TalonControlMode.Follower);
		right3.set(4);		
		
		double leftAxis = xDrive.getRawAxis(LEFT_UP_AXIS);
	    double rightAxis = xDrive.getRawAxis(RIGHT_UP_AXIS);

		left1.set(leftAxis);
		right1.set(rightAxis);

	}
	

	/* Joystick Method to Collect & Spit out Boulders */
	public void simpleCollector() {
		double speed = xMan.getRawAxis(LEFT_UP_AXIS);
		strongCollector.set(speed);
		SmartDashboard.putDouble("xMan LeftUpAxis",	speed);
	}

	/* Joystick method to move Armstrong up & down manually */
	public void simpleArmstrongMove() {
		
		armMotor.changeControlMode(TalonControlMode.PercentVbus);		//go to default control mode
		double speed = xMan.getRawAxis(RIGHT_UP_AXIS);
		boolean armLimitFloor = armLimiterFloor.get();
		boolean armLimitBack = armLimiterBack.get();
		SmartDashboard.putBoolean("Arm Limiter Floor", armLimitFloor);
		SmartDashboard.putBoolean("Arm Limiter Back", armLimitBack);
		
		if (armLimitFloor == true || armLimitBack == true){
			armMotor.set(0.0);
			SmartDashboard.putString("Arm Moving?", "STOP!");
		} else {
			armMotor.set(speed);
			SmartDashboard.putString("Arm Moving?", "Going...");
		}

	}

	/* Joystick method to spin the launcher wheels */
	public void simpleLauncherWheels() {
		if (xMan.getRawButton(X_BUTTON) == true) {
			launchWheels(1.0);
		} else if (xMan.getRawButton(Y_BUTTON) == true) {
			launchWheels(-1.0);
		} else {
			launchWheels(0.0);		}
	}
	
	/* Joystick method to trigger the launcher*/
	public void simpleLauncherTrigger() {
		
		
		
	}

	/* Joystick method to adjust angle of Launcher */
	public void simpleLauncherAngle() {
		if (xMan.getRawButton(RB_BUTTON)) {
			raiseLauncher();
		} else if (xMan.getRawAxis(RT_AXIS) > 0.5){
			lowerLauncher();
		}
	}


/* AUTO METHODS */

	/* shoots boulder at given speed */
	public void launchWheels(double speed) {
		launcherLeft.set(speed);
		launcherRight.set(-speed);
	}
	
	/* pushes boulder towards wheels */
	public void launchTrigger(){
		
	}
	
	/* retracts pistons */
	public void launchRetract(){
		
		
	}
	
	/* Raise launcher */
	public void raiseLauncher() { 
		angler.set(DoubleSolenoid.Value.kForward);
	 }
	
	/* Lower launcher */
	public void lowerLauncher(){
		angler.set(DoubleSolenoid.Value.kReverse);
	}
	 
	 /* Aim robot yaw based on camera image (JAMESEY, AHMED)
	 */
	public void aimRobotYaw(CamImage image) {
		// determine how to move robot based on image

		// move robot

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
	}//luka was here

	/* AUTO go backwards (ADONIS) */
	public void goBackward(double speed) {
		smartDrive.tinkDrive(-speed, -speed);
	}
	
	/* Method to reach the D, Breach a Drivetrain Def, & 
	 * Score on Low Goal (ADONIS)
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

		// aim generally towards goal (based on time)

		// aim precisely at goal (based on camera)

		// drive fwd to goal (based on time)

		// score boulder (based on time)

		// go back to "reach"

	}


}