package org.usfirst.frc.team1660.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;

public class HkBot extends SampleRobot {

	/* Camera Setup */
	NetworkTable table = NetworkTable.getTable("GRIP/hkContoursReport");
	double[] defaultValue = new double[0];
	//CamImage exime = new CamImage();
	//NetworkTablesBridge ourTable = new NetworkTablesBridge();

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
	SmartDrive smartDrive = new SmartDrive();
	CANTalon launcherRight = new CANTalon(9);
	CANTalon launcherLeft = new CANTalon(8);
	CANTalon armMotor = new CANTalon(7);
	Talon strongCollector = new Talon(0);

	/* Pneumatics */
	Compressor c = new Compressor(20);
	
	Solenoid angler1 = new Solenoid(0);
	Solenoid angler2 = new Solenoid(1);
	Solenoid tester = new Solenoid(7);
	
	//DoubleSolenoid angler = new DoubleSolenoid(0,1);
	//DoubleSolenoid pusher = new DoubleSolenoid(2,3);
	//DoubleSolenoid sallyPortHook = new DoubleSolenoid(4,5);
	
	//Relay compressor = new Relay(0);
	//Relay angler = new Relay(1);
	//Relay pusher = new Relay(2);
	//Relay sallyPortHook = new Relay(3);

	/* Sensor Setup */
	DigitalInput armLimiterFloor = new DigitalInput(0);
	DigitalInput armLimiterBack = new DigitalInput(1);
	AnalogInput batman = new AnalogInput(0);

	
	/* ArmStrong Angles (DONASHIA) */
	int ENC_SCALE = 20; 					//scale from degrees to armstrong encoder bips
	int startAngleValue = (int) (0.0 * ENC_SCALE);		//60
	int drawbridgeAngleValue = (int) (15.0 * ENC_SCALE);
	int collectorAngleValue = (int) (70.0 * ENC_SCALE);
	int floorAngleValue = (int)(85.0 * ENC_SCALE);
	int desiredAngleValue = startAngleValue;
	
	/* Timers */
	Timer timerAuto = new Timer();
	double timerA = timerAuto.get();
	Timer timerSpit = new Timer();

	/*SmartDash Stuff */
	private SendableChooser armP = new SendableChooser();
	//LiveWindow libby = new LiveWindow();

/* 3 MAIN ROBOT METHODS */
	public void RobotInit() {

	//	c.setClosedLoopControl(true);
		

		//exime.camInit();
	}

	public void autonomous() {

		while (isAutonomous() && isEnabled()) {

			// reachBreachScore();

		}
	}

	public void operatorControl() {

		while (isOperatorControl() && isEnabled()) {

		
			
			
			//tester.clearAllPCMStickyFaults();
			//SmartDashboard.putBoolean("Is Blacklisted?", tester.isBlackListed());
			//SmartDashboard.putBoolean("Volt Stick Fault?", tester.getPCMSolenoidVoltageStickyFault());
			//SmartDashboard.putString("To String?", tester.toString());			
			//tester.set(false);
			//jameseyTestCamera();
            
		    checkCompressor();
			checkUltrasonic();
			
			// smartDrive.joyTinkDrive();
			smartDrive.basicTinkDrive();
			
			//simpleArmstrongMove();
			 armMove();
		    
			//simpleCollector();
			comboCollector();
			
			simpleLauncherWheels();
			lowGoalSpit();
			
			simpleLauncherAngle();
			//highGoalLaunch();
			
			simpleLauncherTrigger();
			
			//ourTable.run();

			
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
	
	/* Move ArmStrong with Joystick (DONASHIA/ ELIJAH) */
	public void armMove() {
		
//		 SmartDashboard.getDouble("Change Arm P", armMotor.setP(0.01));
		
		//libby.addActuator("arm", "motor", armMotor);
		//libby.addSensor("arm", 7, component);
		 
		armMotor.changeControlMode(TalonControlMode.Position);
		armMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		armMotor.setPID(1.200, 0.001, 0.010);
		
		
		boolean armLimitFloor = !armLimiterFloor.get();
		boolean armLimitBack = !armLimiterBack.get();
		
		//Check if a limit swith is hit before moving
		if (armLimitFloor == true) {
			armMotor.setEncPosition(floorAngleValue);
			desiredAngleValue = floorAngleValue;
			SmartDashboard.putString("Arm Moving?", "Zeroed at Floor!");
			
		} else if(armLimitBack == true){
			armMotor.setEncPosition(startAngleValue);
			desiredAngleValue = startAngleValue;
			SmartDashboard.putString("Arm Moving?", "STOP at Back!");
			
		} else {

			// Decide which angle to use based on buttons (Samuel Gonzalez)
			if (xMan.getPOV() == POV_UP) {
				desiredAngleValue = startAngleValue;
				//armMotor.setEncPosition((int) startAngleValue);
			} else if (xMan.getPOV() == POV_RIGHT) {
				desiredAngleValue = drawbridgeAngleValue;
				//armMotor.setEncPosition((int) drawbridgeAngleValue);
			} else if (xMan.getPOV() == POV_LEFT) {
				desiredAngleValue = collectorAngleValue;
				//armMotor.setEncPosition((int) collectorAngleValue);
			} else if (xMan.getPOV() == POV_DOWN) {
				desiredAngleValue = floorAngleValue;
				//armMotor.setEncPosition((int) portcullisAngleValue);
			}
		}
		
		//move arm
		armMotor.set(desiredAngleValue);
		
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

	// Turn Compressor on & off with Pressure Switch 
	public void checkCompressor() {
		
		//c.start();
		//tester.set(true);
		//angler1.set(false);
		//angler2.set(true);
		
		// Is it on or not?
		// get values from the pressure switch
		if (c.getPressureSwitchValue() == true) {		// Please show on the smartDashboard
			SmartDashboard.putBoolean("Compressor Status", c.getPressureSwitchValue());
		} else {										// Turn off
			SmartDashboard.putBoolean("Compressor Status", 	c.getPressureSwitchValue());
		}
		
	}

	
	/*Check Value of Ultrasonic Sensor in Inches */
	public void checkUltrasonic(){
		
		
	}
		
	/*Camera check method */
	public void jameseyTestCamera(){

		double[] areas = table.getNumberArray("area", defaultValue);
		System.out.print("Areas: ");
		for (double area: areas) {
			System.out.print(area + " " );
		}
		System.out.println();
		//Timer.delay(1);
		
		
/*
		try {
			Thread.sleep(1000);

		} catch (Exception e) {
			System.out.println("Yo yo yo");
		}
*/
		double x = table.getNumber("centerX", 0.0);
		double y = table.getNumber("centerY", 0.0);
		double width = table.getNumber("width", 0.0);
		double area = table.getNumber("area", 0.0);
		double height = table.getNumber("height", 0.0);
		
		SmartDashboard.putDouble("the center x value is : ", x);
		SmartDashboard.putDouble("the center Y  value is :", y);
		SmartDashboard.putDouble("the width value is : ", width);
		SmartDashboard.putDouble("the area value is : ", area);
		SmartDashboard.putDouble("the height value is : ", height);

		
	}
	
	
/* SIMPLE JOYSTICK METHODS */

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
		boolean armLimitFloor = !armLimiterFloor.get();
		boolean armLimitBack = !armLimiterBack.get();
		SmartDashboard.putBoolean("Arm Limiter Floor", armLimitFloor);
		SmartDashboard.putBoolean("Arm Limiter Back", armLimitBack);
		SmartDashboard.putNumber("Arm Encoder", armMotor.getEncPosition());
		SmartDashboard.putNumber("Arm Speed", speed);
		
		
		if (armLimitFloor == true) {
			
			armMotor.set(0.0);
			SmartDashboard.putString("Arm Moving?", "STOP! floor");
		} else if (armLimitBack == true){
			armMotor.setEncPosition(0);
			armMotor.set(0.0);
			SmartDashboard.putString("Arm Moving?", "STOP! back");
		} else if(speed >= 0.1 || speed < -0.1){
			armMotor.set(speed*1);
			SmartDashboard.putString("Arm Moving?", "Going...");
		} else {
		
			armMotor.set(0);
			SmartDashboard.putString("Arm Moving?", "stop");

			
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
	
	/* Joystick method to trigger the launcher (JATARA & DARYLE) */
	public void simpleLauncherTrigger() {
		if (xMan.getRawButton(LB_BUTTON) == true) {
			launchTrigger();
			SmartDashboard.putString("TestingTrigger","LB_launchTrigger");

		} else {
			launchRetract();
			SmartDashboard.putString("TestingTrigger","LB_launchRetract");
		}
	}

	/* Joystick method to adjust angle of Launcher */
	public void simpleLauncherAngle() {
		if (xMan.getRawButton(RB_BUTTON)) {
			raiseLauncher();
			SmartDashboard.putString("TestingSol","RB_raiseLauncher");
		} else if (xMan.getRawAxis(RT_AXIS) > 0.5){
			SmartDashboard.putString("TestingSol","RT_lowerLauncher");
			lowerLauncher();
		}
	}


/* AUTO METHODS */

	/* shoots boulder at given speed */
	public void launchWheels(double speed) {
		launcherLeft.set(speed);
		launcherRight.set(-speed);
	}
	
	/* pushes boulder towards wheels (JATARA & DARYLE) */
	public void launchTrigger(){
		//pusher.set(DoubleSolenoid.Value.kOff);

		//pusher.set(DoubleSolenoid.Value.kForward);
	}
	
	/* retracts pistons  (JATARA & DARYLE) */
	public void launchRetract(){
		//pusher.set(DoubleSolenoid.Value.kReverse);
	}
	
	/* Raise launcher */
	public void raiseLauncher() { 
		//angler1.set(true);
		//angler2.set(true);
	}
	/* Lower launcher */
	public void lowerLauncher(){
		//angler1.set(false);
		//angler2.set(false);
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
