package org.usfirst.frc.team1660.robot;

import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
//import edu.wpi.first.wpilibj.Compressor;
//import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.Solenoid;

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
	final int POV_LEFT = 270;
	final int POV_DOWN = 180;
	final int POV_RIGHT = 90;

	/* Channels for the Motors */
	SmartDrive smartDrive = new SmartDrive();
	CANTalon launcherRight = new CANTalon(9);
	CANTalon launcherLeft = new CANTalon(8);
	CANTalon armMotor = new CANTalon(7);
	CANTalon armMotor2 = new CANTalon(10); 
	Talon strongCollector = new Talon(0);

	/* Pneumatics */	
	Relay angler = new Relay(1);
	Relay pusher = new Relay(2);
	Relay airC = new Relay(3);
	DigitalInput pressureSwitch = new DigitalInput(3);
	boolean flag = false;
	//Relay sallyPortHook = new Relay(3);
	//Compressor c = new Compressor(12);
	//Solenoid angler1 = new Solenoid(0);
	//Solenoid angler2 = new Solenoid(1);
	//Solenoid tester = new Solenoid(7);
	//DoubleSolenoid angler = new DoubleSolenoid(0,1);
	//DoubleSolenoid pusher = new DoubleSolenoid(2,3);
	//DoubleSolenoid sallyPortHook = new DoubleSolenoid(4,5);
	
	/* Sensor Setup */
	DigitalInput armLimiterFloor = new DigitalInput(0);
	DigitalInput armLimiterBack = new DigitalInput(1);
	AnalogInput batman = new AnalogInput(0);
	boolean armLimitFloor = !armLimiterFloor.get();
	boolean armLimitBack = !armLimiterBack.get();

	
	/* ArmStrong Angles (DONASHIA) */
	int ENC_SCALE = 20; 					//scale from degrees to armstrong encoder bips
	int startAngleValue = (int) (0.0 * ENC_SCALE);		//60
	int drawbridgeAngleValue = (int) (15.0 * ENC_SCALE);
	int collectorAngleValue = 1050;		//(int) (70.0 * ENC_SCALE);
	int floorAngleValue = 1400;  //(int)(85.0 * ENC_SCALE);
	int desiredAngleValue = startAngleValue;
	
	/* Timers */
	Timer timerAuto = new Timer();
	double timerA = timerAuto.get();
	Timer timerSpit = new Timer();
	
	boolean lowGoalFlag = false;

	
	//SmartDashboard Auto Strategy
	SendableChooser strategy = new SendableChooser();


/* 3 MAIN ROBOT METHODS */
	public void robotInit() {

		strategy.addObject("Go Forwad Strategy", new Integer(1));
	    strategy.addObject("Chival Strategy", new Integer(2));
	    strategy.addObject("Portcullis Strategy", new Integer(3));
	    SmartDashboard.putData("strategy selector", strategy);

		armMotor2.changeControlMode(TalonControlMode.Follower);
		armMotor2.set(7);
		
		//exime.camInit();
	}

	public void autonomous() {
        
		robotInit();		
		int currentStrategy = (int) strategy.getSelected(); 
    
		while (isAutonomous() && isEnabled()) {
			
			  driveForwardStrategy();
		}
	}

	public void operatorControl() {

		while (isOperatorControl() && isEnabled()) {

			//jameseyTestCamera();
            
		    autoCompressor();
		  // humanCompressor();
		    checkPressureSwitch();
		    checkLimitSwitches();
			checkUltrasonic();
			
			smartDrive.basicTinkDrive();
			// smartDrive.joyTinkDrive();
			
			//simpleArmstrongMove();
			armMove();
		    
			//simpleCollector();
			comboCollector();
			
			simpleLauncherWheels();
			simpleLauncherTrigger();
			lowGoalSpit();
			
			simpleLauncherAngle();
			//highGoalLaunch();
			
			//ourTable.run();
		
			Timer.delay(0.007); // wait 5ms to avoid hogging CPU cycles
		}

	}


	
/* COMBO JOYSTICK METHODS */

	/*Collector method that spins collector and motor wheels in simultaneously
	 */
	private void comboCollector() {

		double collectTrigger = xMan.getRawAxis(LEFT_UP_AXIS);
		
		if(collectTrigger > 0.05){
			collectWheels(0.4);
			launchWheels(1.0);
		} else if( collectTrigger < -0.05){
			collectWheels(-0.4);
			launchWheels(-1.0);
		} else if(!lowGoalFlag){ //don't interfere with lowGoalSpit method
			collectWheels(0.0);
			launchWheels(0.0);
		}
		
		SmartDashboard.putDouble("Collecting Boulder Axis",	collectTrigger);
	}
	
	/* Move ArmStrong with Joystick (DONASHIA/ ELIJAH) */
	public void armMove() {
				 
		armMotor.changeControlMode(TalonControlMode.Position);
		armMotor.setFeedbackDevice(CANTalon.FeedbackDevice.QuadEncoder);
		armMotor.setPID(1.200, 0.001, 0.010);
		
		armMotor2.changeControlMode(TalonControlMode.Follower);
		armMotor2.set(7);
		
		
		//Check if a limit swith is hit before moving
		if (armLimitFloor == true) {
			armMotor.setEncPosition(floorAngleValue);
			desiredAngleValue = floorAngleValue -20;
			SmartDashboard.putString("Arm Moving?", "Zeroed at Floor!");
			
		} else if(armLimitBack == true ){  //|| armMotor.getOutputCurrent() > 2.0){
			armMotor.setEncPosition(startAngleValue);
			desiredAngleValue = startAngleValue + 40;
			SmartDashboard.putString("Arm Moving?", "STOP at Back!");
			
		} else {

			// Decide which angle to use based on buttons (Samuel Gonzalez)
			if (xMan.getPOV() == POV_UP) {
				desiredAngleValue = startAngleValue + 40;
				SmartDashboard.putString("POV", "UP");
			} else if (xMan.getPOV() == POV_RIGHT) {
				desiredAngleValue = collectorAngleValue;
				SmartDashboard.putString("POV", "RIGHT");
			} else if (xMan.getPOV() == POV_LEFT) {
				desiredAngleValue = drawbridgeAngleValue;
				SmartDashboard.putString("POV", "LEFT");
			} else if (xMan.getPOV() == POV_DOWN) {
				desiredAngleValue = floorAngleValue;
				SmartDashboard.putString("POV", "DOWN");
			} else if(xMan.getRawAxis(RIGHT_UP_AXIS) < -0.2){
				desiredAngleValue -= 20;
			} else if(xMan.getRawAxis(RIGHT_UP_AXIS) > 0.2){
				desiredAngleValue += 20;
			}
			
		}
		
		//move arm
		armMotor.set(desiredAngleValue);
		
		SmartDashboard.putNumber("Arm Encoder", armMotor.getEncPosition());
		SmartDashboard.putNumber("desired enc angle Value", desiredAngleValue);
		SmartDashboard.putNumber("Armstrong Voltage",armMotor.getOutputCurrent());
	}
	
	
	/* Joystick Method to Spit Boulders into Low Goal */
	public void lowGoalSpit() {

		SmartDashboard.putDouble("Spit Timer", timerSpit.get());

		if (xMan.getRawAxis(LT_AXIS) > 0.5 && lowGoalFlag == false) {
			desiredAngleValue = drawbridgeAngleValue; // raise the armstrong out
														// of the way, needs
														// comboCollector!
			collectWheels(-0.6);
			lowerLauncher(); // angle launcher down
			launchWheels(-1.0); // start spinning the launcher wheels out
			lowGoalFlag = true; // flip the flag
			timerSpit.start();
			timerSpit.reset(); // reset the clock to 0
		}

		// wait for wheels to speed up to push ball
		if (timerSpit.get() > 0.5) {
			launchTrigger(); // trig boulder forward
			lowGoalFlag = false; // flip the flag back
			timerSpit.stop();
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

	/* Put Pressure Switch Values on SmartDashboard */
	public void checkPressureSwitch(){
		boolean pressureValue = !pressureSwitch.get();
		SmartDashboard.putBoolean("PressureSwitchTest", pressureValue);
	}
	
	/* Put Limit Switch Values on SmartDashboard */
	public void checkLimitSwitches(){
		armLimitFloor = !armLimiterFloor.get();
		armLimitBack = !armLimiterBack.get();
		double checkCurrent = armMotor.getOutputCurrent();
		SmartDashboard.putDouble("Arm Current", checkCurrent);
		SmartDashboard.putBoolean("Arm Limiter Floor", armLimitFloor);
		SmartDashboard.putBoolean("Arm Limiter Back", armLimitBack);
		
	}
	
	
	
	// Turn Compressor on & off with Pressure Switch 
	public void autoCompressor() {
		
		airC.setDirection(Relay.Direction.kBoth);

		// Is it on or not?
		boolean pressureOn = pressureSwitch.get();
		SmartDashboard.putBoolean("Pressure Switch", pressureOn);

		// Turn on based on pressure switch
		if (pressureOn == true) {
			airC.set(Relay.Value.kOff);
			SmartDashboard.putString("Compressor", "PSwitched OFF");
		} else {
			airC.set(Relay.Value.kForward);
			SmartDashboard.putString("Compressor", "PSwitched ON");
		}

	}
		// Turn Compressor on & off with Pressure Switch 
		public void humanCompressor() {
	
		// Manually change through Manipulator Joystick
		if (xMan.getRawButton(A_BUTTON)==true ){  
				airC.set(Relay.Value.kForward);
				SmartDashboard.putString(  "Compressor", "human On");
			}                     		
		else if (xMan.getRawButton(B_BUTTON)==true){    
				airC.set(Relay.Value.kOff);
				SmartDashboard.putString( "Compressor", "human Off");
			 }
		
			
}

	
	/*Check Value of Ultrasonic Sensor in Inches */
	public void checkUltrasonic(){
		
		
	}
		
	/*Camera check method */
	/*public void jameseyTestCamera(){

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
	/*	double x = table.getNumber("centerX", 0.0);
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
	*/
	
/* SIMPLE JOYSTICK METHODS */

	/* Joystick Method to Collect & Spit out Boulders */
	private void simpleCollector() {
		double speed = xMan.getRawAxis(LEFT_UP_AXIS);
		collectWheels(-speed);
		SmartDashboard.putDouble("Collect LeftUpAxis",	speed);
	}

	/* Joystick method to move Armstrong up & down manually */
	public void simpleArmstrongMove() {
		
		armMotor.changeControlMode(TalonControlMode.PercentVbus);		//go to default control mode
		armMotor2.changeControlMode(TalonControlMode.Follower);
		armMotor2.set(7);
	
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
		} else if (!lowGoalFlag && xMan.getRawAxis(LEFT_UP_AXIS)<0.2 && xMan.getRawAxis(LEFT_UP_AXIS)>-0.2 ) {
			launchWheels(0.0);		}
	}
	
	/* Joystick method to trigger the launcher (JATARA & DARYLE) */
	public void simpleLauncherTrigger() {
		if (xMan.getRawButton(LB_BUTTON) == true) {
			launchTrigger();
			SmartDashboard.putString("SimpleTrigger","LB_push");

		} else {
			launchRetract();
			SmartDashboard.putString("SimpleTrigger","LB_retract");
		}
	}

	/* Joystick method to adjust angle of Launcher */
	public void simpleLauncherAngle() {
		if (xMan.getRawButton(RB_BUTTON)) {
			SmartDashboard.putString("SimpleLauncher","RT_lowering!");
			lowerLauncher();
		} else if (xMan.getRawAxis(RT_AXIS) > 0.5){
			raiseLauncher();
			SmartDashboard.putString("SimpleLauncher","RB_raising!");			
		}
	}


/* AUTO STRATEGY METHODS */
	
	/*AUTO method if lined up in front of LowBar */
	void driveForwardStrategy(){
		double speed = 0.8;
		smartDrive.autoEncDrive(72, speed);
		
		
		//Timer timerA = new Timer();
		//timerA.start();
		
	    //int autoEncoderFoot = 514;
	    //smartDrive.zeroRightEnc();
	    
	    //while(timerA.get() < 5.0 && smartDrive.rightEncPosition() < 2 * autoEncoderFoot){    
	    /*
		while(smartDrive.rightEncPosition() < (6 * autoEncoderFoot)){
	    	goForward(speed);
		}
	    
	    while(true){
	    	goForward(0);
	    }
	    */
	}
	
	/*AUTO method if lined up in front of Cheval de Frise*/
	void chevalStrategy() {
		double speed = 0.5;
		Timer timerA = new Timer();
		timerA.start();
		while(timerA.get() < 3.0){
			goForward(speed);
		}
		while(timerA.get() > 3.0 && timerA.get() < 4.0) {
			armMotor.set(floorAngleValue);
		}
		while(timerA.get()> 4.0 && timerA.get() < 7.0){
			goForward(speed); 
		}
}
	 
	/*AUTO method if lined up in front of Portcullis */
	public void portcullisStrategy() {
		double speed = 0.5;
		Timer timerA = new Timer();
		timerA.start();
		while (timerA.get() < 3.0) {
			goForward(speed);
		}
		while (timerA.get() > 3.0 && timerA.get() < 4.0) {
			armMotor.set(floorAngleValue);
		}
		while (timerA.get() > 4.0 && timerA.get() < 4.5) {
			goForward(speed);
		}
		while (timerA.get() > 4.5 && timerA.get() < 9.0) {
			armMotor.set(startAngleValue);
			goForward(speed);
		}
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

	/* Aim robot yaw based on camera image (JAMESEY, AHMED)
	 */
	public void aimRobotYaw(CamImage image) {
		// determine how to move robot based on image

		// move robot

	}

	
	/* BASIC AUTO METHODS */	
	
	/* shoots boulder at given speed */
	public void launchWheels(double speed) {
		launcherLeft.set(speed);
		launcherRight.set(-speed);
	}	
	
	/* collects or spits boulders */
	public void collectWheels(double speed){
		strongCollector.set(speed);
	}	
	
	/* pushes boulder towards wheels (JATARA & DARYLE) */
	public void launchTrigger(){
		pusher.setDirection(Relay.Direction.kBoth);
		pusher.set(Relay.Value.kForward);
	}
	
	/* retracts pistons  (JATARA & DARYLE) */
	public void launchRetract(){
		pusher.setDirection(Relay.Direction.kBoth);
		pusher.set(Relay.Value.kReverse);
	}
	
	/* Raise launcher */
	public void raiseLauncher() { 
		angler.setDirection(Relay.Direction.kBoth);
		angler.set(Relay.Value.kReverse);
	}
	/* Lower launcher */
	public void lowerLauncher(){
		angler.setDirection(Relay.Direction.kBoth);
		angler.set(Relay.Value.kForward);
	}
	 
	/* AUTO Go forward (ADONIS) */
	public void goForward(double speed) {
		smartDrive.autoTinkDrive(speed, speed);
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
	
	 

}
