package org.usfirst.frc.team1660.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
//import edu.wpi.first.wpilibj.RobotDrive;
//import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;

import org.usfirst.frc.team1660.robot.CamImage;


public class HkBot extends SampleRobot {
	
	/*Joystick Setup */
	Joystick xDrive = new Joystick(0);// uses/assigns ports 
    Joystick xMan = new Joystick(1);
    
    /*Camera Setup */
	//CamImage tinkoCam = new CamImage();
    Timer timerAuto = new Timer();
    double timerA = timerAuto.get();
    int session;
	
	/* Channels for the Motors */
    CANTalon left1	= new CANTalon(1); 
    CANTalon left2	= new CANTalon(2);
    CANTalon left3	= new CANTalon(3);
    CANTalon right1	= new CANTalon(4);
    CANTalon right2	= new CANTalon(5);
    CANTalon right3	= new CANTalon(6);
    
    CANTalon armMotor	= new CANTalon(7);
    CANTalon spitLeft	= new CANTalon(8); 
    CANTalon spitRight	= new CANTalon(9);
    CANTalon dart		= new CANTalon(10);
        
    // RobotDrive tinkoDrive = new RobotDrive(left1, right1);
    
    /* Pistons	*/
    Relay pusher = new Relay(1);
    Relay compressor = new Relay(2);
    Relay extra = new Relay(3);
    
    /* Sensor Setup */
    DigitalInput limit1 = new DigitalInput(1);
      
    
    /*ArmStrong Angles 		(DONASHIA) */
    double startAngle = 0.0;
	double drawbridgeAngle = 45.0;
	double collectorAngle = 90.0;
	double portcullisAngle = 115.0;
	double climbAngle = -15.0;
	
	double currentArmAngle = startAngle;
    
	
    public void RobotInit() {
    	
    	/* Set Drivetrain Motors to follow Master CIM on each side*/
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
        
        armMotor.changeControlMode(TalonControlMode.Position);
        
        /* Initialize values for the Armstrong */
        double currentArmAngle = startAngle;
		
        
    	//tinkoDrive.setInvertedMotor(MotorType.kFrontLeft, true);	// invert the left side motors
        //tinkoDrive.setExpiration(0.1);
        
        //tinkoCam.camInit();
        
    }
    public void autonomous(){
    	
    	while(isAutonomous() && isEnabled()){
    		
    		//reachBreachScore();
    		
	
    	}
    }
        
	public void operatorControl() {
        
    	//tinkoDrive.setSafetyEnabled(true);
 
        while (isOperatorControl() && isEnabled()) {
        	
        	tinkDrive();
        	armMove();
        	encValue();
        	
        	//tinkoCam.camProcessing();
        	//SmartDashboard.putBoolean("Limit Test", testLimit.get());
            //tinkoDrive.tankDrive(xDrive, 1, xDrive, 5);
            
            Timer.delay(0.005);	// wait 5ms to avoid hogging CPU cycles
        }
        
       // tinkoCam.camKill();
        
    }
	
/*TELEOP METHODS*/
	
	/* 6 CIM Drivetrain with Joysticks (MATTHEW) */
	public void tinkDrive(){
		
		//get values ("Desired" motion from Joysticks and "Actual" motion from Encoders)
		double leftJoy = xDrive.getRawAxis(1);
		double rightJoy = xDrive.getRawAxis(5);
	    double leftEncoder = left1.getEncVelocity();
	    double rightEncoder = right1.getEncVelocity();

		SmartDashboard.putDouble("leftJoy", leftJoy);
    	SmartDashboard.putDouble("rightJoy", rightJoy);
		SmartDashboard.putDouble("leftEncoder", leftEncoder);
    	SmartDashboard.putDouble("rightEncoder", rightEncoder);
	    
	    //Find the error between values
	    	//what value would you expect from the encoders at full speed, 1.0?
    	    double freeSpeed = 680;
    	    double workSpeed = freeSpeed/2;
    	    if(leftJoy == 1.0){
    	    	//left1 = workSpeed;
    	    }
    	    if(rightJoy == 1.0){
    	    	//right1 = workSpeed;
    	    }
    	
    	
	    //Adjust the motors with pid loop
	    
    		//set CANTalon control modes
            // left1.setControlMode(speed);
            // right1.setControlMode(speed);
    		//set P, I, D values
	    
	    
	    //Run the motors
		left1.set(leftJoy);
		right1.set(rightJoy);
	    	
    	
	}

	
    /*Move ArmStrong with JoyCCGHJL;'stick (DONASHIA)		*/

	public void armMove(){
	
		//Decide which angle to use based on buttons
		if(xMan.getRawButton(1)==true){
			currentArmAngle = startAngle;
		}
		else if(xMan.getRawButton(2)==true){
		    currentArmAngle = drawbridgeAngle;
		}
		else if(xMan.getRawButton(3)==true){
			currentArmAngle = collectorAngle;
		}
		else if(xMan.getRawButton(4)==true){
			currentArmAngle = portcullisAngle;
		}
		
		armMotor.set(currentArmAngle);
		
		//Move armstrong up and down manually
		
			//missing code
		
		SmartDashboard.putNumber("Arm Encoder", armMotor.getEncPosition());
	}	
	
	/*Joystick Method to Collect Boulders  */
	
	
	/*Joystick Method to Spit Boulders into Low Goal*/
	
	
	/*Joystick Method to Launch Boulders into High Goal */
	
	
	/*Joystick method to adjust angle of Launcher */
    
	
	
	
/*AUTO METHODS */

	/*Method to reach the D, Breach a Drivetrain Def, & Score on Low Goal  (ADONIS) */
	private void reachBreachScore() {
		
		//reach defense(based on time /LS)
		if(timerA < 2){
			goForward(1.0);
			
			
		}
		
		//breach rough terrain(based on time ?)
		if(timerA < 3){
			goForward(1.0);
		}

		
		//aim generally towards goal (based on gyro)
			
		
		//aim precisely at goal (based on camera)
	
		
		//drive fwd to goal (based on time)
		
		
		//score boulder (based on time)
		
		
		
		//go back to "reach"		
		   
	}	
	
	//Encoder drive-train method
	
	
	public void encValue(){ 
		int encThreshold = 1400;
		SmartDashboard.putInt("leftEnc", left1.getEncPosition());
		SmartDashboard.putInt("rightEnc", right1.getEncPosition());
		if(left1.getEncPosition() - right1.getEncPosition() >= encThreshold){
		    left1.setP(0.6);
		}
		
		if(right1.getEncPosition() - left1.getEncPosition() >= encThreshold){
		    right1.setP(0.6);
		    
		}
		
		
		
	}	
    
	
	/* AUTO Go forward (ADONIS) */
	public void goForward(double speed){
		left1.set(speed);
		right1.set(-speed);
	
	}
	
	/* AUTO Turn right (ADONIS) */
	public void turnRight(double speed){
		left1.set(0);
		right1.set(speed);
	}
	
	/* AUTO Turn left (ADONIS) */
	public void turnLeft(double speed){
		left1.set(speed);
		right1.set(0);
	}

	/* AUTO go backwards (ADONIS) */
	public void goBackward(double speed){
		left1.set(-speed);
		right1.set(speed);
	}
	
	
	/* shoots ball at given speed */
	 public void shootBall(double speed){
		spitLeft.set(speed);
		spitRight.set(-speed);
	}

	/* Raise launcher at given speed */
	public void turnOnActuator(double speed){
		dart.set(speed);
	}
	
	/* Aim robot yaw based on camera image (JAMESEY, AHMED) */
	public void aimRobotYaw(CamImage image){
		//determine how to move robot based on image
		
		//move robot
		
		
	}
	
	/*Aim launcher angle based on camera image (JAMESEY, AHMED) */
	
	


	
}