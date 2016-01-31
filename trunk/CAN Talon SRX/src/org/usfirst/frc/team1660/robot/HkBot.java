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
    
	
    public void RobotInit() {
    	
    	/* Set Drivetrain Motors to follow Master CIM on each side*/
       	left2.changeControlMode(TalonControlMode.Follower);
        left2.set(1);
        left3.changeControlMode(TalonControlMode.Follower);
        left3.set(1);
        right2.changeControlMode(TalonControlMode.Follower);
        right2.set(4);
        right3.changeControlMode(TalonControlMode.Follower);
        right3.set(4);
        
        armMotor.changeControlMode(TalonControlMode.Position);
        
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
        	
        	//tinkoCam.camProcessing();
        
        	//SmartDashboard.putNumber("Left Encoder", leftEnc.getRaw());
        	//SmartDashboard.putNumber("Right Encoder", rightEnc.getRaw());
        	//SmartDashboard.putBoolean("Limit Test", testLimit.get());
            //tinkoDrive.tankDrive(xDrive, 1, xDrive, 5);
            
            Timer.delay(0.005);	// wait 5ms to avoid hogging CPU cycles
        }
        
       // tinkoCam.camKill();
        
    }
	
/*TELEOP METHODS*/
	
	/* 6 CIM Drivetrain with Joysticks (MATTHEW) */
	public void tinkDrive(){

		double speed = xDrive.getRawAxis(1);
		double speedTwo = xDrive.getRawAxis(5);
		left1.set(-speed);
		right1.set(speedTwo);
	
		SmartDashboard.putDouble("Axis 1", xDrive.getRawAxis(1));
    	SmartDashboard.putDouble("Axis 5", xDrive.getRawAxis(5));
	}

	
    /*Move ArmStrong with Joystick (DONASHIA)		*/
	public void armMove(){
	
		//Set armstrong to specific angles
		if(xMan.getRawButton(1)==true){
			armMotor.set(startAngle);
		}
		else if(xMan.getRawButton(2)==true){
		    armMotor.set(drawbridgeAngle);
		}
		else if(xMan.getRawButton(3)==true){
			armMotor.set(collectorAngle);
		}
		else if(xMan.getRawButton(4)==true){
		    armMotor.set(portcullisAngle);
		}
		
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
	
	/* AUTO Go forward (ADONIS) */
	public void goForward(double speed){
		
		left1.set(speed);
		left2.set(speed);
		left3.set(speed);
		right1.set(-speed);
		right2.set(-speed);
		right3.set(-speed);
	
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