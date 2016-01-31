package org.usfirst.frc.team1660.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CANTalon.TalonControlMode;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

import org.usfirst.frc.team1660.robot.CamImage;


public class HkBot extends SampleRobot {
	
	//CamImage tinkoCam = new CamImage();
    Timer timerAuto = new Timer();
    double timerA = timerAuto.get();
    Joystick xDrive = new Joystick(0);// uses/assigns ports 
    Joystick xMan = new Joystick(1);
    int session;
	//DigitalInput testLimit = new DigitalInput(0);
	double startAngle = 0;
	double secondAngle = 1000;
	double thirdAngle = 2000;
	double fourthAngle = 3000;
			
	
	

	// Channels for the wheels
    CANTalon left1	= new CANTalon(1); 
    CANTalon left2	= new CANTalon(2);
    CANTalon left3	= new CANTalon(3);
    CANTalon right1	= new CANTalon(4);
    CANTalon right2	= new CANTalon(5);
    CANTalon right3	= new CANTalon(6);
    // RobotDrive tinkoDrive = new RobotDrive(left1, right1);
    
    
    // Channels for manipulator
    //CANTalon spitLeft	= new CANTalon(7); 
    //CANTalon spitRight	= new CANTalon(8);
    //actuator
    //CANTalon dart	= new CANTalon(9);
    //pistons
    Relay pusher = new Relay(1);
    Relay compressor = new Relay(2);
    
    Relay extra = new Relay(3);
    CANTalon armMotor = new CANTalon(7);
    //Encoder rightEnc = new Encoder(null, null);
    

    public void RobotInit() {
        
    	//tinkoDrive.setInvertedMotor(MotorType.kFrontLeft, true);	// invert the left side motors
       // tinkoDrive.setExpiration(0.1);
        
        
        
        
        //tinkoCam.camInit();
        
    }
    public void autonomous(){
    	
    	
    	while(isAutonomous() && isEnabled()){
    		
    		//reachBreachScore();
    		
	
    	}
    }
        
	public void operatorControl() {
        
    	//tinkoDrive.setSafetyEnabled(true);
        
    	left2.changeControlMode(TalonControlMode.Follower);
        left2.set(1);
        left3.changeControlMode(TalonControlMode.Follower);
        left3.set(1);
        right2.changeControlMode(TalonControlMode.Follower);
        right2.set(4);
        right3.changeControlMode(TalonControlMode.Follower);
        right3.set(4);
        
        armMotor.changeControlMode(TalonControlMode.Position);
        
        while (isOperatorControl() && isEnabled()) {
        	armMove();
        	tinkDrive();
        	//tinkoCam.camProcessing();
        
        	//SmartDashboard.putNumber("Left Encoder", leftEnc.getRaw());
        	//SmartDashboard.putNumber("Right Encoder", rightEnc.getRaw());
        //SmartDashboard.putBoolean("Limit Test", testLimit.get());
            //tinkoDrive.tankDrive(xDrive, 1, xDrive, 5);
            //Manipulator Methods
        	SmartDashboard.putDouble("Axis 1", xDrive.getRawAxis(1));
        	SmartDashboard.putDouble("Axis 5", xDrive.getRawAxis(5));
            Timer.delay(0.005);	// wait 5ms to avoid hogging CPU cycles
        }
        
       // tinkoCam.camKill();
        
    }
    //Arm methods Donashia
	
	public void armMove(){
		if(xMan.getRawButton(1)==true){
			armMotor.set(startAngle);
			SmartDashboard.putNumber("Arm Encoder", armMotor.getPosition());
		}
		else if(xMan.getRawButton(2)==true){
		    armMotor.set(secondAngle);
		    SmartDashboard.putNumber("Arm Encoder", armMotor.getPosition());
		}
		else if(xMan.getRawButton(3)==true){
			armMotor.set(thirdAngle);
			SmartDashboard.putNumber("Arm Encoder", armMotor.getPosition());
		}
		else if(xMan.getRawButton(4)==true){
		    armMotor.set(fourthAngle);
		    SmartDashboard.putNumber("Arm Encoder", armMotor.getPosition());
		}
	}
	
    
   //AUTO MODE METHODS

	//Method to reach the D, Breach a Drivetrain Def, & Score on Low Goal
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
	//tinko Drive
		public void tinkDrive(){
//			left2.changeControlMode(TalonControlMode.Follower);
//	        left2.set(1);
//	        left3.changeControlMode(TalonControlMode.Follower);
//	        left3.set(1);
//	        right2.changeControlMode(TalonControlMode.Follower);
//	        right2.set(4);
//	        right3.changeControlMode(TalonControlMode.Follower);
//	        right3.set(4);
//	        
	        
			double speed = xDrive.getRawAxis(1);
			double speedTwo = xDrive.getRawAxis(5);
			left1.set(-speed);
			
			right1.set(speedTwo);
	
		
		}
	
	//go forward
	public void goForward(double speed){
		
		left1.set(speed);
		left2.set(speed);
		left3.set(speed);
		right1.set(-speed);
		right2.set(-speed);
		right3.set(-speed);
	
	}
	// right or left
	public void turnRight(double speed){
		left1.set(0);
		left2.set(0);
		left3.set(0);
		right1.set(speed);
		right2.set(speed);
		right3.set(speed);
	
	}
	// right or left
	public void turnLeft(double speed){
		left1.set(speed);
		left2.set(speed);
		left3.set(speed);
		right1.set(0);
		right2.set(0);
		right3.set(0);
	}

	//go backwards
	public void goBackward(double speed){

		left1.set(-speed);
		left2.set(-speed);
		left3.set(-speed);
		right1.set(speed);
		right2.set(speed);
		right3.set(speed);



	}
	// shoots ball at given speed
	/*public void shootBall(double speed){
		spitLeft.set(speed);
		spitRight.set(-speed);
	}
	*/
	//turns on act. motor at given speed
	/*public void turnOnActuator(double speed){
		dart.set(speed);
	}
	*/
	//move robot based on camera image
	public void moveRobot(CamImage image){
		//determine how to move robot based on image
		
		//move robot
	}
	



}

