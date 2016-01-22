package org.usfirst.frc.team1660.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.USBCamera;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.NamedSendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

public class Image {

    static USBCamera usbCam = new USBCamera();
    static Timer timerA = new Timer();
    static double timerB = timerA.get();
	
	//FIELDS//
	
		//raw image array
	
		//centerX value of target (col)
	
		//centerY value of target (row)
	
		//size of target
	

    
    //METHODS//
    // (1) Initialization Method for all pre-match functions
    public void camInit(){
    	
    		usbCam.startCapture();
		   
    }
    
    // (2) Overall Processing Method to be called by Robot.java
    public void camProcessing(){
    	
    }
    
    
    	//Method to capture values from camera
    
    
    
    	//Method to analyze camera image for stuff
    
    // (3) Kill the camera
    public void camKill(){
    			usbCam.stopCapture();
    	}


    
    //ACCESSOR METHODS
    
    
    //MUTATOR METHODS
    
    
    
	
}
