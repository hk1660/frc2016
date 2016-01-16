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
    
	public static void main(String[] args) {		
		timerA.start();
		if(timerB > 0.0 && timerB < 60.0){
			usbCam.startCapture();
		}
		if(timerB > 60.0){
			usbCam.stopCapture();
		}
	}

}
