package org.usfirst.frc.team1660.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.USBCamera;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.NamedSendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ShapeMode;

public class CamImage {

	//FIELDS//

	static CameraServer camInspection = CameraServer.getInstance();
	static USBCamera jamesey = new USBCamera ("cam1");
	
	
	
	static Timer timerA = new Timer();
	static double timerB = timerA.get();
	int session;
	Image frame;
    
	//raw image arra

	//centerX value of target (col)

	//centerY value of target (row)

	//size of target



	//METHODS//
	// (1) Initialization Method for all pre-match functions
	public void camInit(){
		camInspection.setQuality(50); 
		camInspection.startAutomaticCapture(jamesey);
		
		//usbCam.startCapture();

		//frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);

		// the camera name (ex "cam0") can be found through the roborio web interface
		//session = NIVision.IMAQdxOpenCamera("cam0",
				//NIVision.IMAQdxCameraControlMode.CameraControlModeController);
		//NIVision.IMAQdxConfigureGrab(session);


	}

	// (2) Overall Processing Method to be called by Robot.java
	public void camProcessing(){
		//NIVision.IMAQdxStartAcquisition(session);

		/**
		 * grab an image, draw the circle, and provide it for the camera server
		 * which will in turn send it to the dashboard.
		 */
	/*	NIVision.Rect rect = new NIVision.Rect(10, 10, 100, 100);


		NIVision.IMAQdxGrab(session, frame, 1);
		NIVision.imaqDrawShapeOnImage(frame, frame, rect,
				DrawMode.DRAW_VALUE, ShapeMode.SHAPE_OVAL, 0.0f);
*/
		//CameraServer.getInstance().setImage(frame);

		/** robot code here! **/
		Timer.delay(0.005);		// wait for a motor update time
	}



	//Method to capture values from camera



	//Method to analyze camera image for stuff

	// (3) Kill the camera
	public void camKill(){
		//camInspection.stopCapture();
		//NIVision.IMAQdxStopAcquisition(session);

	}



	//ACCESSOR METHODS


	//MUTATOR METHODS




}
