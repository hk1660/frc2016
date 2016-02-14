package org.usfirst.frc.team1660.robot;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.logging.*;



public class NetworkTablesBridge {

	//public static void main(String[] args) {
	//	new NetworkTablesBridge().run();
	//}
	
	public void run()
	{
		//NetworkTable.setClientMode();
		//NetworkTable.setIPAddress("10.16.60.67");
		
		NetworkTable table = NetworkTable.getTable("datatable");
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

		System.out.println("" + x + " " + y);

		
	
	}

}
