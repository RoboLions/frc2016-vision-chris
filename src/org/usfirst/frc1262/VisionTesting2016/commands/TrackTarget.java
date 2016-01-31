// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc1262.VisionTesting2016.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

import org.usfirst.frc1262.VisionTesting2016.Robot;

/**
 *
 */
public class TrackTarget extends Command {
	
	NetworkTable contourTable = NetworkTable.getTable("GRIP/myContoursReport");
	NetworkTable sizeTable = NetworkTable.getTable("GRIP/imageSize");
	final double DEFAULT_Y_IMAGE_SIZE = 480;
	final double DEFAULT_X_IMAGE_SIZE = 640;
	double[] defaultEmptyArray = new double[0];
	double yImageSize = sizeTable.getNumber("y", DEFAULT_Y_IMAGE_SIZE);
	double xImageSize = sizeTable.getNumber("x", DEFAULT_X_IMAGE_SIZE);
	double yAxisSweetSpot = yImageSize / 2;
	double xAxisSweetSpot = xImageSize / 2;
	double yAxisTolerance = yImageSize * 0.02;
	double xAxisTolerance = xImageSize * 0.02;
	double yAxisTopLimit = yAxisSweetSpot - yAxisTolerance;
	double yAxisBottomLimit = yAxisSweetSpot + yAxisTolerance;
	double xAxisLeftLimit = xAxisSweetSpot - xAxisTolerance;
	double xAxisRightLimit = xAxisSweetSpot + xAxisTolerance;


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public TrackTarget() {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.cameraArm);

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Starting");
    	
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	double[] yValues = contourTable.getNumberArray("centerY", defaultEmptyArray);
    	if(yValues.length > 0)
    	{	
    	    	if(yValues[0] < yAxisTopLimit){
    	    		Robot.cameraArm.getMotor().set(0.24);
    	    		try {
						wait(2);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
    	    		Robot.cameraArm.getMotor().set(0);
    	    		System.out.println("Moving down");
    	    		
    	    	}
    	    	else if(yValues[0] > yAxisBottomLimit){
    	    		Robot.cameraArm.getMotor().set(-0.24);
    	    		try {
						wait(2);
					} catch (InterruptedException e) {
						// TODO Auto-generated catch block
						e.printStackTrace();
					}
    	    		Robot.cameraArm.getMotor().set(0);
    	    		System.out.println("Moving Up");
    	    	}
    	    	else{
    	    		Robot.cameraArm.getMotor().set(0);
    	    		System.out.println(yValues[0]);
    	    	}
    	}
    	else{
    		System.out.println("No Contour data found.");
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
