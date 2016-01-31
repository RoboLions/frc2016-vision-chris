package org.usfirst.frc1262.VisionTesting2016.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

import org.usfirst.frc1262.VisionTesting2016.Robot;

/**
 *
 */
public class PIDTrackTarget extends Command {
	
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

    public PIDTrackTarget() {
        // Use requires() here to declare subsystem dependencies
    	requires(Robot.PIDCameraArm);
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Starting");
    	Robot.PIDCameraArm.setSetpoint(yAxisSweetSpot);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	double[] yValues = contourTable.getNumberArray("centerY", defaultEmptyArray);
    	double yOffset = Math.abs(yAxisSweetSpot - yValues[0]);
    	return yOffset < yAxisTolerance;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
