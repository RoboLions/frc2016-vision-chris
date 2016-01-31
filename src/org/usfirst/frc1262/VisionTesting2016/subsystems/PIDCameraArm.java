package org.usfirst.frc1262.VisionTesting2016.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import org.usfirst.frc1262.VisionTesting2016.RobotMap;
import org.usfirst.frc1262.VisionTesting2016.commands.*;
import edu.wpi.first.wpilibj.CANTalon;

/**
 *
 */
public class PIDCameraArm extends PIDSubsystem {
	
	final double DEFAULT_Y_IMAGE_SIZE = 480;
	final double DEFAULT_X_IMAGE_SIZE = 640;
	NetworkTable contourTable = NetworkTable.getTable("GRIP/myContoursReport");
	NetworkTable sizeTable = NetworkTable.getTable("GRIP/imageSize");
	double[] defaultEmptyArray = new double[0];
	double[] yValues = contourTable.getNumberArray("centerY", defaultEmptyArray);
	double yImageSize = sizeTable.getNumber("y", DEFAULT_Y_IMAGE_SIZE);
	double xImageSize = sizeTable.getNumber("x", DEFAULT_X_IMAGE_SIZE);
	double yAxisSweetSpot = yImageSize / 2;
	double xAxisSweetSpot = xImageSize / 2;
	
	private final CANTalon armTalon = RobotMap.cameraArmArmTalon;
	
    // Initialize your subsystem here
    public PIDCameraArm() {
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.
    	super("PIDCameraArm", 0.0, 0.0, 0.0);
    	getPIDController().setContinuous(false);
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }
    
    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
    	if(yValues[0] > 0){
    		return yValues[0];
    	}
    	else{
    		return yAxisSweetSpot;
    	}
    }
    
    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    	armTalon.pidWrite(output);
    }
}
