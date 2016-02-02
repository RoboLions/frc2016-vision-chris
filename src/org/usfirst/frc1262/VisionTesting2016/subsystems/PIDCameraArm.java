package org.usfirst.frc1262.VisionTesting2016.subsystems;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import org.usfirst.frc1262.VisionTesting2016.RobotMap;
import org.usfirst.frc1262.VisionTesting2016.commands.PIDTrackTarget;
import org.usfirst.frc1262.VisionTesting2016.commands.TrackTarget;

import edu.wpi.first.wpilibj.CANTalon;

/**
 *
 */
public class PIDCameraArm extends PIDSubsystem {
	
	static final double DEFAULT_Y_IMAGE_SIZE = 480;
	static final double DEFAULT_X_IMAGE_SIZE = 640;
	static final double kP = 0.0015;
	static final double kI = 0.0;
	static final double kD = 0.0;
	NetworkTable contourTable = NetworkTable.getTable("GRIP/myContoursReport");
	NetworkTable sizeTable = NetworkTable.getTable("GRIP/imageSize");
	static final double[] defaultEmptyArray = new double[0];
	double yImageSize = sizeTable.getNumber("y", DEFAULT_Y_IMAGE_SIZE);
	double xImageSize = sizeTable.getNumber("x", DEFAULT_X_IMAGE_SIZE);
	double yAxisSweetSpot = yImageSize / 2;
	double xAxisSweetSpot = xImageSize / 2;
	double yAxisTolerance = yImageSize * 0.05;

	private final CANTalon armTalon = RobotMap.cameraArmArmTalon;
	
    // Initialize your subsystem here
    public PIDCameraArm() {
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.
    	super("PIDCameraArm", kP, kI, kD);
    	System.out.println("Current PID values: " + kP + ", " + kI + ", " + kD);
    	setAbsoluteTolerance(yAxisTolerance);
    	getPIDController().setContinuous(false);
    }
    
    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
        setDefaultCommand(new PIDTrackTarget());
    }
    
    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
    	double[] yValues = contourTable.getNumberArray("centerY", defaultEmptyArray);
    	double[] areas = contourTable.getNumberArray("area", defaultEmptyArray);
    	if(yValues.length > 0){
    		double yValue = yValues[0];
    		double maxArea = 0;
    		for (int i = 0; i < areas.length; i++) {
    			if (areas[i] > maxArea) {
    				maxArea = areas[i];
    				yValue = yValues[i];
    			}
    		}
    		System.out.println("yValue: " + yValue);
    		return yValue;
    	}
    	else{
    		return yAxisSweetSpot;
    	}
    }
    
    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    	System.out.println("PID output: " + output);
    	armTalon.pidWrite(output);
    }
    
    public CANTalon getMotor(){
    	return armTalon;
    }
}
