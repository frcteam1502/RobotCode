package org.usfirst.frc.team1502.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;

public class Robot extends IterativeRobot {

	/* SOFTWARE ERROR: GURU MEDITATION #00000022.66666666
	
	 You are now under attack by the Chris Comment (should be comet in case you don't get it), or CC for short. 
	 It has landed on your two precious and confusing java files right here and has made them somewhat less confusing.
	 
	 Press left mouse button to continue.
	 	 
	*/
	
	// Add in drive
	Drive drive;
	
	// Create Joysticks
	Joystick leftStick;
	Joystick rightStick;
	Joystick manipStick;
	
	// Motor Controllers
	CANTalon rightFrontWheel;
	CANTalon rightBackWheel;
	CANTalon leftFrontWheel;
	CANTalon leftBackWheel;
	CANTalon addTalonOne;
	CANTalon addTalonTwo;
	
	// Add SPI Gyro ADXRS450
	public ADXRS450_Gyro spiGyro;	
	
	
    public void robotInit() {
    	
    	// Initialize Joysticks
    	leftStick = new Joystick(0);
		rightStick = new Joystick(1);
		manipStick = new Joystick(2);
		
		// Initialize and Calibrate SPI Gyro
		spiGyro = new ADXRS450_Gyro();
		spiGyro.calibrate();
		CameraServer.getInstance().startAutomaticCapture();
		// Load in Drive.java with controls
		drive = new Drive(leftStick, rightStick, manipStick, 1, 2, 3, 4, 5, 6, spiGyro);

		
    }

    public void autonomousInit() {
    	spiGyro.reset();

    	
    }
    
    public void autonomousPeriodic() {
    	
    	// see Drive.java for a list of autonomous methods
    	drive.autonomousDriveForward();
    	
    }
    
    public void disabledInit(){
    
    }
    
    public void disabledPeriodic(){
    	
   
    }
    
    public void teleopInit(){
    	
    	spiGyro.reset();
    	
    }
    
    public void teleopPeriodic() {
    	
    	// this is in Drive.java (no crap)
		drive.driveFromJoysticks();	
		//System.out.println("gyro angle: " + spiGyro.getAngle() + " gyro rate: " + spiGyro.getRate());
	} 
} 