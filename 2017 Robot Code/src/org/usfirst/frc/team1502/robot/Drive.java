package org.usfirst.frc.team1502.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;


public class Drive {
	// Declare that the hardware exists and will be in the code.
    public Talon rightFrontWheel;
    public Talon rightBackWheel;
    public Talon leftFrontWheel;
    public Talon leftBackWheel;
    
    // suggested edit:
    
//    public WheelManager wheel = new WheelManager() {
//    	Talon lf;
//    	Talon lb;
//    	Talon rf;
//    	Talon rb;
//    };
    
//  then you can reference wheels like wheels.lf, wheels.lb, wheels.rf, wheels.rb for less typing/more intuitiveness
    
    public Talon shooterWheel;
    public Talon climberOne;
    public Talon climberTwo;
    public Talon shootIntake;
    public DoubleSolenoid gearActuator;
    public Compressor compressor;
    double DEADZONE = 0.01;
    private Joystick leftStick;
    private Joystick rightStick;
    private Joystick manipStick;
    public ADXRS450_Gyro spiGyro;
    Relay relayOne;
    Relay relayTwo;
    Talon groundIntake;
    Talon agitator;
    AnalogInput distance;

    // Glorious global variables (GGV's)
    double speedChange;
    double final1;
    double final2;
    double final3;
    double final4;
    
    // Create instances of the hardware we're going to use.
    public Drive(Joystick j0, Joystick j1, Joystick j2, int w1, int w2, int w3, int w4, int w5, int w6, ADXRS450_Gyro spiGyro){
        this.leftStick = j0;
        this.rightStick = j1;
        this.manipStick = j2;
        this.spiGyro = spiGyro;
        this.rightFrontWheel = new Talon(w1);
        this.rightBackWheel = new Talon(w2);
        this.leftFrontWheel = new Talon(w5);
        this.leftBackWheel = new Talon(w6);
        this.shooterWheel = new Talon(w3);
        this.climberOne = new Talon(w4);
        this.agitator = new Talon(1);
        this.climberTwo =  new Talon(8);
        this.relayOne = new Relay(1);
        this.relayTwo = new Relay(2);
        this.shootIntake = new Talon(7);
        this.groundIntake = new Talon(3);
        this.distance = new AnalogInput(0);
        this.gearActuator = new DoubleSolenoid(11,1,3);
        this.compressor = new Compressor(11);
    }
     
    // Makes sure there's no unintended joystick drift
    private double deadZone(double i){
        if (Math.abs(i) < DEADZONE){
            i = 0.0;
        }
        return i;
    }
    // Gyro adjusted move backwards code (In autonomous straight is always defined as the gear slot
    // regardless of what is forward in the teleop code)
    public void autonomousDriveForward() {    	
    	spiGyroCorrector(0.2, "backward");
    	
    	this.rightFrontWheel.set(final1);
    	this.rightBackWheel.set(final2);
    	this.leftFrontWheel.set(final3);
    	this.leftBackWheel.set(final4);
    }
    
    public void autonomousSideGear(){
    	while(true){
    		this.rightFrontWheel.set(-.5);
    		this.rightBackWheel.set(-.5);
        	this.leftFrontWheel.set(.2);
        	this.leftBackWheel.set(.2);
        	if(spiGyro.getAngle() > 30){
        		break;
        	}
    	}
    	
    	this.rightFrontWheel.set(0);
    	this.rightBackWheel.set(0);
        this.leftFrontWheel.set(0);
        this.leftBackWheel.set(0);
    	
    }
    // Drive forward at variable speeds until it reaches a certain distance. Encoders might help with this, but here's a "skeleton" of the process.
    public void autonomousUltrasonic() {
    	boolean shooting = false;
    	double correctDistance = 11.00;
    	while(!shooting) {
    		
    		if((distance.getVoltage() / .0098) > correctDistance) {
    			
    			spiGyroCorrector(0, "forward");
    			this.rightFrontWheel.set((distance.getVoltage() / .0098) * 0.01 + final1);
        		this.rightBackWheel.set((distance.getVoltage() / .0098) * 0.01 + final2);
        		this.leftFrontWheel.set(-(distance.getVoltage() / .0098) * 0.01 + final3);
        		this.leftBackWheel.set(-(distance.getVoltage() / .0098) * 0.01 + final4);
    		}
    		else if((distance.getVoltage() / .0098) < correctDistance) {
    			
    			spiGyroCorrector(0, "backward");
    			this.rightFrontWheel.set(-(distance.getVoltage() / .0098) * 0.01 + final1);
    			this.rightBackWheel.set(-(distance.getVoltage() / .0098) * 0.01 + final2);
    			this.leftFrontWheel.set((distance.getVoltage() / .0098) * 0.01 + final3);
        		this.leftBackWheel.set((distance.getVoltage() / .0098) * 0.01 + final4);
    			
    		}
    		else {
    			
    			shooting = true;
    		}
    	}
    	while(shooting) {
    		
    		//add code to shoot the robot
    		//Maybe add warmup and shoot methods?
    	}
    	
    }
    
    // used by go
    public double maxSpeedScaleCoefficientCoeffiecient(double strafeValue, double fwdBkwdValue, double turnValue){
    	double addedValues = strafeValue + fwdBkwdValue + turnValue;
    	
    	if (addedValues == 0){
    		return 0;
    	}
    	return strafeValue / addedValues;
    }
    
    // used in go
    public double getStrafeSpeedAdjustment(double maxSpeedScaleCoefficientCoefficient){
    	return maxSpeedScaleCoefficientCoefficient * 1;
    }
    
    // This can be called to move the robot at a base speed and direction using the gyro. How Convenient!
    // If you use this, and then set the motor values to ONLY final1, final2, etc., you DO NOT have to make final3 and final4 negative.
    public void spiGyroCorrector(double baseSpeed, String direction) {
    	// Store Values
    	double[] motorSpeeds;
    	motorSpeeds = new double[2];
    
    	// Get Gyro Angle. Since it is continuous (goes from 360->361 instead of 360->1), it is divided by 360, and takes the remainder.
    	double spiGyroAngle = spiGyro.getAngle() % 360;
    	
    	// Motor Modifier
    	double tempSpeedChange = 0.05 + (spiGyroAngle * 0.01);
    	
    	// confirms in left hemisphere
    	if (spiGyroAngle < 0 && spiGyroAngle > -180 || spiGyroAngle > 180 && spiGyroAngle < 360) {
    		//rotate clockwise
    		
    		motorSpeeds[0] = tempSpeedChange;
    		motorSpeeds[1] = 0;
    		
    	}
    	
    	// confirms in right hemisphere
    	 else if (spiGyroAngle > 0 && spiGyroAngle < 180 || spiGyroAngle < -180 && spiGyroAngle > -360) {
    		// rotate counterclockwise
    		
    		motorSpeeds[0] = 0; 
    		motorSpeeds[1] = tempSpeedChange;
    		
    	}
    	
    	// figure out direction
    	if(direction == "forward") {
    		
    		final1 = baseSpeed + motorSpeeds[1];
        	final2 = baseSpeed + motorSpeeds[1];
        	final3 = -(baseSpeed + motorSpeeds[0]);
        	final4 = -(baseSpeed + motorSpeeds[0]);
    		
    	}
    	else if(direction == "backward") {
    		
    		final1 = -baseSpeed - motorSpeeds[0];
        	final2 = -baseSpeed - motorSpeeds[0];
        	final3 = baseSpeed + motorSpeeds[1];
        	final4 = baseSpeed + motorSpeeds[1];
    		
    	}
    	else if(direction == "left") {
    		
    		final1 = baseSpeed + motorSpeeds[1];
        	final2 = -(baseSpeed + motorSpeeds[1]);
        	final3 = (baseSpeed + motorSpeeds[0]);
        	final4 = -(baseSpeed + motorSpeeds[0]);
    		
    	}
    	else if(direction == "right") {
    		
    		final1 = -(baseSpeed + motorSpeeds[0]);
        	final2 = (baseSpeed + motorSpeeds[0]);
        	final3 = -(baseSpeed + motorSpeeds[1]);
        	final4 = (baseSpeed + motorSpeeds[1]);
    		
    	}
    	
    }
	
    
    /* Grabs the values from the joysticks and cubes them to give the driver a bigger range of speeds
    at the lower values. Set like this to try to ensure the driver has the greatest control over the robot
    also because drivers most likely won't need a lot of control when they are trying to go full speed
    */
    public void driveFromJoysticks() {
        double yInput = ((rightStick.getX() * rightStick.getX() * rightStick.getX()) * -1);
        double xInput = rightStick.getY() * rightStick.getY() * rightStick.getY();
        double turn = leftStick.getX() * leftStick.getX() * leftStick.getX() *-1;
        if(rightStick.getRawButton(1)){
        	turn *= -1;
        }
        xInput = deadZone(xInput);
        yInput = deadZone(yInput);
        turn = deadZone(turn);
        go(xInput, yInput,turn);
        //MecanumDrive.drive(this, yInput, xInput, turn);
    }
    
    // this is used and what the fricking fricks is this trig
    //What the fuck is this shit?
    public void go(double xInput, double yInput, double turn){
    	
        // create calculation variables
        double normalizedY;
        double normalizedX;
        double theta;
        double tanTheta;
        double cosTheta;
        double sinTheta;
       
        //create variables to hold the final values for each motor
        // these are now glorious global variables at the tops
        
        //First, calculate the drift drive out using trig
        //start by calculating theta
        //Tan of theta = opp/adj, or Y/X
        tanTheta = yInput / xInput;
        
        // you don't need all this crap just to normalize a vector :-(
        
        //apply an inverse tan function to find theta
        theta = Math.atan(tanTheta);
        
        //get the sin and cosine of theta to find x and y
        cosTheta = Math.cos(theta);
        sinTheta = Math.sin(theta);
        
        //calculate the normalized x and y values using sT and cT
        normalizedX = cosTheta * xInput;
        normalizedY = sinTheta * yInput;
        if (Double.isNaN(normalizedY)) {
            normalizedY = 0;
        } 
        if (Double.isNaN(normalizedX)) {
            normalizedX = 0;
        }
        if (yInput < 0){
            normalizedY = -1 * Math.abs(normalizedY);
        }    
        if (yInput >= 0){
            normalizedY = Math.abs(normalizedY);
        } 
        if (xInput > 0){
            normalizedX = Math.abs(normalizedX);
        }
        
        // set final values according to the trig and other factors above
        //double gyroAdjustment = this.getGyroAdjustment(turn);
        
        final1 = normalizedY - normalizedX - turn;
        final2 = -(normalizedY + normalizedX + turn);
        final3 = normalizedY + normalizedX - turn;
        final4 = -normalizedY + normalizedX - turn;
        double strafeValue = Math.abs(rightStick.getX());
        double fwdBkwdValue = Math.abs(rightStick.getY());
        double turnValue = Math.abs(leftStick.getX());
        
        // strafe adjustments
        double strafeScaleCoefficient = maxSpeedScaleCoefficientCoeffiecient(strafeValue, fwdBkwdValue, turnValue);
        double getStrafeSpeedAdjustment = getStrafeSpeedAdjustment(strafeScaleCoefficient);
        double maxSpeedScaleCoefficient = getStrafeSpeedAdjustment + 1;
        
        /* ******************************************************************************************************************************
         place button functions here, including those that can affect final motor values (unless you actually know what you're doing).
         ****************************************************************************************************************************** */
        
        int leftPovPosition = leftStick.getPOV();
        int rightPovPosition = rightStick.getPOV();
             
        // change speed of robot
        if(leftStick.getRawButton(2)){
        	speedChange = .25;
        }
        else{
        	speedChange = 1;
        }
        
        // These buttons move the robot slowly and perfectly. (Almost) perfect for tiny, accurate maneuvers.
        // FORWARD
        if(leftPovPosition == 0 || rightPovPosition == 0){
        	
        	spiGyroCorrector(0.2, "right");
        }
        // BACKWARD
        else if(leftPovPosition == 180 || rightPovPosition == 180){
        	
        	spiGyroCorrector(0.2, "left");
        }
        // LEFT
        else if(leftPovPosition == 270 || rightPovPosition == 270){
        	
        	spiGyroCorrector(0.2, "forward");
        }
        // RIGHT
        else if(leftPovPosition == 90 || rightPovPosition == 90){
        	
        	spiGyroCorrector(0.2, "backward");	
        }
        // RESET
        else if(leftStick.getRawButton(4) || rightStick.getRawButton(4)){
        	
        	spiGyro.reset();
        	
        	final1 = 0;
        	final2 = 0;
        	final3 = 0;
        	final4 = 0;
        	
        }
        //ManipStick - A is 1
        //B is 2
        //X is 3
        //Y is 4
        // shooter alone ("warmup")
        if(manipStick.getRawButton(4)){
        	climberOne.set(1);
        }
        else if(manipStick.getRawButton(2)){
        	climberOne.set(-1);
        }
        else{
        	climberOne.set(0);
        }

        
        
      /*  if(manipStick.getRawButton(4)){
        	agitator.set(.7);
        }
        else if(manipStick.getRawButton(3)){
        	agitator.set(-.7);
        }
        else{
        	agitator.set(0);
        }
*/
        // Launcher/Shooter
        if(manipStick.getRawButton(5)){
        	shootIntake.set(-1);
        	agitator.set(.7);
        }

        else{
        	shootIntake.set(0);
        	agitator.set(0);
        }
        if(manipStick.getRawButton(1)){
        	groundIntake.set(1);
        }
        else{
        	groundIntake.set(0);
        }
        if(manipStick.getRawButton(6)){
        	shooterWheel.set(-1);        
        }
        else{
        	shooterWheel.set(0);
        }
    	if(manipStick.getRawButton(7)){
    		gearActuator.set(DoubleSolenoid.Value.kForward);
    	}
    	if(manipStick.getRawButton(8)){
    		gearActuator.set(DoubleSolenoid.Value.kReverse);
    	}

        // distance sensor gear thingy
        if(rightStick.getRawButton(12)){
        	if((distance.getVoltage() / .0098) > 20){
        		
        		spiGyroCorrector(0.2, "backward");
            	
        	}
        	else{
        		
        		final1 = 0;
            	final2 = 0;
            	final3 = 0;
            	final4 = 0;
        	}

        }
        if(rightStick.getRawButton(11)){
        	if((distance.getVoltage() / .0098) < 100){
        		
        		spiGyroCorrector(0.2, "forward");
                        	
        	}
        	else{
        		
        		final1 = 0;
            	final2 = 0;
            	final3 = 0;
            	final4 = 0;
        	}

        }
        
        // shoot and move - WITH STYLE
        /*
        if(manipStick.getRawButton(3)) {
        	
        	boolean shooting = false;
        	double correctDistance = 11.00;
        	while(!shooting && !manipStick.getRawButton(4)) {
        		
        		if((distance.getVoltage() / .0098) > correctDistance) {
        			
        			
        			final1 = (distance.getVoltage() / .0098) * 0.01;
            		final2 = (distance.getVoltage() / .0098) * 0.01;
            		final3 = (distance.getVoltage() / .0098) * 0.01;
            		final4 = (distance.getVoltage() / .0098) * 0.01;
        		}
        		else if((distance.getVoltage() / .0098) < correctDistance) {
        			
        			final1 = -(distance.getVoltage() / .0098) * 0.01;
            		final2 = -(distance.getVoltage() / .0098) * 0.01;
            		final3 = -(distance.getVoltage() / .0098) * 0.01;
            		final4 = -(distance.getVoltage() / .0098) * 0.01;
        			
        		}
        		else {
        			
        			shooting = true;
        		}
        	}
        	while(shooting && !manipStick.getRawButton(4)) {
        		
        		//shoot here! wooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo
        	}
        	
        }
        */
        	System.out.println("distance: " + distance.getVoltage()/.0098);
       
        /* 
         =====================================================================================
         This is where the motors are set
         Towards each other is towards that side, the other motors are then moving away from each other
         
         			 < Front >
         			
             final3 [|-------|] final1
                     |       |
                     |       |
             final4 [|-------|] final2
         =====================================================================================
         */
        	/*PNEUMATIC CODE
        	 * PNEUMATIC CODE
        	 */

       /* 	if(rightStick.getRawButton(7)){
        		compressor.setClosedLoopControl(true);
        	}
        	else{
        		compressor.setClosedLoopControl(false);
        	}*/
        	if(rightStick.getRawButton(1)){	
            	this.rightFrontWheel.set(maxSpeedScaleCoefficient*(final1*speedChange));
            	this.rightBackWheel.set(maxSpeedScaleCoefficient*(final2*speedChange));
            	this.leftFrontWheel.set(maxSpeedScaleCoefficient*(final3*speedChange));
            	this.leftBackWheel.set(maxSpeedScaleCoefficient*(final4*speedChange));  
        	}
        	else{
        		this.rightFrontWheel.set(-maxSpeedScaleCoefficient*(final1*speedChange));
            	this.rightBackWheel.set(-maxSpeedScaleCoefficient*(final2*speedChange));
            	this.leftFrontWheel.set(-maxSpeedScaleCoefficient*(final3*speedChange));
            	this.leftBackWheel.set(-maxSpeedScaleCoefficient*(final4*speedChange)); 
            	}
        	
        	

    }   
    
}