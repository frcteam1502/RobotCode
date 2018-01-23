
//import com.sun.squawk.util.MathUtils;
// import edu.wpi.first.wpilibj.DriverStation;
import java.sql.Time;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.concurrent.TimeUnit;
import edu.wpi.first.wpilibj.*;
//import org.usfirst.frc.team1502.robot.DriveStraight;
/**
 *
 * @author Robotics SE
 */
public class Drive {
    //Don't forget about the rooster (method)
    // private DigitalInput wheelPulse;
    protected Talon wheel1;
    protected Talon wheel2;
    protected Talon wheel3;
    protected Talon wheel4;
    protected Talon leftToteWacker;
    protected Talon rightToteWacker;
    //protected Victor wheel5;
    //protected Victor wheel6;
    private double DEADZONE = 0.01;
    private Joystick leftStick;
    private Joystick rightStick;
    // private Timer rpmTimer;
    // private int revolutionsWheel;
    // private double lastrpmWheel;
    // private boolean previousOpticalValueWheel;
    // private Timer collectionTimer;
    // private boolean collectionTimerEnabled;
    private Joystick manipStick;
    private AnalogGyro orientationGyro;
    // private DriverStation ds1;
    //DriveStraight driveStraight;
    double gyroZero;
    double targetInchesTraveled;
    double inchesTraveled;
    boolean adjustingGyro;
    Timer gyroCooldownTimer;
    Encoder elevatorEncoder;
    Encoder wheel1Encoder;
    Encoder wheel2Encoder;
    Encoder wheel3Encoder;
    Encoder wheel4Encoder;
    double wheel1EncoderValue;
    double wheel2EncoderValue;
    double wheel3EncoderValue;
    double wheel4EncoderValue;
    QuadratureEncoderRpmMeasurer leftRearRpmMeasurer;
    QuadratureEncoderRpmMeasurer leftFrontRpmMeasurer;
    QuadratureEncoderRpmMeasurer rightRearRpmMeasurer;
    QuadratureEncoderRpmMeasurer rightFrontRpmMeasurer;
    QuadratureEncoderRpmMeasurer elevatorRpmMeasurer;
//    Elevator lift = new Elevator(elevatorEncoder);
    boolean moveBackwards = false;
    double defaultAutoSpeedMagnitude = .28;
    double autoSpeedMagnitude;
    
    
//    protected Gyro gyro1 = new Gyro(0);
    public Drive(Joystick j1, Joystick j2, Joystick j3, int w1, int w2, int w3, int w4, int w5, int w6, AnalogGyro gyro, Encoder elevatorEncoder){
        this.orientationGyro = gyro;
    	setGyroZero();
    	setTargetInchesZero();
        this.leftStick = j1;
        this.rightStick = j2;
        this.manipStick = j3;
        this.wheel1 = new Talon(w1);
        this.wheel2 = new Talon(w2);
        this.wheel3 = new Talon(w3);
        this.wheel4 = new Talon(w4);
        //this.wheel5 = new Victor(w5);
        //this.wheel6 = new Victor(w6);
        this.elevatorEncoder = elevatorEncoder;
        wheel1Encoder = new Encoder(2, 3);
        wheel2Encoder = new Encoder(4, 5);
        //wheel3Encoder = new Encoder(6, 7);
        //wheel4Encoder = new Encoder(8, 9);
        
//        wheelPulse = new DigitalInput(7);
        // rpmTimer = new Timer();
        // collectionTimer = new Timer();
        // collectionTimerEnabled = false;

//        driveStraight = new DriveStraight(orientationGyro, this);
        this.gyroCooldownTimer = new Timer();
        
        leftRearRpmMeasurer = new QuadratureEncoderRpmMeasurer(wheel1Encoder);
        leftFrontRpmMeasurer = new QuadratureEncoderRpmMeasurer(wheel2Encoder);
     //   rightRearRpmMeasurer = new QuadratureEncoderRpmMeasurer(wheel3Encoder);
    //    rightFrontRpmMeasurer = new QuadratureEncoderRpmMeasurer(wheel4Encoder);
        elevatorRpmMeasurer = new QuadratureEncoderRpmMeasurer(elevatorEncoder);
        autoSpeedMagnitude = defaultAutoSpeedMagnitude;
        
    }
    
    public void setToMoveBackwards() {
    	moveBackwards = true;
    }
    
    public void setToMoveForwards() {
    	moveBackwards = false;
    }
    
    public void maintainState() {
    	elevatorRpmMeasurer.maintainState();
    	leftRearRpmMeasurer.maintainState();
    	leftFrontRpmMeasurer.maintainState();
    //	rightRearRpmMeasurer.maintainState();
    //	rightFrontRpmMeasurer.maintainState();
	}
    
    
    private double deadZone(double i){
        if (Math.abs(i) < DEADZONE){
            i = 0.0;
        }
        
        return i;
    }
    
    /*
    public void rpm() {
        revolutionsWheel = 0;
        // lastrpmWheel = 0;
        
        if (wheelPulse.get() == true && previousOpticalValueWheel == false) {
            revolutionsWheel++;
	}
        if (rpmTimer.get() > 1) {
            // The belt spins once for every five wheel revolutions; multiply by 60 to get per minute instead of per second
            // lastrpmWheel = revolutionsWheel * 60;
            revolutionsWheel = 0;
        }
    }
    */
    
    public void setTargetInchesZero() {
    	targetInchesTraveled = 0;
    }
    
    public void goForward(double speed) {
        wheel1.set(-1 * speed);
        wheel2.set(-1 * speed);
        wheel3.set(.9 * speed);
        wheel4.set(.9 * speed);
    }
    
    public void stop() {
        wheel1.set(0);
        wheel2.set(0);
        wheel3.set(0);
        wheel4.set(0);
    }
    
//    public boolean scaleForCollector(int collectorState) {
//        
//        boolean collecting = false;
//        
//        if (collectorState == States.DEPLOYED || collectorState == States.COLLECTING) {
//            collecting = true;
//            collectionTimer.reset();
//            collectionTimer.start();
//            System.out.println("True by state");
//            collectionTimerEnabled = true;
//        }
//        if (collectionTimer.get() < .65 && collectionTimerEnabled) {
//            collecting = true;
//            System.out.println("True by timer");
//        }
//        
//        return collecting;
//                
//        //return false;
//    }
    
    public double getLeftRearRpm() {
    	return leftRearRpmMeasurer.get();
    }
    
    public double getLeftFrontRpm() {
    	return leftFrontRpmMeasurer.get();
    }
    
    public double getRightRearRpm() {
    	return rightRearRpmMeasurer.get();
    }
    
 //   public double getRightFrontRpm() {
   // 	return rightFrontRpmMeasurer.get();
    //}
    
    public double getElevatorRpm() {
    	return elevatorRpmMeasurer.get();
    }
    
    public double maxSpeedScaleCoefficientCoeffiecient(double strafeValue, double fwdBkwdValue, double turnValue){
    	double addedValues = strafeValue + fwdBkwdValue + turnValue;
    	
    	if(addedValues == 0){
    		return 0;
    	}
    	return strafeValue / addedValues;
    	
    }
   
    public double getStrafeSpeedAdjustment(double maxSpeedScaleCoefficientCoefficient){
    	return maxSpeedScaleCoefficientCoefficient * .25;
    }
    
    public void setTargetAngle(double angle) {
    	gyroZero = angle;
    }
    
    public void autonomousGo() {
        double gyroAdjustment = this.getAutonomousGyroAdjustment();
        gyroAdjustment /= 1.75;
        //gyroAdjustment = 0;
        double speedAdjustment = this.getAutonomousSpeedAdjustment();
        
        
        double final1 = speedAdjustment + gyroAdjustment;
        double final2 = -1 * speedAdjustment + gyroAdjustment;
        double final3 = speedAdjustment + gyroAdjustment;
        double final4 = -1 * speedAdjustment + gyroAdjustment;        
        this.wheel1.set(final1);
    	this.wheel2.set(final2);
    	this.wheel3.set(final3);
    	this.wheel4.set(final4);
    }
    
    public void setAutoSpeedMagnitude(double magnitude) {
    	autoSpeedMagnitude = magnitude;
    }
    
    public void resetAutoSpeedMagnitude() {
    	autoSpeedMagnitude = defaultAutoSpeedMagnitude;
    }
    
    public double getAutonomousSpeedAdjustment() {
    	double speedAdjustment = 0;
    	
    	this.inchesTraveled = Math.abs(this.getInchesTraveled());

        SmartDashboard.putString("DB/String 2", Double.toString(inchesTraveled));
    	
    	if (inchesTraveled < targetInchesTraveled) {
    		speedAdjustment = -1 * autoSpeedMagnitude;
    		
    		if (moveBackwards) {
    			speedAdjustment *= -1;
    		}
    	}
    	
    	
    	return speedAdjustment;
    }
    
    public boolean robotHasTraveledDistance() {
    	boolean robotHasTraveledDistance = false;
    	
    	if (inchesTraveled >= targetInchesTraveled) {
    		robotHasTraveledDistance = true;
    	}
    	
    	return robotHasTraveledDistance;
    }
    
    public void setTargetDistanceInches(double inches) {
    	this.targetInchesTraveled = inches;
    }
    
    public void driveFromJoysticks() {
        //get the base input values
        double xInput = rightStick.getX() * rightStick.getX() * rightStick.getX();
        double yInput = rightStick.getY() * rightStick.getY() * rightStick.getY();
        //double turn = leftStick.getX() * leftStick.getX() * leftStick.getX();
        double turn = leftStick.getX() * leftStick.getX() * leftStick.getX();


        xInput = deadZone(xInput);
        yInput = deadZone(yInput);
        turn = deadZone(turn);
        
        go(xInput, yInput,turn);
    }
    
    public void go(double xInput, double yInput, double turn){
        // create calculation variables
        double normalizedY;
        double normalizedX;
        double theta;
        double tanTheta;
        double cosTheta;
        double sinTheta;
       
        //create variables to hold the final values for each motor
        double final1;
        double final2;
        double final3;
        double final4;
        // double final5;
        // double final6;
        
        //First, calculate the drift drive out using trig
        //start by calculating theta
        //Tan of theta = opp/adj, or Y/X
        tanTheta = yInput / xInput;
        
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
        
//        if (scaleForCollector(collectorState)) {
//            normalizedY = normalizedY / 3.5;
//            turn = turn / 3;
//        }
        
        //Adjust Gyro to Change Speed
//        double gyroMod = gyro1.getAngle() % 360;
//        double gyroAdjust = gyroMod / 100;
    	
        double gyroAdjustment = this.getGyroAdjustment(turn);
        

//    	System.out.println("Gyro: " + orientationGyro.getAngle() + " Gyro adjustment: " + gyroAdjustment);
    	
    	// Negative gyro mod means the robot has drifted counter-clockwise.
    	// Positive gyro mod means the robot has drifted clockwise.
    	// Adjust it back the other way.
        
    	
        
        
        //add the X, Y, and turn components together to get the final values for each wheel
        final1 = normalizedY - normalizedX - turn + gyroAdjustment;
        final2 = -(normalizedY + normalizedX + turn) + gyroAdjustment;
        final3 = normalizedY + normalizedX - turn + gyroAdjustment;
        final4 = -normalizedY + normalizedX - turn + gyroAdjustment;
        // final5 = normalizedY;
        // final6 = normalizedY;

        
        //System.out.println("Turn: " + turn + " Final 1: " + final1 + " Final 2: " + final2 + " Final 3: " + final3 + " Final 4: " + final4);
        //set the drive motors to their calculated speeds
//        if (leftStick.getRawButton(1)){
//        	this.driveStraight(final1, -final2, -final3, final4, normalizedX, normalizedY);
//        }
//        else{
        double strafeValue = Math.abs(rightStick.getX());
        double fwdBkwdValue = Math.abs(rightStick.getY());
        double turnValue = Math.abs(leftStick.getX());
        
        double strafeScaleCoefficient = maxSpeedScaleCoefficientCoeffiecient(strafeValue, fwdBkwdValue, turnValue);
        
        double getStrafeSpeedAdjustment = getStrafeSpeedAdjustment(strafeScaleCoefficient);
        
        double maxSpeedScaleCoefficient = getStrafeSpeedAdjustment + .35;
        
        if (rightStick.getRawButton(1)){
        	maxSpeedScaleCoefficient = 1;
        }
        /*
        if (leftStick.getRawButton(5)){
        	wheel1Encoder.reset();
        	wheel2Encoder.reset();
        	wheel3Encoder.reset();
        	wheel4Encoder.reset();
        }
        */
        
        if (leftStick.getRawButton(6)) {
        	orientationGyro.reset();
        }
            this.wheel1.set(maxSpeedScaleCoefficient*(final1));
        	this.wheel2.set(maxSpeedScaleCoefficient*(final2));
        	this.wheel3.set(maxSpeedScaleCoefficient*(final3));
        	this.wheel4.set(maxSpeedScaleCoefficient*(final4));  
        	///this.wheel5.set(-manipStick.getY());
        	///this.wheel6.set(-manipStick.getY());
	    	
        	double gyroAngle = orientationGyro.getAngle();
            String gyroAngleString = Double.toString(gyroAngle);
        
            SmartDashboard.putString("DB/String 0", gyroAngleString);
//          	System.out.println(ds1.getBatteryVoltage());
//        }
                    //ystem.out.println("one :" + wheel1.getSpeed() + " two :" + wheel2.getSpeed() + 
                    //" three :" + wheel3.getSpeed() + " four :" + wheel4.getSpeed());
        //System.out.println("LS out : " + turn);
    }   
    
//    public void adjustMotorPower(double wheel1MotorPower, double wheel1MotorPower, double wheel1MotorPower, double wheel1MotorPower){     	
//    	
//    	double motorPowerCoefficient = motorPower;
//    }
    
    public void turnToAngle(double angle) {
    	this.gyroZero = angle;
    	
    }
    
    public void turnDegrees(double degrees){
    	double gyroMod = this.orientationGyro.getAngle() % 360;
    	
    	SmartDashboard.putString("DB/String 3", "Target gyro mod value: " + (this.gyroZero + degrees));
		SmartDashboard.putString("DB/String 4", "Gyro mod value: " + gyroMod);
    	
    	
    	if (gyroMod < this.gyroZero + degrees){
    		this.wheel1.set(.35);
         	this.wheel2.set(.35);
         	this.wheel3.set(.35);
         	this.wheel4.set(.35);
    	}
    	
    	
    }
    
    public void driveStraight(double wheel1Spd, double wheel2Spd, double wheel3Spd, double wheel4Spd, double joystickX, double joystickY) {
    	
//    	left motors are negative
    	double gyroMod = orientationGyro.getAngle() % 360;
    	
    	if (gyroMod < -180){
    		gyroMod = gyroMod + 360;
    	}
    	if (gyroMod > 180){
    		gyroMod = gyroMod - 360;
    	}    	
    	
    	if (gyroMod != 0){
    		
    			if (joystickY != 0){
    				adjustGyroStraight(wheel1Spd * -joystickY, wheel2Spd * -joystickY, wheel3Spd * -joystickY, wheel4Spd * -joystickY, gyroMod);
    			}
    			if (joystickX != 0){
    				adjustGyroStrafe(wheel1Spd * -joystickY, wheel2Spd * -joystickY, wheel3Spd * -joystickY, wheel4Spd * -joystickY, gyroMod);
    			}
    			// System.out.println("Nothing doing.");
    		}
//			
    	else{
    		// System.out.println("Setting wheel speeds.");
    		this.wheel4.set(wheel4Spd);
    		this.wheel2.set(wheel2Spd);
    		this.wheel3.set(wheel3Spd);
    		this.wheel1.set(wheel1Spd);
    	}
    	
    	
    }  
    
    public double getInchesTraveled() {
    	this.wheel2EncoderValue();
    	double encoderValue = this.wheel2EncoderValue;
    	double numberOfInchesTraveled = encoderValue / 16.697 / 2.54;
    	
    	
    	
    	return numberOfInchesTraveled;
    }
    
    public void autonDriveForwardInches(double numberOfInches){
    	this.wheel1EncoderValue();
    	// Make a method to get an encoder value from a number of inches
    	double encoderValue = numberOfInches * 16.697 * 25.4;
    	
    	SmartDashboard.putString("DB/String 5", "Target encoder value: " + encoderValue);
		SmartDashboard.putString("DB/String 6", "Wheel 1 encoder value: " + wheel2EncoderValue);
    	
    	
    	if (wheel2EncoderValue < encoderValue){
    		this.wheel1.set(.35);
         	this.wheel2.set(-.35);
         	this.wheel3.set(.35);
         	this.wheel4.set(-.35); 
    	}
    	
    }
    
    public boolean stopAuton;
    
    public void stopAuton(){
    	this.stopAuton = true;
    }
    
   
    
    public void autonDriveBackInches(double numberOfInches){
    	this.wheel1EncoderValue();
    	double encoderValue = -numberOfInches * 16.697 * 25.4;
    	
		SmartDashboard.putString("DB/String 3", "Target encoder value: " + encoderValue);
		SmartDashboard.putString("DB/String 4", "Wheel 1 encoder value: " + wheel1EncoderValue);
    	
    	// Make a method for driving such that we can update motor power, etc.
    	if (wheel1EncoderValue > encoderValue){
            this.wheel1.set(-.35);
        	this.wheel2.set(.35);
        	this.wheel3.set(-.35);
        	this.wheel4.set(.35); 
    	}
    }
    
    // private void cooldownGyroAdjustment() {
    	
    // }
    
    private boolean adjustGyroDueToTimer() {
    	double time = this.gyroCooldownTimer.get();
    	
    	boolean adjust = false;
    	
    	if (time > 0 && time < 1.0) {
    		adjust = true;
    	}
    	else if (time > 1.0) {
    		gyroCooldownTimer.stop();
    	}
    	
    	return adjust;
    }
    
    private double getGyroAdjustment(double turn) {
        if (Math.abs(turn) > 0 || this.adjustGyroDueToTimer()) {
        	this.setGyroZero();
        	
        	if (Math.abs(turn) > 0) {
	        	this.gyroCooldownTimer.reset();
	        	this.gyroCooldownTimer.start();
        	}
        }
    	
    	double gyroMod = (Math.round(orientationGyro.getAngle()) - gyroZero) % 360;

    	if (gyroMod < -180){
    		gyroMod = gyroMod + 360;
    	}
    	if (gyroMod > 180){
    		gyroMod = gyroMod - 360;
    	}
    	
    	gyroMod = Math.round(gyroMod) % 360;
    	
    	double gyroDivisionConstant = 23;
    	
    	double gyroAdjustment = gyroMod / gyroDivisionConstant;
    	
    	// System.out.println("gyroDivisionConstant: " + gyroDivisionConstant + " gyroAdjustment: " + gyroAdjustment);
    	
    	return 0;
    	//return gyroAdjustment;
    }
    

    private double getAutonomousGyroAdjustment() {
    	double gyroMod = (Math.round(orientationGyro.getAngle()) - gyroZero) % 360;

    	if (gyroMod < -180){
    		gyroMod = gyroMod + 360;
    	}
    	if (gyroMod > 180){
    		gyroMod = gyroMod - 360;
    	}
    	
    	gyroMod = Math.round(gyroMod) % 360;
    	
    	double gyroDivisionConstant = 23;
    	
    	double secondGyroDivisionConstant = 1.25;
    	
    	double gyroAdjustment = gyroMod / gyroDivisionConstant;
    	
    	gyroAdjustment /= secondGyroDivisionConstant;
    	
    	// System.out.println("gyroDivisionConstant: " + gyroDivisionConstant + " gyroAdjustment: " + gyroAdjustment);
    	
    	
    	return gyroAdjustment;
    }
    
    public boolean hasReachedTargetAngle() {
    	boolean hasReachedTargetAngle = false;
    	
    	double gyroMod = (Math.round(orientationGyro.getAngle()) - gyroZero) % 360;

    	if (gyroMod < -180){
    		gyroMod = gyroMod + 360;
    	}
    	if (gyroMod > 180){
    		gyroMod = gyroMod - 360;
    	}
    	
    	gyroMod = Math.round(gyroMod) % 360;
    	
    	if (gyroMod < 1) {
    		hasReachedTargetAngle = true;
    	}
    	
    	return hasReachedTargetAngle;
    }

    
    public double wheel1EncoderValue(){
    	
    	this.wheel1EncoderValue = wheel1Encoder.get();
    	
    	return wheel1EncoderValue;
    }
    
    public double wheel2EncoderValue(){
    	
    	this.wheel2EncoderValue = wheel2Encoder.get();
    	
    	return wheel2EncoderValue;
    }
    
    public double wheel3EncoderValue(){
    	
    	this.wheel3EncoderValue = wheel3Encoder.get();
    	
    	return wheel3EncoderValue;
    }
    
 //   public double wheel4EncoderValue(){
    	
    //	this.wheel4EncoderValue = wheel4Encoder.get();
    	
//    	return wheel4EncoderValue;
  //  }
    
    public double setGyroZero(){
//    	return orientationGyro.getAngle();
    	
    	this.gyroZero = orientationGyro.getAngle();
    	
    	return gyroZero;
    }
    
    private void adjustGyroStraight(double wheel1Spd, double wheel2Spd, double wheel3Spd, double wheel4Spd, double gyroMod){
    	System.out.println("Adjust gyro straight.");
    	this.wheel4.set(wheel4Spd + (gyroMod / 100));
    	this.wheel2.set(wheel2Spd + (gyroMod / 100));
    	this.wheel3.set(wheel3Spd + (gyroMod / 100));
    	this.wheel1.set(wheel1Spd + (gyroMod / 100));

    }
    
    private void adjustGyroStrafe(double wheel1Spd, double wheel2Spd, double wheel3Spd, double wheel4Spd, double gyroMod){
    	System.out.println("Adjust gyro strafe.");
    	this.wheel4.set(-(wheel4Spd + (gyroMod / 100)));
    	this.wheel2.set(wheel2Spd + (gyroMod / 100));
    	this.wheel3.set(wheel3Spd + (gyroMod / 100));
    	this.wheel1.set(-(wheel1Spd + (gyroMod / 100)));
    	    	
    }
    
}
