package org.usfirst.frc.team839.robot.commands;

import org.usfirst.frc.team839.robot.Robot;
import org.usfirst.frc.team839.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Command;

public class DrivePID extends Command implements PIDOutput {
	private double rotateToAngleRate = 0;
	PIDController turnController;
	PIDController distanceController;
	DistancePidListener distanceListener = null;
	ADXRS450_Gyro gyro;

	private double distance;
	private double drivetoDistanceRate;			//drivetoDistanceRate is not used
	private float angle;

	static final double kP = 0.033;				//0.010		//Slows robot down as it reaches destination (90 degree turn)
	static final double kI = 0;	//0.002;				//0.003		//Detects the difference between current location and target
	static final double kD = 0;	//0.002;				//#.###		//Measures rate of change (Change in Time (y) / Change in Speed)
	static final double kF = 0.000;				//#.###		//Product of P and I (?)
	static final double ticksPerInch = 300.05;		//217.2995489686

	static final double drive_kP = 0.0004;		//#.###		//Slows robot down as it reaches destination 
	static final double drive_kI = 0.0000000;	//#.###		//Detects the difference between current location and target
	static final double drive_kD = 0.00;		//#.###		//Measures rate of change (Change in Time (y) / Change in Speed)
	static final double drive_kF = 0.000;		//#.###		//Product of P and I (?)

	static final double kToleranceDegrees = 2.0f;
	static double itterations = 0;
	public DrivePID(float distance, float angle) {
		this.angle = angle;
		this.distance = ticksPerInch * distance;
		this.gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS1);
	}
	public DrivePID(float distance, float angle, double timeOut) {
		this.setTimeout(timeOut);
		this.angle = angle;
		this.distance = ticksPerInch * distance;
		this.gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS1);	//new AHRS(SPI.Port.kMXP);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		
		RobotMap.frontRightMotor.configOpenloopRamp(.5, 0);
		RobotMap.frontLeftMotor.configOpenloopRamp(.5, 0);
		RobotMap.backRightMotor.configOpenloopRamp(.5, 0);
		RobotMap.backLeftMotor.configOpenloopRamp(.5, 0);

		distanceListener = new DistancePidListener(RobotMap.frontRightMotor);
		this.gyro.reset();
		distanceListener.resetEncoder();
		this.turnController = new PIDController(kP, kI, kD, kF, gyro, this,0.02);
		this.turnController.setInputRange(-180.0f,  180.0f);
		this.turnController.setOutputRange(-0.8, 0.8);
		this.turnController.setAbsoluteTolerance(kToleranceDegrees);
		this.turnController.setContinuous(true);
		this.turnController.setSetpoint(this.angle);

		this.distanceController = new PIDController(drive_kP, drive_kI, drive_kD, drive_kF, distanceListener, distanceListener,0.02);
		this.distanceController.setInputRange(-50000.0f,  50000.0f);
		this.distanceController.setOutputRange(-0.1, 0.3);
		this.distanceController.setAbsoluteTolerance(3000);
		this.distanceController.setContinuous(false);
		this.distanceController.setSetpoint(this.distance);
		this.setTimeout(10);

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
protected void execute() {
		turnController.enable();
		distanceController.enable();
        double currentRotationRate = this.rotateToAngleRate;
        double currentDrivetoDistanceRate = distanceListener.drivetoDistanceRate;
        try {
        	if(itterations%10 == 0)
        	{
        		System.out.println("Drive rate: " + currentDrivetoDistanceRate);
        		System.out.println("ERROR: " + this.distanceController.getError());
        		System.out.println("Current position: " + distanceListener.pidGet());
        		System.out.println("Turn rate: " + currentRotationRate);
             	System.out.println("Angle ERROR: " + this.turnController.getError());
             	System.out.println("Current angle: " + gyro.getAngle());
        	}
         	Robot.drivetrain.setDriveSpeeds(0, -currentDrivetoDistanceRate, -currentRotationRate, 0);
        	//RobotMap.robotDrive.driveCartesian(0, -currentDrivetoDistanceRate,-currentRotationRate, 0);
         	itterations++;
        } catch( RuntimeException ex ) {
            DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
        }
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return this.isTimedOut() || this.distanceController.getError() < 1000;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		System.out.println(gyro.getAngle());
        RobotMap.robotDrive.driveCartesian(0, 0,0);
		RobotMap.frontRightMotor.configOpenloopRamp(Robot.defaultRampRate, 0);
		RobotMap.frontLeftMotor.configOpenloopRamp(Robot.defaultRampRate, 0);
		RobotMap.backRightMotor.configOpenloopRamp(Robot.defaultRampRate, 0);
		RobotMap.backLeftMotor.configOpenloopRamp(Robot.defaultRampRate, 0);

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
	}

	@Override
	public void pidWrite(double output) {
		
		this.rotateToAngleRate = output;

	}
	private class DistancePidListener  implements PIDOutput, PIDSource{
		public DistancePidListener(BaseMotorController motorControler) {
			this.motorControler = motorControler;
			motorControler.setSelectedSensorPosition(0, 0, 0);
		}
		
		public void resetEncoder(){
			motorControler.setSelectedSensorPosition(0, 0, 0);
		}
		
		public double drivetoDistanceRate = 0.0f;
		private BaseMotorController motorControler = null;
		private PIDSourceType pidsourceType;
		@Override
		public void pidWrite(double output) {
			this.drivetoDistanceRate = output;
			
		}
		@Override
		public void setPIDSourceType(PIDSourceType pidSource) {
			this.pidsourceType = pidSource;
			
		}
		@Override
		public PIDSourceType getPIDSourceType() {
			return PIDSourceType.kDisplacement;
		}
		@Override
		public double pidGet() {
			return motorControler.getSelectedSensorPosition(0);
		}
		
	}
}