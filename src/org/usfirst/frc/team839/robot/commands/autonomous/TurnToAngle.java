package org.usfirst.frc.team839.robot.commands.autonomous;

import org.usfirst.frc.team839.robot.Robot;
import org.usfirst.frc.team839.robot.RobotMap;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Command;

public class TurnToAngle extends Command implements PIDOutput {
	private double rotateToAngleRate = 0;
	PIDController turnController;
	AHRS ahrs;
	private float angle = 0;
	private String side;
	static final double kP = 0.033;	//0.010			//Slows robot down as it reaches destination (90 degree turn)
	static final double kI = 0.00;//2;	//0.003			//Detects the difference between current location and target
	static final double kD = 0.00;//2;	//#.###			//Measures rate of change (Change in Time (y) / Change in Speed)
	static final double kF = 0.000;	//#.###			//Product of P and I (?)

	static final double kToleranceDegrees = 2.0f;

	public TurnToAngle(float angle) {
		this.angle  = angle;
		this.ahrs = new AHRS(SPI.Port.kMXP);
		this.turnController = new PIDController(kP, kI, kD, kF, ahrs, this,0.02);
		this.turnController.setInputRange(-180.0f,  180.0f);
		this.turnController.setOutputRange(-0.1, 0.1);
		this.turnController.setAbsoluteTolerance(kToleranceDegrees);
		this.turnController.setContinuous(true);
		this.turnController.setSetpoint(this.angle);

		this.setTimeout(5);
	}
	public TurnToAngle(float angle, String side) {
		this.side = side;
		this.angle  = angle;
		this.ahrs = new AHRS(SPI.Port.kMXP);
		this.turnController = new PIDController(kP, kI, kD, kF, ahrs, this,0.02);
		this.turnController.setInputRange(-180.0f,  180.0f);
		this.turnController.setOutputRange(-0.5, 0.5);
		this.turnController.setAbsoluteTolerance(kToleranceDegrees);
		this.turnController.setContinuous(true);
		this.turnController.setSetpoint(this.angle);

		this.setTimeout(5);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		this.angle  = Robot.fieldLayout.isOurSwitch(side)?angle:-angle;
		this.ahrs.reset();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		turnController.enable();
        double currentRotationRate = this.rotateToAngleRate;
        try {
        	System.out.println(currentRotationRate);
         	System.out.println(this.turnController.getError());
         	System.out.println(ahrs.getYaw());
         	Robot.drivetrain.setDriveSpeeds(0, 0, -currentRotationRate, 0);
//        	RobotMap.robotDrive.driveCartesian(0, 0,-currentRotationRate, 0);
        } catch( RuntimeException ex ) {
            DriverStation.reportError("Error communicating with drive system:  " + ex.getMessage(), true);
        }
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return this.isTimedOut();
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		System.out.println(ahrs.getYaw());
        RobotMap.robotDrive.driveCartesian(0, 0,0);
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
}