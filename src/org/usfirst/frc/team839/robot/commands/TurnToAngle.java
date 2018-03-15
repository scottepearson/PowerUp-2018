package org.usfirst.frc.team839.robot.commands;

import org.usfirst.frc.team839.robot.RobotMap;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TurnToAngle extends Command implements PIDOutput {
	private double rotateToAngleRate = 0;
	PIDController turnController;
	ADXRS450_Gyro gyro;
	private float angle = 0;
	static final double kP = 0.03;
	static final double kI = 0.00;
	static final double kD = 0.00;
	static final double kF = 0.00;

	static final double kToleranceDegrees = 2.0f;

	public TurnToAngle(float angle) {
		this.angle  = angle;
		this.gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS1);
		this.turnController = new PIDController(kP, kI, kD, kF, gyro, this);
		this.turnController.setInputRange(-180.0f,  180.0f);
		this.turnController.setOutputRange(-1.0, 1.0);
		this.turnController.setAbsoluteTolerance(kToleranceDegrees);
		this.turnController.setContinuous(true);
		this.setTimeout(20);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		turnController.enable();
        double currentRotationRate = this.rotateToAngleRate;
        try {
            /* Use the joystick X axis for lateral movement,          */
            /* Y axis for forward movement, and the current           */
            /* calculated rotation rate (or joystick Z axis),         */
            /* depending upon whether "rotate to angle" is active.    */
            RobotMap.robotDrive.driveCartesian(0, 0,currentRotationRate, gyro.getAngle());
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
		System.out.println(gyro.getAngle());
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
