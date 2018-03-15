package org.usfirst.frc.team839.robot.commands.autonomous;

import org.usfirst.frc.team839.robot.commands.DrivePID;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;

public class ChooseSwitch extends CommandGroup
{
	{
	  //addSequential (new DrivePID [Driving + Turning] / TurnToAngle [Turning Only] (#.#f [Inches] , #.#f [Target Angle]));
		addSequential (new DrivePID(12.0f, 0.0f));		//13 Feet
		addSequential (new WaitCommand(0.5));
		addSequential (new TurnToAngle(45.0f,"R"));		//13 Feet
		addSequential (new WaitCommand(0.5));
		addSequential (new DrivePID(76.368f, 45.0f));		//13 Feet
		addSequential (new WaitCommand(0.5));
		addSequential (new TurnToAngle(-45.0f,"R"));		//13 Feet
		addSequential (new WaitCommand(0.5));
		addSequential (new DrivePID(46.0f, 0.0f));		//13 Feet
	  //addSequential (new TurnToAngle(90.0f));
	}
}