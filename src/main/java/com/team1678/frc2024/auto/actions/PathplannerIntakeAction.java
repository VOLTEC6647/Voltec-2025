package com.team1678.frc2024.auto.actions;

import org.littletonrobotics.frc2025.subsystems.drive.Drive;

import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.path.PathConstraints;
import com.team4678.CommandSwerveDrivetrain;
import com.team6647.frc2025.subsystems.Superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class PathplannerIntakeAction implements Action {

	private CommandSwerveDrivetrain mDrive = null;

	private Timer autoTimer = new Timer();
	private double correctionDelay = 1.5;
	private Command pathPlannerCommand;

	public PathplannerIntakeAction() {
		
	}

	@Override
	public void start() {
		mDrive = CommandSwerveDrivetrain.getInstance();

		Pose2d endpose = Superstructure.getInstance().sourcePose.toLegacy();

		PathConstraints constraints = new PathConstraints(1.5, 1.5, 2 * Math.PI, 4 * Math.PI); // The constraints for

		pathPlannerCommand = AutoBuilder.pathfindToPose(endpose, constraints,0.0);
		pathPlannerCommand.schedule();
	}

	@Override
	public void update() {
		//System.out.println("Trajectory set");
	}

	@Override
	public boolean isFinished() {
		return pathPlannerCommand.isFinished();
	}


	@Override
	public void done() {
		mDrive.stopDrive();
	}
}
