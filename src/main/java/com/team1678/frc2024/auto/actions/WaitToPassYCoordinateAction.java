package com.team1678.frc2024.auto.actions;

import org.littletonrobotics.frc2025.RobotState;

import com.team4678.CommandSwerveDrivetrain;


public class WaitToPassYCoordinateAction implements Action {
	double startingXCoordinate;
	double targetXCoordinate;
	CommandSwerveDrivetrain drive;

	public WaitToPassYCoordinateAction(double y) {
		targetXCoordinate = y;
		drive = CommandSwerveDrivetrain.getInstance();
	}

	@Override
	public boolean isFinished() {
		return Math.signum(startingXCoordinate - targetXCoordinate)
				!= Math.signum(drive.getState().Pose.getTranslation().getY() - targetXCoordinate);
	}

	@Override
	public void start() {
		startingXCoordinate = drive.getState().Pose.getTranslation().getY();
	}

	@Override
	public void update() {}

	@Override
	public void done() {}
}
