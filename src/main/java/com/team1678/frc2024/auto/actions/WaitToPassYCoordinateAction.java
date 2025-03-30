package com.team1678.frc2024.auto.actions;

import org.littletonrobotics.frc2025.RobotState6328;


public class WaitToPassYCoordinateAction implements Action {
	double startingXCoordinate;
	double targetXCoordinate;
	RobotState6328 state;

	public WaitToPassYCoordinateAction(double y) {
		targetXCoordinate = y;
		state = RobotState6328.getInstance();
	}

	@Override
	public boolean isFinished() {
		return Math.signum(startingXCoordinate - targetXCoordinate)
				!= Math.signum(state.getOdometryPose().getTranslation().getY() - targetXCoordinate);
	}

	@Override
	public void start() {
		startingXCoordinate = state.getOdometryPose().getTranslation().getY();
	}

	@Override
	public void update() {}

	@Override
	public void done() {}
}
