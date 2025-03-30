package com.team1678.frc2024.auto.actions;

import org.littletonrobotics.frc2025.RobotState;


public class WaitToPassYCoordinateAction implements Action {
	double startingXCoordinate;
	double targetXCoordinate;
	RobotState state;

	public WaitToPassYCoordinateAction(double y) {
		targetXCoordinate = y;
		state = RobotState.getInstance();
	}

	@Override
	public boolean isFinished() {
		return Math.signum(startingXCoordinate - targetXCoordinate)
				!= Math.signum(state.getEstimatedPose().getTranslation().getY() - targetXCoordinate);
	}

	@Override
	public void start() {
		startingXCoordinate = state.getEstimatedPose().getTranslation().getY();
	}

	@Override
	public void update() {}

	@Override
	public void done() {}
}
