package com.team1678.frc2024.auto.actions;

import org.littletonrobotics.frc2025.RobotState;

import com.team6647.frc2025.FieldLayout;
import com.team6647.frc2025.Robot;

public class WaitToPassXCoordinateAction implements Action {
	double startingXCoordinate;
	double targetXCoordinate;
	RobotState state;

	public WaitToPassXCoordinateAction(double x) {
		if (Robot.is_red_alliance) {
			targetXCoordinate = FieldLayout.kFieldLength - x;
		} else {
			targetXCoordinate = x;
		}
	}

	@Override
	public boolean isFinished() {
		return Math.signum(startingXCoordinate - targetXCoordinate)
				!= Math.signum(state.getEstimatedPose().getTranslation().getX() - targetXCoordinate);
	}

	@Override
	public void start() {
		startingXCoordinate = state.getEstimatedPose().getTranslation().getX();
	}

	@Override
	public void update() {}

	@Override
	public void done() {}
}
