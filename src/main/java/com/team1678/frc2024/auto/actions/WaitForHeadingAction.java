package com.team1678.frc2024.auto.actions;

import org.littletonrobotics.frc2025.RobotState6328;

import com.team1678.lib.swerve.ChassisSpeeds;
import com.team254.lib.util.TimeDelayedBoolean;

public class WaitForHeadingAction implements Action {
	RobotState6328 state;
	double target;
	double margin;
	TimeDelayedBoolean onTarget = new TimeDelayedBoolean();

	public WaitForHeadingAction(double target, double margin) {
		state = RobotState6328.getInstance();
		this.target = target;
		this.margin = margin;
	}

	@Override
	public boolean isFinished() {
		double heading = state.getHeading().getDegrees();
		System.out.println(heading);
		return onTarget.update(Math.abs(target - heading) <= margin, 1.0);
	}

	@Override
	public void start() {}

	@Override
	public void update() {
	}

	@Override
	public void done() {}
}
