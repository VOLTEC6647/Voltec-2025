package com.team1678.frc2024.auto.actions;

import org.littletonrobotics.frc2025.RobotState;

import com.team254.lib.util.TimeDelayedBoolean;
import com.team4678.CommandSwerveDrivetrain;

public class WaitForHeadingAction implements Action {
	CommandSwerveDrivetrain drive;
	double target;
	double margin;
	TimeDelayedBoolean onTarget = new TimeDelayedBoolean();

	public WaitForHeadingAction(double target, double margin) {
		drive = CommandSwerveDrivetrain.getInstance();
		this.target = target;
		this.margin = margin;
	}

	@Override
	public boolean isFinished() {
		double heading = drive.getState().Pose.getRotation().getDegrees();
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
