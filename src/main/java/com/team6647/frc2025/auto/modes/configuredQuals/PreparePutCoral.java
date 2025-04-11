package com.team6647.frc2025.auto.modes.configuredQuals;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeEndedException;
import com.team1678.frc2024.auto.actions.CommandAction;
import com.team1678.frc2024.subsystems.CoralPivot;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.trajectory.Trajectory254;
import com.team254.lib.trajectory.timing.TimedState;
import com.team6647.frc2025.commands.GoCoralCommand;
import com.team6647.frc2025.subsystems.Superstructure;
import com.team6647.frc2025.subsystems.coral_roller.CoralRoller;

public class PreparePutCoral extends AutoModeBase {
	private Superstructure s = Superstructure.getInstance();

	Trajectory254<TimedState<Pose2dWithMotion>> putCoral;
	public static Trajectory254<TimedState<Pose2dWithMotion>> enterCoral;

	public PreparePutCoral() {
		enterCoral = null;
	}

	// spotless:off
	@Override
	public void routine() throws AutoModeEndedException {
		CoralRoller.getInstance().setState(CoralRoller.State.CONSTANT);
		CoralPivot.getInstance().setSetpointMotionMagic(s.currentLevel.coralAngle);
		runAction(new CommandAction(new GoCoralCommand()));
		//s.addRequestToQueue(s.prepareLevel(s.currentLevel));
		System.out.println("Finished auto!");
	}
	// spotless:on
}