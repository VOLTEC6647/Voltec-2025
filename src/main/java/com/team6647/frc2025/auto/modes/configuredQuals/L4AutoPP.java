package com.team6647.frc2025.auto.modes.configuredQuals;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeEndedException;
import com.team1678.frc2024.auto.actions.PathplannerTrajectoryAction;
import com.team1678.lib.requests.SequentialRequest;
import com.team254.lib.geometry.Pose2dWithMotion;
import com.team254.lib.trajectory.Trajectory254;
import com.team254.lib.trajectory.timing.TimedState;
import com.team6647.frc2025.subsystems.Superstructure;
import com.team6647.frc2025.subsystems.leds.LEDSubsystem;

public class L4AutoPP extends AutoModeBase {
	private Superstructure s = Superstructure.getInstance();

	Trajectory254<TimedState<Pose2dWithMotion>> putCoral;
	public static Trajectory254<TimedState<Pose2dWithMotion>> enterCoral;

	public L4AutoPP() {
		
	}

	// spotless:off
	@Override
	protected void routine() throws AutoModeEndedException {
		LEDSubsystem.getInstance().solidYellow();
		runAction(new PathplannerTrajectoryAction("L4"));
		s.currentLevel = Superstructure.Levels.LEVEL3;
		LEDSubsystem.getInstance().solidBlue();
		s.request(
		new SequentialRequest(
			s.prepareLevel(s.currentLevel)
		));
		System.out.println("Finished auto!");
	}
	// spotless:on
}