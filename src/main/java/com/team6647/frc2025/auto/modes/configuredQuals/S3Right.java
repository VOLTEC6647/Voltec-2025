package com.team6647.frc2025.auto.modes.configuredQuals;

import com.team1678.frc2024.auto.AutoModeBase;
import com.team1678.frc2024.auto.AutoModeEndedException;
import com.team1678.frc2024.auto.actions.ChoreoTrajectoryAction;
import com.team1678.frc2024.auto.actions.LambdaAction;
import com.team1678.frc2024.auto.actions.WaitAction;
import com.team1678.frc2024.subsystems.CoralPivot;
import com.team1678.frc2024.subsystems.Drive;
import com.team6647.frc2025.subsystems.Elevator;
import com.team6647.frc2025.subsystems.Superstructure;
import com.team6647.frc2025.subsystems.Superstructure.Levels;
import com.team6647.frc2025.subsystems.coral_roller.CoralRoller;

import edu.wpi.first.wpilibj2.command.WaitCommand;

public class S3Right extends AutoModeBase {
	private Drive d = Drive.getInstance();
	private Superstructure s = Superstructure.getInstance();
	public S3Right() {
		
	}

	// spotless:off
	@Override
	protected void routine() throws AutoModeEndedException {
		runAction(new WaitAction(1));
		s.request(s.prepareLevel(Levels.LEVEL2));
		runAction(new ChoreoTrajectoryAction("S3Right1",true));
		runAction(new WaitAction(1.5));
		CoralRoller.getInstance().setState(CoralRoller.getInstance().OUTAKING);
		runAction(new WaitAction(1));
		//Elevator.getInstance().setWantHome(true);

		runAction(new ChoreoTrajectoryAction("S3Right2"));
		s.request(s.prepareLevel(Levels.ALGAEING2));
		runAction(new WaitAction(2));
		
		CoralRoller.getInstance().setState(CoralRoller.State.IDLE);
		Elevator.getInstance().setSetpointMotionMagic(Elevator.kL2Height);
		CoralPivot.getInstance().setSetpointMotionMagic(CoralPivot.kIntakingAngle);

		runAction(new ChoreoTrajectoryAction("S3Right3"));




		System.out.println("Finished auto!");
	}
	// spotless:on
}