package com.team1678.lib.requests;

import com.team6647.frc2025.subsystems.Superstructure;
import com.team6647.frc2025.subsystems.coral_roller.CoralRoller;

public class IntakeCoralRequest {
	public static SequentialRequest get(){
		CoralRoller mCoralRoller = CoralRoller.getInstance();
		return
			new SequentialRequest(
				new LambdaRequest(
					()->{}
				),
				new LambdaRequest(
					()->{mCoralRoller.setState(CoralRoller.State.INTAKING);}
				),
				new WaitRequest(0.2),
				new WaitForPrereqRequest(()->mCoralRoller.getBeamBreak()),
				new LambdaRequest(
					()->{mCoralRoller.setState(CoralRoller.State.IDLE);}
				)
			);
	}
}
