package com.team1678.lib.requests;

import com.team1678.frc2024.controlboard.ControlBoard;
import com.team6647.frc2025.subsystems.Superstructure;
import com.team6647.frc2025.subsystems.coral_roller.CoralRoller;

public class DepositCoralRequest {
	public static SequentialRequest get(){
		Superstructure s = Superstructure.getInstance();
		s.placing_coral = true;
		if(Superstructure.getInstance().level<4){
			return new SequentialRequest(
				s.prepareLevel(s.currentLevel),
				new LambdaRequest(()->{CoralRoller.getInstance().setState(CoralRoller.getInstance().OUTAKING);}),
				new WaitRequest(0.5),
				new LambdaRequest(()->{CoralRoller.getInstance().setState(CoralRoller.State.IDLE);}),
				s.softHome()
			);
		}else{
			return new SequentialRequest(
				new WaitRequest(0.3),
				s.prepareLevel(s.currentLevel),
				new LambdaRequest(()->{CoralRoller.getInstance().setState(CoralRoller.getInstance().OUTAKING);}),
				new WaitRequest(0.5),
				new LambdaRequest(()->{CoralRoller.getInstance().setState(CoralRoller.State.INTAKING);}),
				new WaitRequest(0.5),
				new LambdaRequest(()->{CoralRoller.getInstance().setState(CoralRoller.State.IDLE);}),
				s.softHome()
				);
		}
	}
}
