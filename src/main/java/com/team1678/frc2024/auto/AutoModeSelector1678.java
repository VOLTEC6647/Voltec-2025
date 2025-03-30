package com.team1678.frc2024.auto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;

public class AutoModeSelector1678 {
	public enum DesiredMode {
		DO_NOTHING,
		TEST_PATH_AUTO,
		SIX_NOTE_MODE,
		ALTERNATE_SIX_NOTE_MODE,
		SEMI_FAST_SIX_NOTE_MODE,
		ONE_THREE_SEMI_FAST_SIX_NOTE_MODE,
		SHOOT_AND_DRIVE,
		THREE_NOTE_MODE_54,
		THREE_NOTE_MODE_53,
		THREE_NOTE_MODE_45,
		THREE_NOTE_MODE_43,
		THREE_NOTE_MODE_35,
		THREE_NOTE_MODE_34,
		RIGHT_FIVE_MODE_4,
		RIGHT_FIVE_MODE_5,
		ADAPTIVE_SIX_MODE
	}

	public enum TargetNote {
		N1,
		N2,
		N3,
		N4,
		N5
	}

	public enum TargetSpike {
		LEFT,
		CENTER,
		RIGHT
	}

	private DesiredMode mCachedDesiredMode = DesiredMode.DO_NOTHING;
	private TargetNote mCachedFirstNote = TargetNote.N1;
	private TargetNote mCachedSecondNote = TargetNote.N2;
	private TargetSpike mCachedSpike = TargetSpike.LEFT;

	private Optional<AutoModeBase> mAutoMode = Optional.empty();

	private static SendableChooser<DesiredMode> mModeChooser = new SendableChooser<>();
	private static SendableChooser<TargetNote> mFirstNoteChooser = new SendableChooser<>();
	private static SendableChooser<TargetNote> mSecondNoteChooser = new SendableChooser<>();
	private static SendableChooser<TargetSpike> mSpikeChooser = new SendableChooser<>();

	public AutoModeSelector1678() {
		mModeChooser.addOption("Do Nothing", DesiredMode.DO_NOTHING);
		mModeChooser.setDefaultOption("Semi Fast Six Note", DesiredMode.SEMI_FAST_SIX_NOTE_MODE);
		mModeChooser.setDefaultOption("13 Semi Fast Six Note", DesiredMode.ONE_THREE_SEMI_FAST_SIX_NOTE_MODE);
		mModeChooser.addOption("Three Note 4/5 + Limelight", DesiredMode.THREE_NOTE_MODE_45);
		mModeChooser.addOption("TESTPATH!!!", DesiredMode.TEST_PATH_AUTO);
		mModeChooser.addOption("SHOOT AND DRIVE", DesiredMode.SHOOT_AND_DRIVE);
		mModeChooser.addOption("Six Note", DesiredMode.SIX_NOTE_MODE);
		mModeChooser.addOption("Alternate Six Note", DesiredMode.ALTERNATE_SIX_NOTE_MODE);
		mModeChooser.addOption("Three Note 5/4", DesiredMode.THREE_NOTE_MODE_54);
		mModeChooser.addOption("Three Note 5/3", DesiredMode.THREE_NOTE_MODE_53);
		mModeChooser.addOption("Three Note 4/3", DesiredMode.THREE_NOTE_MODE_43);
		mModeChooser.addOption("Three Note 3/5 + Limelight", DesiredMode.THREE_NOTE_MODE_35);
		mModeChooser.addOption("Three Note 3/4", DesiredMode.THREE_NOTE_MODE_34);
		mModeChooser.addOption("Adaptive Six", DesiredMode.ADAPTIVE_SIX_MODE);
		mModeChooser.addOption("N4 Source Five Mode", DesiredMode.RIGHT_FIVE_MODE_4);
		mModeChooser.addOption("N5 Source Five Mode", DesiredMode.RIGHT_FIVE_MODE_5);

		SmartDashboard.putData("Auto Mode", mModeChooser);

		
	}

	public void updateModeCreator(boolean force_regen) {
		DesiredMode desiredMode = mModeChooser.getSelected();
		TargetNote firstNote = mFirstNoteChooser.getSelected();
		TargetNote secondNote = mSecondNoteChooser.getSelected();
		TargetSpike desiredSpike = mSpikeChooser.getSelected();

		if (desiredMode == null) {
			desiredMode = DesiredMode.DO_NOTHING;
		}
		if (mCachedDesiredMode != desiredMode
				|| mCachedFirstNote != firstNote
				|| mCachedSecondNote != secondNote
				|| mCachedSpike != desiredSpike
				|| force_regen) {
			System.out.println("Auto selection changed, updating creator: desiredMode-> " + desiredMode.name() + "//"
					+ firstNote.name() + "//" + secondNote.name() + "//" + desiredMode.name() + " Spike");
		}
		mCachedDesiredMode = desiredMode;
		mCachedFirstNote = firstNote;
		mCachedSecondNote = secondNote;
		mCachedSpike = desiredSpike;
	}

	public static SendableChooser<DesiredMode> getModeChooser() {
		return mModeChooser;
	}

	public DesiredMode getDesiredAutomode() {
		return mCachedDesiredMode;
	}

	public void reset() {
		mAutoMode = Optional.empty();
		mCachedDesiredMode = null;
	}

	public void outputToSmartDashboard() {
		SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
	}

	public Optional<AutoModeBase> getAutoMode() {
		if (!mAutoMode.isPresent()) {
			return Optional.empty();
		}
		return mAutoMode;
	}

	public boolean isDriveByCamera() {
		return mCachedDesiredMode == DesiredMode.DO_NOTHING;
	}
}
