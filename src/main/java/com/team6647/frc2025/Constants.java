package com.team6647.frc2025;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystem;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystem.TalonFXConstants;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystemBorePID.ServoMotorSubsystemConstantsBorePID;
import com.team1678.frc2024.subsystems.servo.ServoMotorSubsystemWithCancoder.AbsoluteEncoderConstants;
import com.team1678.lib.Conversions;
import com.team254.lib.drivers.CanDeviceId;
import com.team254.lib.geometry.Rotation2d;
import com.team254.lib.geometry.Transform2d;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Constants {
    public static class OperatorConstants {

    }
    public static class DriveConstants {
        public static String swerveCANBus = "6647_Swerve";
		public static String mechanismsCANBus = "6647_CANivore";
        //public static String swerveCANBus = "rio";


    }

    public static class ElevatorConstants {
        public static final ServoMotorSubsystemConstants kElevatorServoConstants = new ServoMotorSubsystemConstants();

		static {
			kElevatorServoConstants.kName = "Elevator";

			kElevatorServoConstants.kMainConstants.counterClockwisePositive = true;
			kElevatorServoConstants.kMainConstants.invert_sensor_phase = false;

			kElevatorServoConstants.kFollowerConstants = new TalonFXConstants[1];
			kElevatorServoConstants.kFollowerConstants[0] = new TalonFXConstants();
			kElevatorServoConstants.kMainConstants.id = Ports.ELEVATOR_MAIN;
			kElevatorServoConstants.kFollowerConstants[0].id = Ports.ELEVATOR_FOLLOWER;
			kElevatorServoConstants.kFollowerConstants[0].counterClockwisePositive = false;

			kElevatorServoConstants.kHomePosition = 0.0; // meters

			//kElevatorServoConstants.kMaxUnitsLimit = 0.46;
			//kElevatorServoConstants.kMinUnitsLimit = 0.0;

			//14/20
			//81/20
			kElevatorServoConstants.kRotationsPerUnitDistance = (9.0) / (Conversions.inchesToMeters(1.432) * Math.PI);;

			kElevatorServoConstants.kKp = 10.0; // Raw output / raw error
			kElevatorServoConstants.kKi = 0.0; // Raw output / sum of raw error
			kElevatorServoConstants.kKd = 0.0; // Raw output / (err - prevErr)
			kElevatorServoConstants.kKa = 0.0; // Raw output / accel in (rots/s) / s
			kElevatorServoConstants.kKg = 0;
			kElevatorServoConstants.kDeadband = 0; // rots

			kElevatorServoConstants.kCruiseVelocity = 2;//4  //12.0; // m / s
			kElevatorServoConstants.kAcceleration = 1.7;//2 // m / s^2
			kElevatorServoConstants.kRampRate = 0.0; // s
			kElevatorServoConstants.kMaxForwardOutput = 12.0;
			kElevatorServoConstants.kMaxReverseOutput = -12.0;

			kElevatorServoConstants.kEnableSupplyCurrentLimit = true;
			kElevatorServoConstants.kSupplyCurrentLimit = 40; // amps
			kElevatorServoConstants.kSupplyCurrentThreshold = 0; // amps
			kElevatorServoConstants.kSupplyCurrentTimeout = 0.02; // seconds

			kElevatorServoConstants.kNeutralMode = NeutralModeValue.Brake;

			kElevatorServoConstants.kTolerance = 1;
			
		}
		

		public static double kHomingZone = 0.1; // meters
		public static double kHomingTimeout = 0.1; // seconds
		public static double kHomingVelocityWindow = 0.1; // "units" / second
		public static double kHomingOutput = -2.0; // volts
		public static double kMaxHomingOutput = -3.5; // volts

		//public static double kHomingZone = 0; // meters
		//public static double kHomingTimeout = 0.5; // seconds
		//public static double kHomingVelocityWindow = 0.1; // "units" / second
		//public static double kHomingOutput = -2.0; // volts
    }

	public static final class AlgaeRollerConstants {
		public static TalonFXConfiguration RollerFXConfig() {
			TalonFXConfiguration config = new TalonFXConfiguration();

			config.CurrentLimits.SupplyCurrentLimitEnable = true;
			config.CurrentLimits.SupplyCurrentLimit = 40.0;

			config.CurrentLimits.StatorCurrentLimitEnable = true;
			config.CurrentLimits.StatorCurrentLimit = 80.0;

			config.Voltage.PeakForwardVoltage = 4.0;
			config.Voltage.PeakReverseVoltage = -4.0;

			config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
			
			return config;
		}
	}

	public static final class CoralRollerConstants {
		public static double sensorThreshold = 130;
		public static int sensorId;
	}

	public static final class AlgaeHolderConstants {
		public static SparkMaxConfig SparkMaxConfig() {
			SparkMaxConfig config = new SparkMaxConfig();

			config.smartCurrentLimit(30);

			//config.Voltage.PeakForwardVoltage = 12.0;
			//config.Voltage.PeakReverseVoltage = -12.0;

			config.idleMode(IdleMode.kBrake);
			return config;
		}
		public double movementThreshold = 5.0;
	}

	public static final class CoralPivotConstants {

		public static final double kRotorRotationsPerOutputRotation = 1.0; // Rotor to unit distance

		public static final ServoMotorSubsystemConstants kHoodServoConstants = new ServoMotorSubsystemConstants();
		public static final AbsoluteEncoderConstants kHoodEncoderConstants = new AbsoluteEncoderConstants();

		public static final double kHomingVoltage = -3.5;
		public static final double kHomingCurrentThreshold = 15.0;
		public static final double kMinHomingTime = 0.4;
		public static final double kMaxHomingTime = 10.0;

        public static double kHomePosition = 0;

		static {
			kHoodServoConstants.kName = "CoralPivot";

			kHoodServoConstants.kMainConstants.id = Ports.CORAL_PIVOT;
			kHoodServoConstants.kMainConstants.counterClockwisePositive = false;

			kHoodServoConstants.kHomePosition = 0; // Degrees
			kHoodServoConstants.kTolerance = 1; // Degrees
			kHoodServoConstants.kRotationsPerUnitDistance = (1.0 / 360.0 * 75) /* (7.16 / 1.0)*/; // Cancoder to unit distance
			kHoodServoConstants.kKp = 3.0;
			kHoodServoConstants.kKi = 0;
			kHoodServoConstants.kKd = 0.0;
			kHoodServoConstants.kKg = 0;
			kHoodServoConstants.kKs = 0.0;
			kHoodServoConstants.kDeadband = 0; // Ticks

			//kHoodServoConstants.kMinUnitsLimit = 15.0;
			//kHoodServoConstants.kMaxUnitsLimit = 62.0;

			kHoodServoConstants.kCruiseVelocity = 400.0; // degrees / s
			kHoodServoConstants.kAcceleration = 400.0; // degrees / s^2

			kHoodServoConstants.kEnableSupplyCurrentLimit = true;
			kHoodServoConstants.kSupplyCurrentLimit = 80;
			kHoodServoConstants.kSupplyCurrentThreshold = 0;

			kHoodServoConstants.kEnableStatorCurrentLimit = true;
			kHoodServoConstants.kStatorCurrentLimit = 40;

			kHoodServoConstants.kMaxForwardOutput = 12.0;
			kHoodServoConstants.kMaxReverseOutput = -12.0;//12

			kHoodServoConstants.kRampRate = 0.0;

			kHoodServoConstants.kNeutralMode = NeutralModeValue.Coast;

			kHoodEncoderConstants.encoder_type = FeedbackSensorSourceValue.RotorSensor; //FusedCANcoder
			kHoodEncoderConstants.remote_encoder_port = Ports.CORAL_CANCODER;
			kHoodEncoderConstants.rotor_rotations_per_output = 1.0;
			kHoodEncoderConstants.remote_encoder_offset = 0;
		}
	}

	public static final class ClimberConstants {

		public static final double kRotorRotationsPerOutputRotation = 1.0; // Rotor to unit distance

		public static final ServoMotorSubsystemConstants kHoodServoConstants = new ServoMotorSubsystemConstants();
		public static final AbsoluteEncoderConstants kHoodEncoderConstants = new AbsoluteEncoderConstants();

		public static final double kHomingVoltage = -3.5;
		public static final double kHomingCurrentThreshold = 15.0;
		public static final double kMinHomingTime = 0.4;
		public static final double kMaxHomingTime = 10.0;

        public static double kHomePosition = 0;

		static {
			kHoodServoConstants.kName = "Climber";

			kHoodServoConstants.kMainConstants.id = Ports.CLIMBER;
			kHoodServoConstants.kMainConstants.counterClockwisePositive = false;

			kHoodServoConstants.kHomePosition = 0; // Degrees
			kHoodServoConstants.kTolerance = 1; // Degrees
			kHoodServoConstants.kRotationsPerUnitDistance = (1.0 / 360.0 * 9*5*5*1.6) /* (7.16 / 1.0)*/; // Cancoder to unit distance
			kHoodServoConstants.kKp = 2.0;
			kHoodServoConstants.kKi = 0;
			kHoodServoConstants.kKd = 0.0;
			kHoodServoConstants.kKg = 0;
			kHoodServoConstants.kKs = 0.0;
			kHoodServoConstants.kDeadband = 0; // Ticks

			//kHoodServoConstants.kMinUnitsLimit = 15.0;
			//kHoodServoConstants.kMaxUnitsLimit = 62.0;

			kHoodServoConstants.kCruiseVelocity = 300.0; // degrees / s
			kHoodServoConstants.kAcceleration = 50.0; // degrees / s^2

			kHoodServoConstants.kEnableSupplyCurrentLimit = false ;
			kHoodServoConstants.kSupplyCurrentLimit = 80;
			kHoodServoConstants.kSupplyCurrentThreshold = 0;

			kHoodServoConstants.kEnableStatorCurrentLimit = false;
			kHoodServoConstants.kStatorCurrentLimit = 120;

			kHoodServoConstants.kMaxForwardOutput = 12.0;
			kHoodServoConstants.kMaxReverseOutput = -12.0;//12

			kHoodServoConstants.kRampRate = 0.0;

			kHoodServoConstants.kNeutralMode = NeutralModeValue.Coast;

			kHoodEncoderConstants.encoder_type = FeedbackSensorSourceValue.RotorSensor; //FusedCANcoder
			kHoodEncoderConstants.remote_encoder_port = Ports.CORAL_CANCODER;
			kHoodEncoderConstants.rotor_rotations_per_output = 1.0;
			kHoodEncoderConstants.remote_encoder_offset = 0;
		}
	}

	public static final class ClimberConstantsBorePID {

		public static final double kRotorRotationsPerOutputRotation = 1.0; // Rotor to unit distance

		public static final ServoMotorSubsystemConstantsBorePID kHoodServoConstants = new ServoMotorSubsystemConstantsBorePID();

		public static final double kHomingVoltage = -3.5;
		public static final double kHomingCurrentThreshold = 15.0;
		public static final double kMinHomingTime = 0.4;
		public static final double kMaxHomingTime = 10.0;

        public static double kHomePosition = 0;

		static {
			kHoodServoConstants.kName = "ClimberPID";

			kHoodServoConstants.kMainConstants.id = Ports.CLIMBER;
			kHoodServoConstants.kMainConstants.counterClockwisePositive = false;

			kHoodServoConstants.kTolerance = 1; // Degrees
			kHoodServoConstants.kRotationsPerUnitDistance = (1.0 / 360.0 * 9*5*5*1.6) /* (7.16 / 1.0)*/; // Cancoder to unit distance
			kHoodServoConstants.kKp = 2.0;
			kHoodServoConstants.kKi = 0;
			kHoodServoConstants.kKd = 0.0;

			kHoodServoConstants.kDeadband = 0; // Ticks

			kHoodServoConstants.kEnableSupplyCurrentLimit = false ;
			kHoodServoConstants.kSupplyCurrentLimit = 80;
			kHoodServoConstants.kSupplyCurrentThreshold = 0;

			kHoodServoConstants.kEnableStatorCurrentLimit = false;
			kHoodServoConstants.kStatorCurrentLimit = 120;

			kHoodServoConstants.kMaxForwardOutput = 12.0;
			kHoodServoConstants.kMaxReverseOutput = -12.0;//12

			kHoodServoConstants.kRampRate = 0.0;

			kHoodServoConstants.kNeutralMode = NeutralModeValue.Coast;

			kHoodServoConstants.encoder_id = 1;
		}
	}

	public static final class AlgaeTConstants {

		public static final double kRotorRotationsPerOutputRotation = 1.0; // Rotor to unit distance

		public static final ServoMotorSubsystemConstants kHoodServoConstants = new ServoMotorSubsystemConstants();
		public static final AbsoluteEncoderConstants kHoodEncoderConstants = new AbsoluteEncoderConstants();

        public static double kHomePosition = 0;

		public static double kCruiseVelocity1 = 800.0;
		public static double kAcceleration1 = 400.0;
			
		public static double kCruiseVelocity2 = 800.0;
		public static double kAcceleration2 = 400.0;


		static {
			kHoodServoConstants.kName = "AlgaeT";

			kHoodServoConstants.kMainConstants.id = Ports.ALGAE_T;
			kHoodServoConstants.kMainConstants.counterClockwisePositive = true;

			kHoodServoConstants.kHomePosition = 0; // Degrees
			kHoodServoConstants.kTolerance = 1; // Degrees
			kHoodServoConstants.kRotationsPerUnitDistance = (1.0 / 360.0 * 5) /* (7.16 / 1.0)*/; // Cancoder to unit distance
			kHoodServoConstants.kKp = 17.0;
			kHoodServoConstants.kKi = 0;
			kHoodServoConstants.kKd = 0.0;
			kHoodServoConstants.kKg = 0;
			kHoodServoConstants.kKs = 0.0;
			kHoodServoConstants.kDeadband = 0; // Ticks

			//kHoodServoConstants.kMinUnitsLimit = 15.0;
			//kHoodServoConstants.kMaxUnitsLimit = 62.0;

			kHoodServoConstants.kCruiseVelocity = 800.0; // degrees / s
			kHoodServoConstants.kAcceleration = 400.0; // degrees / s^2

			

			kHoodServoConstants.kEnableSupplyCurrentLimit = true;
			kHoodServoConstants.kSupplyCurrentLimit = 60;
			kHoodServoConstants.kSupplyCurrentThreshold = 0;

			kHoodServoConstants.kEnableStatorCurrentLimit = true;
			kHoodServoConstants.kStatorCurrentLimit = 120;

			kHoodServoConstants.kMaxForwardOutput = 12.0;
			kHoodServoConstants.kMaxReverseOutput = -12.0;//12

			kHoodServoConstants.kRampRate = 0.0;

			kHoodServoConstants.kNeutralMode = NeutralModeValue.Coast;

			kHoodEncoderConstants.encoder_type = FeedbackSensorSourceValue.RotorSensor; //FusedCANcoder
			kHoodEncoderConstants.remote_encoder_port = Ports.CORAL_CANCODER;
			kHoodEncoderConstants.rotor_rotations_per_output = 1.0;
			kHoodEncoderConstants.remote_encoder_offset = 0;
		}
	}

	public static final class VisionConstants {
		Transform2d robotToBottomLimelight = new com.team254.lib.geometry.Transform2d(
				new com.team254.lib.geometry.Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)),
				Rotation2d.fromDegrees(180));

				Transform2d robotToTopLimelight = new com.team254.lib.geometry.Transform2d(
					new com.team254.lib.geometry.Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(0)),
					Rotation2d.fromDegrees(180));
	}

	public static class VisionPhotonConstants{

        /**
         * Standard deviation of the limelight measurements
         */
        public static final double STANDARD_DEV_TRANSLATION = 0.5;
        public static final double STANDARD_DEV_ROTATION = 1.5;
        public static final double AMBIGUITY_THRESHOLD = 0.17;
        public static final double TAG_AREA_THRESHOLD = 0.06; //0.1;

        public static final Transform3d CAMERA_CORALL_TRANSFORM = new Transform3d(new Translation3d(Units.inchesToMeters(-6.76877), Units.inchesToMeters(-2.469746), Units.inchesToMeters(11.425)), new Rotation3d(0,0,Math.toRadians(200)));//Units.inchesToMeters(0)
        public static final Transform3d CAMERA_CORALR_TRANSFORM = new Transform3d(new Translation3d(Units.inchesToMeters(-6.76877), Units.inchesToMeters(2.469746), Units.inchesToMeters(11.425)), new Rotation3d(0,0,Math.toRadians(200)));//Units.inchesToMeters(0)
        //public static final String LAYOUT_FILE_NAME = "2024-crescendo";
    }

	//ll source x: 1.02191417 y: 0.29256156 z: 0.79701279 r: 50

	
    
}
