package frc.robot.subsystems;
import frc.robot.Constants.OperatorConstants;
import frc.robot.util.Conversions;
import frc.robot.util.SwerveModuleConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;






public class SwerveModule extends SubsystemBase{

     /**
     * TalonFX swerve module drive motor
     */
    private final TalonFX driveMotor;
    /**
     * TalonFX swerve module steer motor
     */
    private final TalonFX steerMotor;
    /**
     * Swerve module steer encoder (absolute angular position)
     */
    private final CANCoder steerEncoder;
    public int moduleNumber;

    public SwerveModule(SwerveModuleConstants constants, int moduleNumber) {

         driveMotor = new TalonFX(constants.driveMotorID);
         steerMotor = new TalonFX(constants.angleMotorID);
         steerEncoder = new CANCoder(constants.cancoderID);
         this.moduleNumber = moduleNumber;

        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        driveConfig.slot0.kP = OperatorConstants.KP_value_swerve;
        driveConfig.slot0.kF = OperatorConstants.KF_value_swerve;
        //add current limit
       

        driveMotor.configAllSettings(driveConfig);

        //come back to invert_type
        driveMotor.setInverted(TalonFXInvertType.Clockwise);
        driveMotor.setNeutralMode(NeutralMode.Brake);

        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();
        steerConfig.slot0.kP = OperatorConstants.KP_value_steer;

        //add current limit
        

        steerMotor.configAllSettings(steerConfig);
        steerMotor.setSensorPhase(true);
        steerMotor.setInverted(TalonFXInvertType.Clockwise);
        steerMotor.setNeutralMode(NeutralMode.Brake);

        CANCoderConfiguration steerEncoderConfig = new CANCoderConfiguration();
        steerEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        steerEncoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        steerEncoderConfig.magnetOffsetDegrees = Units.radiansToDegrees(constants.angleOffset);

        steerEncoder.configAllSettings(steerEncoderConfig);

    }
        

        

        







    public void setTargetSteerPosition(double targetSteerPosition_RAD) {
        steerMotor.set(TalonFXControlMode.Position, Conversions.degreesToFalcon(targetSteerPosition_RAD * Math.PI / 180, OperatorConstants.gear_ratio_drive));
        
    }


    public void setTargetVelocity(double targetDriveVelocity_METER_PER_SEC) {
        driveMotor.set(TalonFXControlMode.Velocity, targetDriveVelocity_METER_PER_SEC);
    }

    public void resetToAbsoluteAngle() {
        steerMotor.setSelectedSensorPosition(
                Units.degreesToRadians(steerEncoder.getAbsolutePosition() * OperatorConstants.gear_ratio_drive);
    }

    public SwerveModulePosition getMotorPosition() {
                 return new SwerveModulePosition(
                Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(), OperatorConstants.Swerve.wheelCircumference, OperatorConstants.Swerve.gear_ratio_drive); 
                
                
    }

    public void stop()  {
        driveMotor.set(ControlMode.PercentOutput, 0);
        steerMotor.set(ControlMode.PercentOutput, 0);
    }

    
    


    
}
