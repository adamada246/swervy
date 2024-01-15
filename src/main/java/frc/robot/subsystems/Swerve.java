package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Swerve extends SubsystemBase{


    public static SwerveModule[] mSwerveMods;
    public static AHRS gyro;
     SwerveDriveOdometry odometry;
     MedianFilter filter; 

    public Swerve() {
        //we can test out some values 
         filter = new MedianFilter(4);
       

        
        gyro = new AHRS(SPI.Port.kMXP);
        zeroGyro();
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(OperatorConstants.Swerve.Mod0.constants, 0),
            new SwerveModule(OperatorConstants.Swerve.Mod1.constants, 1),
            new SwerveModule(OperatorConstants.Swerve.Mod2.constants, 2),
            new SwerveModule(OperatorConstants.Swerve.Mod3.constants, 3),
        };
        
        odometry = new SwerveDriveOdometry(OperatorConstants.Swerve.kinematics, getGyro(), getModulePositions());
        
    }

    public void resetEveything() {
        gyro.reset();
        gyro.resetDisplacement();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates =
            OperatorConstants.Swerve.kinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getY(), 
                                    translation.getX(), 
                                    rotation, 
                                    getGyro()
                                )
                                : new ChassisSpeeds(
                                    translation.getY(), 
                                    translation.getX(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, OperatorConstants.Swerve.maxSpeed);
        setModuleStates(swerveModuleStates);
        }

        public void moveByChassisSpeeds(ChassisSpeeds speed){
            SwerveModuleState[] swerveModuleStates = OperatorConstants.Swerve.kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(speed, getGyro()));
            setModuleStates(swerveModuleStates);
        }

        
      public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, OperatorConstants.Swerve.maxSpeed);
        for(SwerveModule mod : mSwerveMods){

            var optimized = SwerveModuleState.optimize(desiredStates[mod.moduleNumber], getGyro());
            
            mod.setTargetSteerPosition(optimized.angle.getRadians());
            mod.setTargetVelocity(optimized.speedMetersPerSecond);
        }
    }    

//don't see why this is needed but lmk
    public boolean isAtState(){
        return false;
    }
    

    public void zeroGyro(){
        gyro.reset();
    }

    public Rotation2d getGyro() {
        return Rotation2d.fromDegrees(360- filter.calculate(gyro.getYaw()));
    }

    //public void resetFieldPosition(){
    //zeroGyro();
    //odometry.resetPosition(getGyro(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    //}

    public Pose2d getOdometryPose() {
        return odometry.getPoseMeters();
    }
    public Rotation2d getRoll(){
        return Rotation2d.fromDegrees(gyro.getRoll());
    }
    public void resetGyro(){
        gyro.reset();
        gyro.resetDisplacement();
    }

    @Override
    public void periodic(){
        
        odometry.update(getGyro(), getModulePositions());
        
      
    }

  
    
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getMotorPosition();
        }
        return positions;
    }
    public Rotation2d getPitch(){
        return Rotation2d.fromDegrees(gyro.getPitch());
    }
    
    public SwerveModule[] getModules(){
        return mSwerveMods;
    }

    public double getXGyro(){
        return gyro.getDisplacementX();
    }

    public double getYGyro(){
        return gyro.getDisplacementY();
    }

    public AHRS getAhrs(){
        return gyro;
    }
    public void stop(){        
        for(SwerveModule mod : mSwerveMods){
            mod.stop();
        }
    }
    
  

public void moveByChassisSpeeds(double forwardSpeed, double leftwardSpeed, double angSpeed, double currentAng) {
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            forwardSpeed,
            leftwardSpeed,
            angSpeed,
            Rotation2d.fromDegrees(Math.toDegrees(currentAng)));
    SwerveModuleState[] states = OperatorConstants.Swerve.kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, OperatorConstants.Swerve.maxSpeed);
    setModuleStates(states);
}

}

 
    
