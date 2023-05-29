package frc.robot.subsystems;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PositionConstants;
import frc.robot.commands.ArmAndWrist.SetArmWithWristPosition;
import frc.robot.commands.ArmAndWrist.ArmCommands.ClosedLoopArm.PIDArmCommand;
import frc.robot.commands.ArmAndWrist.WristCommands.ClosedLoopWrist.PIDWristCommand;
import frc.robot.commands.AutoCommands.WristConsumeWithTime;
import frc.robot.commands.AutoCommands.WristEjectWithTime;
import frc.robot.subsystems.Arm.ArmSubsystem;
import frc.robot.subsystems.Swerve.DriveSubsystem;
import frc.robot.subsystems.Wrist.WristIntake;
import frc.robot.subsystems.Wrist.WristSubsystem;

import java.util.HashMap;
import java.util.List;

/**
 * A wrapper class that used Path Planer lib to generate autos and selects the
 * correct auto for the alliance color.
 */
public class Autos {

    private final DriveSubsystem m_drivetrainSubsystem;
    private final WristSubsystem m_WristSubsystem;
    private final ArmSubsystem m_ArmSubsystem;
    private final WristIntake m_WristIntake;


    private SendableChooser<String> autoChooser;

    private HashMap<String, List<Command>> m_commandMap;
    ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");

    /**
     * A wrapper class that used Path Planer lib to generate autos and selects the
     * correct auto for the alliance color.
     *
     * @param drivetrainSubsystem  The Drivetrain Subsystem
     * @param elevatorSubsystem    The Elevator Subsystem
     * @param armSubsystem         The Arm Subsystem
     * @param manipulatorSubsystem The Manipulator Subsystem
     */
    public Autos(DriveSubsystem drivetrainSubsystem, WristSubsystem wristSubsystem,
            ArmSubsystem armSubsystem, WristIntake wristIntake) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_WristSubsystem = wristSubsystem;
        m_ArmSubsystem = armSubsystem;
        m_WristIntake = wristIntake;

        // PathPlannerServer.startServer(5811);

       // HashMap<String, Command> autoCommands = new HashMap<String, Command>();

        AutoConstants.eventMap.put("Home", new SetArmWithWristPosition(m_WristSubsystem, PositionConstants.HomeWristPosition, m_ArmSubsystem, PositionConstants.HomeArmPosition));
        AutoConstants.eventMap.put("CubeGroundIntake", new SetArmWithWristPosition(m_WristSubsystem, PositionConstants.CubeIntakeWristPosition, m_ArmSubsystem, PositionConstants.CubeIntakeArmPosition));
        //AutoConstants.eventMap.put("CubeGroundIntake", new SetArmWithWristPosition(m_WristSubsystem, PositionConstants.CubeIntakeWristPosition, m_ArmSubsystem, PositionConstants.CubeIntakeArmPosition));
        
        AutoConstants.eventMap.put("ScoreMidAuto", new SetArmWithWristPosition(m_WristSubsystem, PositionConstants.ScoreMidWristPosition + 0.06, m_ArmSubsystem, PositionConstants.ScoreMidArmPosition));
        AutoConstants.eventMap.put("ScoreMid", new SetArmWithWristPosition(m_WristSubsystem, PositionConstants.ScoreMidWristPosition, m_ArmSubsystem, PositionConstants.ScoreMidArmPosition));
        
        
        AutoConstants.eventMap.put("ConeGroundIntake", new SetArmWithWristPosition(m_WristSubsystem, PositionConstants.ConeIntakeWristPosition, m_ArmSubsystem, PositionConstants.ConeIntakeArmPosition));
        AutoConstants.eventMap.put("WristConsumeWithTime", new WristConsumeWithTime(m_WristIntake, 2, AutoConstants.slowIntakeSpeed));
        AutoConstants.eventMap.put("WristEjectWithTime", new WristEjectWithTime(m_WristIntake, 2, AutoConstants.slowIntakeSpeed));
        AutoConstants.eventMap.put("WristConsumeWithTimeFast", new WristConsumeWithTime(m_WristIntake, 2, AutoConstants.fastIntakeSpeed));
        AutoConstants.eventMap.put("WristEjectWithTimeFast", new WristEjectWithTime(m_WristIntake, 2, AutoConstants.fastIntakeSpeed));
        
        AutoConstants.eventMap.put("ScoreHigh", new SetArmWithWristPosition(m_WristSubsystem, PositionConstants.ScoreHighWristPosition + 0.02, m_ArmSubsystem, PositionConstants.ScoreHighArmPosition + 0.015));
        AutoConstants.eventMap.put("ScoreBeginning", new PIDArmCommand(m_ArmSubsystem, 0.15).alongWith(new PIDWristCommand(m_WristSubsystem, 0.37)));
        

        // autoCommands.put("Balance", new AutoLevel(0.5, m_drivetrainSubsystem));

        autoChooser = new SendableChooser<>();
        autoTab.add(autoChooser);
        m_commandMap = new HashMap<>();

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            m_drivetrainSubsystem::getPose, 
            m_drivetrainSubsystem::resetOdometry,
            DriveConstants.kDriveKinematics,
            new PIDConstants(5.0, 0.0 ,0.2), //original p = 5, 1st attempt: p = 5, d = 0.5, 2nd attempt: p= 5, d = 0.5, 3rd attempt: p = 5, d = 3 this caused the wheels to shutter
            new PIDConstants(1.5, 0.0, 0), //5.0, 0, 0.2
            m_drivetrainSubsystem::setModuleStates,
            AutoConstants.eventMap,
            true,
            m_drivetrainSubsystem);



            autoChooser.setDefaultOption("DoNothing", "DoNothing");
            autoChooser.addOption("3PieceCable", "3PieceCable");
            autoChooser.addOption("3Piece", "3Piece");   
            autoChooser.addOption("2CubeBalance", "2CubeBalance");
            autoChooser.addOption("OneMobility", "OneMobility"); 

        PathConstraints standardConstraints = new PathConstraints(2.5, 3.0);


        m_commandMap.put("DoNothing", List.of(
               new InstantCommand(), new InstantCommand()));

        m_commandMap.put("3PieceCable", List.of(
                autoBuilder.fullAuto(PathPlanner.loadPathGroup("3PieceCableBLUENew", standardConstraints)),
                autoBuilder.fullAuto(PathPlanner.loadPathGroup("3PieceCableRED", standardConstraints))));

        m_commandMap.put("3Piece", List.of(
                autoBuilder.fullAuto(PathPlanner.loadPathGroup("3PieceBLUENew", standardConstraints)),
                autoBuilder.fullAuto(PathPlanner.loadPathGroup("3PieceREDNew", standardConstraints))));

        m_commandMap.put("OneMobility", List.of(
                autoBuilder.fullAuto(PathPlanner.loadPathGroup("OneMobility", standardConstraints)),
                autoBuilder.fullAuto(PathPlanner.loadPathGroup("OneMobility", standardConstraints))));

        m_commandMap.put("2CubeBalance", List.of(
            autoBuilder.fullAuto(PathPlanner.loadPathGroup("2CubeBalance", standardConstraints)),
            autoBuilder.fullAuto(PathPlanner.loadPathGroup("2CubeBalanceRed", standardConstraints))));
    
        SmartDashboard.putData(autoChooser);
    }

    public Command getAutonomousCommand() {
        String auto = autoChooser.getSelected();
        return m_commandMap.get(auto).get(DriverStation.getAlliance() == Alliance.Blue ? 0 : 1);
    }
}