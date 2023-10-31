package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
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

    private HashMap<String, List<PathPlannerAuto>> m_commandMap;
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


            autoChooser.setDefaultOption("DoNothing", "DoNothing");
            autoChooser.addOption("3PieceCable", "3PieceCable");
            autoChooser.addOption("3Piece", "3Piece");   
            autoChooser.addOption("2CubeBalance", "2CubeBalance");
            autoChooser.addOption("OneMobility", "OneMobility"); 

        m_commandMap.put("DoNothing", List.of());

               
        m_commandMap.put("3PieceCable", List.of(
                new PathPlannerAuto("3PieceCableBLUENew"),
                new PathPlannerAuto("3PieceCableRED")));

        m_commandMap.put("3Piece", List.of(
                new PathPlannerAuto("3PieceBLUENew"),
                new PathPlannerAuto("3PieceREDNew")));

                
        m_commandMap.put("OneMobility", List.of(
                new PathPlannerAuto("OneMobility"),
                new PathPlannerAuto("OneMobility")));

        m_commandMap.put("2CubeBalance", List.of(
            new PathPlannerAuto("2CubeBalance"),
            new PathPlannerAuto("2CubeBalanceRed")));
    
        SmartDashboard.putData(autoChooser);
    }

    public Command getAutonomousCommand() {
        String auto = autoChooser.getSelected();
        return m_commandMap.get(auto).get(DriverStation.getAlliance() == Alliance.Blue ? 0 : 1);
    }
}