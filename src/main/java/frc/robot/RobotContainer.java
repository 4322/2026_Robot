// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.constants.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOBoron;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.areaManager.AreaManager;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFx;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.spindexer.Spindexer;
import frc.robot.subsystems.shooter.spindexer.SpindexerIO;
import frc.robot.subsystems.shooter.spindexer.SpindexerIOTalonFx;
import frc.robot.subsystems.shooter.tunnel.Tunnel;
import frc.robot.subsystems.shooter.tunnel.TunnelIO;
import frc.robot.subsystems.shooter.tunnel.TunnelIOTalonFx;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOTalonFx;
import frc.robot.subsystems.vision.visionGlobalPose.VisionGlobalPose;
import frc.robot.subsystems.vision.visionObjectDetection.VisionObjectDetection;
import frc.robot.subsystems.vision.visionObjectDetection.VisionObjectDetectionIO;
import frc.robot.subsystems.vision.visionObjectDetection.VisionObjectDetectionIOPhoton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems

  private static VisionGlobalPose visionGlobalPose;
  private static VisionObjectDetection visionObjectDetection;
  private static Shooter shooter;
  private static Flywheel flywheel;
  private static Hood hood;
  private static Spindexer spindexer;
  private static Tunnel tunnel;
  private static Turret turret;

  // TODO private static Intake intake;
  private static LED led;

  private static Drive drive;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  public static ScoringManager operatorBoard = new ScoringManager(1, 2);

  private enum ShootingCommands {
    AUTO_SHOOT,
    INHIBIT_AUTO_SHOOT,
    AREA_INHIBIT_AUTO_SHOOT
  }

  private ShootingCommands lastShootingCommand = ShootingCommands.AUTO_SHOOT;
  private ShootingCommands currentShootingCommand = ShootingCommands.AUTO_SHOOT;

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Boolean suppliers
    private final BooleanSupplier toggle1 = () -> operatorBoard.getLeftController().getRawButton(Constants.Control.toggle1ButtonNumber);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL -> {
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            Constants.driveMode == Constants.SubsystemMode.DISABLED
                ? new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {})
                : new Drive(
                    new GyroIOBoron(),
                    new ModuleIOTalonFX(TunerConstants.FrontLeft),
                    new ModuleIOTalonFX(TunerConstants.FrontRight),
                    new ModuleIOTalonFX(TunerConstants.BackLeft),
                    new ModuleIOTalonFX(TunerConstants.BackRight));

        visionGlobalPose =
            Constants.visionGlobalPose == Constants.SubsystemMode.DISABLED
                ? new VisionGlobalPose() // TODO add IO for this
                : new VisionGlobalPose(); // TODO add IO for this

        visionObjectDetection =
            Constants.visionObjectDetection == Constants.SubsystemMode.DISABLED
                ? new VisionObjectDetection(drive, new VisionObjectDetectionIO() {})
                : new VisionObjectDetection(drive, new VisionObjectDetectionIOPhoton());

        flywheel =
            Constants.flywheelMode == Constants.SubsystemMode.DISABLED
                ? new Flywheel(new FlywheelIO() {})
                : new Flywheel(new FlywheelIOTalonFx());

        hood =
            Constants.hoodMode == Constants.SubsystemMode.DISABLED
                ? new Hood(new HoodIO() {})
                : new Hood(new HoodIO() {}); // TODO this will have real io

        spindexer =
            Constants.spindexerMode == Constants.SubsystemMode.DISABLED
                ? new Spindexer(new SpindexerIO() {})
                : new Spindexer(new SpindexerIOTalonFx());

        tunnel =
            Constants.tunnelMode == Constants.SubsystemMode.DISABLED
                ? new Tunnel(new TunnelIO() {})
                : new Tunnel(new TunnelIOTalonFx());

        turret =
            Constants.turretMode == Constants.SubsystemMode.DISABLED
                ? new Turret(new TurretIO() {})
                : new Turret(new TurretIOTalonFx());

        shooter = new Shooter(flywheel, hood, spindexer, tunnel, turret);

        /*
        intake = Constants.intakeMode == Constants.SubsystemMode.DISABLED ?
            new Intake() do intake
        :
            new Intake(); do intake
        */

        led = new LED();
      }

      case SIM -> {
        // Sim robot, instantiate physics sim IO implementations
        drive =
            Constants.driveMode == Constants.SubsystemMode.DISABLED
                ? new Drive(
                    new GyroIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {},
                    new ModuleIO() {})
                : new Drive(
                    new GyroIO() {},
                    new ModuleIOSim(TunerConstants.FrontLeft),
                    new ModuleIOSim(TunerConstants.FrontRight),
                    new ModuleIOSim(TunerConstants.BackLeft),
                    new ModuleIOSim(TunerConstants.BackRight));

        visionGlobalPose =
            Constants.visionGlobalPose == Constants.SubsystemMode.DISABLED
                ? new VisionGlobalPose() // TODO add IO for this
                : new VisionGlobalPose(); // TODO add IO for this

        visionObjectDetection =
            Constants.visionObjectDetection == Constants.SubsystemMode.DISABLED
                ? new VisionObjectDetection(drive, new VisionObjectDetectionIO() {})
                : new VisionObjectDetection(
                    drive, new VisionObjectDetectionIOPhoton()); // TODO add sim io

        shooter = new Shooter(flywheel, hood, spindexer, tunnel, turret);

        /*
        intake = Constants.intakeMode == Constants.SubsystemMode.DISABLED ?
            new Intake() //TODO add actual io
        :
            new Intake(); //TODO add actual io
        */

      }

      default -> {
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        visionGlobalPose = new VisionGlobalPose(); // TODO add IO for this
        visionObjectDetection =
            new VisionObjectDetection(
                drive, new VisionObjectDetectionIOPhoton()); // TODO add emppty io
        flywheel = new Flywheel(new FlywheelIO() {});
        hood = new Hood(new HoodIO() {});
        spindexer = new Spindexer(new SpindexerIO() {});
        tunnel = new Tunnel(new TunnelIO() {});
        turret = new Turret(new TurretIO() {});
        shooter = new Shooter(flywheel, hood, spindexer, tunnel, turret);
        // TODO intake = new Intake();
        led = new LED();
      }
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }
  // Triggers
  public final Trigger inNonShootingArea = new Trigger(() -> !AreaManager.isShootingArea(drive.getPose().getTranslation()));

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    

    // Shooter command bindings
shooter.setDefaultCommand(ShooterCommands.autoShoot(shooter, drive));
new JoystickButton(operatorBoard.getLeftController(), Constants.Control.toggle1ButtonNumber).onTrue(
    ShooterCommands.inhibitAutoShoot(shooter, toggle1)
    .andThen(new InstantCommand(() -> lastShootingCommand = ShootingCommands.INHIBIT_AUTO_SHOOT))
);

  }

  inNonShootingArea.whileTrue(
    ShooterCommands.areaInhibitAutoShoot(shooter, drive)
  ).onlyIf(currentShootingCommand != ShootingCommands.INHIBIT_AUTO_SHOOT && !shooter.getState() != ShooterState.UNWIND && !toggle1.getAsBoolean());

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
