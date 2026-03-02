package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.vision.visionObjectDetection.VisionObjectDetection;
import org.littletonrobotics.junction.Logger;

public class CDepotOutpost extends SequentialCommandGroup {
  public CDepotOutpost(
      Drive drive, LED led, Intake intake) {
    setName("C_DEPOT_OUTPOST");
    addCommands(
        new InstantCommand(() -> Logger.recordOutput("Autonomous/autoStarted", true)),
        new InstantCommand(
            () -> {
              if (Robot.alliance == Alliance.Blue) {
                drive.setPose(Robot.C_Start_To_Depot.getStartingHolonomicPose().get());
              } else {
                drive.setPose(
                    Robot.C_Start_To_Depot.flipPath().getStartingHolonomicPose().get());
              }
            }),
        new ParallelCommandGroup(IntakeCommands.setIntaking(intake),
        AutoBuilder.followPath(Robot.C_Start_To_Depot).andThen(
        AutoBuilder.followPath(Robot.C_Depot_To_Outpost))));
  }

}
