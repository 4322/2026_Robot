package frc.robot.test;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.vision.visionObjectDetection.VisionObjectDetection;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class TesterSelector {

  private LoggedDashboardChooser<SequentialCommandGroup> autonomousSelector =
      new LoggedDashboardChooser<SequentialCommandGroup>("Autonomous");

  public enum TestName {
    DO_NOTHING,
  }

  private class Test {
    TestName name;
    SequentialCommandGroup command;

    private Test(TestName name, SequentialCommandGroup command) {
      this.name = name;
      this.command = command;
    }
  }

  private List<Test> test;
  private TestName defaultAuto = TestName.DO_NOTHING;

  public TesterSelector(
      Drive drive,
      Hood hood,
      Turret turret,
      Shooter shooter,
      VisionObjectDetection visionObjectDetection,
      LED led,
      Intake intake) {
    test = List.of(
      new Test(defaultAuto, null)
    );

    for (Test nextAuto : test) {
      if (nextAuto.name == defaultAuto) {
        autonomousSelector.addDefaultOption(nextAuto.name.toString(), nextAuto.command);
      } else {
        autonomousSelector.addOption(nextAuto.name.toString(), nextAuto.command);
      }
    }
  }

  // public SequentialCommandGroup get() {
  //   if (Constants.currentMode == Mode.SIM) {
  //     for (Test nextTest : test) {
  //       if (nextTest.name == Simulator.getAutoScenario()) {
  //         Logger.recordOutput("AutoName", Simulator.getAutoScenario());
  //         return nextTest.command;
  //       }
  //     }
  //     System.out.println("Simulated auto " + Simulator.getAutoScenario() + " not found");
  //     System.exit(1);
  //   }
  //   return autonomousSelector.get();
  // }
}
