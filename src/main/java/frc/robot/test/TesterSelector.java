package frc.robot.test;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import frc.robot.subsystems.Simulator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.rollers.Rollers;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.vision.visionObjectDetection.VisionObjectDetection;
import frc.robot.test.RealTests.DriveTest;
import frc.robot.test.RealTests.FlywheelTest;
import frc.robot.test.RealTests.RollerTest;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class TesterSelector {

  private LoggedDashboardChooser<SequentialCommandGroup> testerSelector =
      new LoggedDashboardChooser<SequentialCommandGroup>("Tester");

  public enum TestName {
    DO_NOTHING,
    DRIVE_TEST,
    ROLLER_TEST,
    FLYWHEEL_TEST,
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
  private TestName defaultTestName = TestName.DO_NOTHING;

  public TesterSelector(
      Drive drive,
      Hood hood,
      Turret turret,
      Shooter shooter,
      Flywheel flywheel,
      VisionObjectDetection visionObjectDetection,
      LED led,
      Intake intake,
      Rollers rollers) {
    test =
        List.of(
            new Test(TestName.DO_NOTHING, new SequentialCommandGroup()),
            new Test(TestName.DRIVE_TEST, new DriveTest(drive)),
            new Test(TestName.ROLLER_TEST, new RollerTest(intake, rollers)),
            new Test(TestName.FLYWHEEL_TEST, new FlywheelTest(flywheel, intake, shooter, drive)));
    for (Test nextAuto : test) {
      if (nextAuto.name == defaultTestName) {
        testerSelector.addDefaultOption(nextAuto.name.toString(), nextAuto.command);
      } else {
        testerSelector.addOption(nextAuto.name.toString(), nextAuto.command);
      }
    }
  }

  public SequentialCommandGroup get() {
    if (Constants.currentMode == Mode.SIM) {
      for (Test nextTest : test) {
        if (nextTest.name == Simulator.getTestScenario()) {
          Logger.recordOutput("TesterName", Simulator.getTestScenario());
          return nextTest.command;
        }
      }
      System.out.println("Simulated test " + Simulator.getTestScenario() + " not found");
      System.exit(1);
    }
    return testerSelector.get();
  }
}
