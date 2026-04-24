package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;

public class DeletePrevStringTester extends Command {

  public DeletePrevStringTester() {}

  @Override
  public void initialize() {
    SmartDashboard.putString(Constants.Tester.driveKeyFL, " ");
    SmartDashboard.putString(Constants.Tester.driveKeyFR, " ");
    SmartDashboard.putString(Constants.Tester.driveKeyBL, " ");
    SmartDashboard.putString(Constants.Tester.driveKeyBR, " ");
    SmartDashboard.putString(Constants.Tester.turnKeyFL, " ");
    SmartDashboard.putString(Constants.Tester.turnKeyFR, " ");
    SmartDashboard.putString(Constants.Tester.turnKeyBL, " ");
    SmartDashboard.putString(Constants.Tester.turnKeyBR, " ");
    SmartDashboard.putString(Constants.Tester.rollerKeyLeader, " ");
    SmartDashboard.putString(Constants.Tester.rollerKeyFollower, " ");
    SmartDashboard.putString(Constants.Tester.flywheelKeyLeader, " ");
    SmartDashboard.putString(Constants.Tester.flywheelKeyFollower, " ");
  }
}
