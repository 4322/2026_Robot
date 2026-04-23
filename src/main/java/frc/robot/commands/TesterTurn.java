package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

public class TesterTurn extends Command {
  private Drive drive;
  private String FLStatus;
  private String FRStatus;
  private String BLStatus;
  private String BRStatus;
  private Color FLColorStatus;
  private Color FRColorStatus;
  private Color BLColorStatus;
  private Color BRColorStatus;
  private String testName;
  private String turnConnectionMessage = " Turn Not Connected";

  // If Not working = color
  // If working = green
  // If not connected = red
  // If too slow = orange
  // If pulling too much current = blue;
  // still running Tests = purple
  public TesterTurn(Drive drive, String testName) {
    this.drive = drive;
    this.testName = testName;
  }

  @Override
  public void initialize() {
    FLColorStatus = Constants.NetworkTables.purple;
    FRColorStatus = Constants.NetworkTables.purple;
    BLColorStatus = Constants.NetworkTables.purple;
    BRColorStatus = Constants.NetworkTables.purple;
    FLStatus = null;
    FRStatus = null;
    BLStatus = null;
    BRStatus = null;
   setColorStatus();
   setTextStatus();
    Logger.recordOutput("Tester/Turn", testName);
  }

  @Override
  public void execute() {

    if (!drive.isTurnConnected(0)) {
      FLStatus = turnConnectionMessage;
      FLColorStatus = Constants.NetworkTables.red;
    } else if (!drive.isCorrectAngleSpeed(0)) {
      FLColorStatus = Constants.NetworkTables.orange;
      FLStatus =
          " Incorrect Angle Speed by "
              + (100 - ((drive.getModuleTurnVelocity(0) / drive.anglePerSecondRequested) * 100))
              + "%";
    } else {
      FLColorStatus = Constants.NetworkTables.green;
    }

    if (!drive.isTurnConnected(1)) {
      FRColorStatus = Constants.NetworkTables.red;
      FRStatus = turnConnectionMessage;
    } else if (!drive.isCorrectAngleSpeed(1)) {
      FRColorStatus = Constants.NetworkTables.orange;
      FRStatus =
          FRStatus
              + " Incorrect Angle Speed by "
              + (100 - ((drive.getModuleTurnVelocity(1) / drive.anglePerSecondRequested) * 100))
              + "%";
    } else {
      FRColorStatus = Constants.NetworkTables.green;
    }

    if (!drive.isTurnConnected(2)) {
      BLColorStatus = Constants.NetworkTables.red;
      BLStatus = turnConnectionMessage;
    } else if (!drive.isCorrectAngleSpeed(2)) {
      BLColorStatus = Constants.NetworkTables.orange;
      BLStatus =
          BLStatus
              + " Incorrect Angle Speed by "
              + (100 - ((drive.getModuleTurnVelocity(2) / drive.anglePerSecondRequested) * 100))
              + "%";

    } else {
      BLColorStatus = Constants.NetworkTables.green;
    }

    if (!drive.isTurnConnected(3)) {
      BRColorStatus = Constants.NetworkTables.red;
      BRStatus = turnConnectionMessage;
    } else if (!drive.isCorrectAngleSpeed(3)) {
      BRColorStatus = Constants.NetworkTables.orange;
      BRStatus =
          BRStatus
              + " Incorrect Angle Speed by "
              + (100 - ((drive.getModuleTurnVelocity(3) / drive.anglePerSecondRequested) * 100))
              + "%";
    } else {
      BRColorStatus = Constants.NetworkTables.green;
    }

    setTextStatus();
    setColorStatus();
  }

  @Override
  public void end(boolean interrupted) {}

  private void setTextStatus() {
    SmartDashboard.putString(
        Constants.Tester.TurnKeyFL, FLStatus);
    SmartDashboard.putString(
        Constants.Tester.TurnKeyFR, FRStatus);
    SmartDashboard.putString(
        Constants.Tester.TurnKeyBL, BLStatus);
    SmartDashboard.putString(
        Constants.Tester.TurnKeyBR, BRStatus);
  }

  private void setColorStatus(){
    SmartDashboard.putString(Constants.Tester.TurnColorKeyFL, FLColorStatus.toHexString());
    SmartDashboard.putString(Constants.Tester.TurnColorKeyFR, FRColorStatus.toHexString());
    SmartDashboard.putString(Constants.Tester.TurnColorKeyBL, BLColorStatus.toHexString());
    SmartDashboard.putString(Constants.Tester.TurnColorKeyBR, BRColorStatus.toHexString());
  }
}
