package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;

public class TesterDrive extends Command {
  private Drive drive;
  private String currentFLStatus = "Front Left:";
  private String currentFRStatus = "Front Right:";
  private String currentBLStatus = "Back Left:";
  private String currentBRStatus = "Back Right:";
  private Color currentFLColorStatus = new Color(0, 0, 0);
  private Color currentFRColorStatus = new Color(0, 0, 0);
  private Color currentBLColorStatus = new Color(0, 0, 0);
  private Color currentBRColorStatus = new Color(0, 0, 0);
  private String test;
  private String frontLeftKey = "Tester/Drive/Drive/FL Drive Color Status";
  private String frontRightKey = "Tester/Drive/Drive/FR Drive Color Status";
  private String backLeftKey = "Tester/Drive/Drive/BL Drive Color Status";
  private String backRightKey = "Tester/Drive/Drive/BR Drive Color Status";
  private String driveConnectionMessage = " Drive Not Connected";

  // If Not working = color
  // If working = green
  // If not connected = red
  // If too slow = orange
  // If pulling too much current = blue;
  // still running Tests = purple
  public TesterDrive(Drive drive, String test) {
    this.drive = drive;
    this.test = test;
  }

  @Override
  public void initialize() {
    this.currentFLColorStatus = Constants.NetworkTables.purple;
    this.currentFRColorStatus = Constants.NetworkTables.purple;
    this.currentBLColorStatus = Constants.NetworkTables.purple;
    this.currentBRColorStatus = Constants.NetworkTables.purple;
    SmartDashboard.putString(frontLeftKey, this.currentFLColorStatus.toHexString());
    SmartDashboard.putString(frontRightKey, this.currentFRColorStatus.toHexString());
    SmartDashboard.putString(backLeftKey, this.currentBLColorStatus.toHexString());
    SmartDashboard.putString(backRightKey, this.currentBRColorStatus.toHexString());
  }

  @Override
  public void execute() {

    if (!drive.isDriveConnected(0)) {
      this.currentFLColorStatus = Constants.NetworkTables.red;
      this.currentFLStatus = this.currentFLStatus + "FL " + driveConnectionMessage;
    } else if (!drive.isDriveCorrectSpeed(0)) {
      this.currentFLColorStatus = Constants.NetworkTables.orange;
      this.currentFLStatus = this.currentFLStatus + "FL " + " Too Slow by " + (100 - ((drive.getModuleVelocity(0) / drive.requestedSpeed) * 100)) + "% ";
    } else {
      this.currentFLColorStatus = Constants.NetworkTables.green;
    }

    if (!drive.isDriveConnected(1)) {
      this.currentFRColorStatus = Constants.NetworkTables.red;
      this.currentFRStatus = this.currentFRStatus + "FR " + driveConnectionMessage;
    } else if (!drive.isDriveCorrectSpeed(1)) {
      this.currentFRColorStatus = Constants.NetworkTables.orange;
      this.currentFRStatus = this.currentFRStatus + "FR "+
          " Too Slow by "
              + (100 - ((drive.getModuleVelocity(1) / drive.requestedSpeed) * 100))
              + "%";
    } else {
      this.currentFRColorStatus = Constants.NetworkTables.green;
    }

    if (!drive.isDriveConnected(2)) {
      this.currentBLColorStatus = Constants.NetworkTables.red;
      this.currentBLStatus = this.currentBLStatus + "BL " + driveConnectionMessage;
    } else if (!drive.isDriveCorrectSpeed(2)) {
      this.currentBLColorStatus = Constants.NetworkTables.orange;
      this.currentBLStatus =
          this.currentBLStatus + "BL " + " Too Slow by "
              + (100 - ((drive.getModuleVelocity(2) / drive.requestedSpeed) * 100))
              + "%";

    } else {
      this.currentBLColorStatus = Constants.NetworkTables.green;
    }

    if (!drive.isDriveConnected(3)) {
      this.currentBRColorStatus = Constants.NetworkTables.red;
      this.currentBRStatus = driveConnectionMessage;
    } else if (!drive.isDriveCorrectSpeed(3)) {
      this.currentBRColorStatus = Constants.NetworkTables.orange;
      this.currentBRStatus =
          this.currentBRStatus + "BR " + " Too Slow by "
              + (100 - ((drive.getModuleVelocity(3) / drive.requestedSpeed) * 100))
              + "%";
    } else {
      this.currentBRColorStatus = Constants.NetworkTables.green;
    }

    SmartDashboard.putString(backLeftKey, this.currentFLColorStatus.toHexString());
    SmartDashboard.putString(frontRightKey, this.currentFRColorStatus.toHexString());
    SmartDashboard.putString(backLeftKey, this.currentBLColorStatus.toHexString());
    SmartDashboard.putString(backRightKey, this.currentBRColorStatus.toHexString());

    SmartDashboard.putString(
        "Tester/Drive/Drive/Front Drive Left Status", SmartDashboard.getString("Tester/Drive/Drive/Front Drive Left Status", "") + test + this.currentFLStatus);
    SmartDashboard.putString(
        "Tester/Drive/Drive/Front Drive Right Status", SmartDashboard.getString("Tester/Drive/Drive/Front Drive Right Status", "") + test + this.currentFRStatus);
    SmartDashboard.putString(
        "Tester/Drive/Drive/Back Drive Left Status", SmartDashboard.getString("Tester/Drive/Drive/Back Drive Left Status", "") + test + this.currentBLStatus);
    SmartDashboard.putString(
        "Tester/Drive/Drive/Back Drive Right Status", SmartDashboard.getString("Tester/Drive/Drive/Back Drive Right Status", "") + test + this.currentBRStatus);
  }

  @Override
  public void end(boolean interrupted) {}
}
