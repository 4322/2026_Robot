

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;

public class TesterDrive extends Command{
    private Drive drive;
    private String currentFLStatus = "Front Left:";
  private String currentFRStatus = "Front Right:";
  private String currentBLStatus = "Back Left:";
  private String currentBRStatus = "Back Right:";
  private String test;
  private double numberTest;

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
        SmartDashboard.putString(
              "Tester/Drive/Front Left Color Status",
              Constants.NetworkTables.red.kMediumPurple.toHexString());
        SmartDashboard.putString(
              "Tester/Drive/Front Right Color Status",
              Constants.NetworkTables.red.kMediumPurple.toHexString());
        SmartDashboard.putString(
              "Tester/Drive/Back Left Color Status",
              Constants.NetworkTables.red.kMediumPurple.toHexString());
        SmartDashboard.putString(
              "Tester/Drive/Back Right Color Status",
              Constants.NetworkTables.red.kMediumPurple.toHexString());
          
          this.currentFLStatus = this.currentFLStatus + " Running Test " + test;
          this.currentFRStatus = this.currentFRStatus + " Running Test " + test;
          this.currentBLStatus = this.currentBLStatus + " Running Test " + test;
          this.currentBRStatus = this.currentBRStatus + " Running Test " + test;
          this.numberTest = 0;
    }

    @Override
    public void execute() {
          this.currentFLStatus = this.currentFLStatus + " #" + (numberTest +  1);
          this.currentFRStatus = this.currentFRStatus + " Running Test" + (numberTest +  1);
          this.currentBLStatus = this.currentBLStatus + " Running Test" + (numberTest +  1);
          this.currentBRStatus = this.currentBRStatus + " Running Test" + (numberTest +  1);

            if (!drive.isDriveConnected(0) || !drive.isTurnConnected(0)) {
            if (!drive.isDriveConnected(0)) {
              SmartDashboard.putString(
                  "Tester/Drive/Front Left Color Status",
                  Constants.NetworkTables.red.toHexString());
              this.currentFLStatus = this.currentFLStatus + " Drive Not Connected";
            }
            if (!drive.isTurnConnected(0)) {
            SmartDashboard.putString(
                "Tester/Drive/Front Left Color Status",
                Constants.NetworkTables.red.toHexString());
            this.currentFLStatus = this.currentFLStatus + " Turn Not Connected";
            }
          } else if (!drive.isDriveCorrectSpeed(0) || !drive.isCorrectAngleSpeed(1)) {
            if (!drive.isDriveCorrectSpeed(0)) {
            SmartDashboard.putString(
                "Tester/Drive/Front Left Color Status",
                Constants.NetworkTables.red.kOrange.toHexString());
            this.currentFLStatus =
                this.currentFLStatus
                    + " Too Slow by "
                    + (100 - ((drive.getModuleVelocity(0) / drive.requestedSpeed) * 100))
                    + "%";
            } 
            if (!drive.isCorrectAngleSpeed(0)) {
              SmartDashboard.putString(
                  "Tester/Drive/Front Left Color Status",
                  Constants.NetworkTables.red.kOrange.toHexString());
              this.currentFLStatus = this.currentFLStatus + " Incorrect Angle Speed by " + (100 - ((drive.getModuleAngle(3) / drive.anglePerSecondRequested) * 100)) + "%";
            }
          } else {
            SmartDashboard.putString(
                "Tester/Drive/Front Left Color Status",
                Constants.NetworkTables.green.toHexString());
          }


          if (!drive.isDriveConnected(1) || !drive.isTurnConnected(1)) {
            if (!drive.isDriveConnected(1)) {
              SmartDashboard.putString(
                  "Tester/Drive/Front Right Color Status",
                  Constants.NetworkTables.red.toHexString());
              this.currentFRStatus = this.currentFRStatus + " Drive Not Connected";
            }
            if (!drive.isTurnConnected(1)) {
            SmartDashboard.putString(
                "Tester/Drive/Front Right Color Status",
                Constants.NetworkTables.red.toHexString());
            this.currentFRStatus = this.currentFRStatus + " Turn Not Connected";
            }
          } else if (!drive.isDriveCorrectSpeed(1) || !drive.isCorrectAngleSpeed(1)) {
            if (!drive.isDriveCorrectSpeed(1)) {
            SmartDashboard.putString(
                "Tester/Drive/Front Right Color Status",
                Constants.NetworkTables.red.kOrange.toHexString());
            this.currentFRStatus =
                this.currentFRStatus
                    + " Too Slow by "
                    + (100 - ((drive.getModuleVelocity(1) / drive.requestedSpeed) * 100))
                    + "%";
            } 
            if (!drive.isCorrectAngleSpeed(1)) {
              SmartDashboard.putString(
                  "Tester/Drive/Front Right Color Status",
                  Constants.NetworkTables.red.kOrange.toHexString());
              this.currentFRStatus = this.currentFRStatus + " Incorrect Angle Speed by " + (100 - ((drive.getModuleAngle(3) / drive.anglePerSecondRequested) * 100)) + "%";
            }
          } else {
            SmartDashboard.putString(
                "Tester/Drive/Front Right Color Status",
                Constants.NetworkTables.green.toHexString());
          }

          if (!drive.isDriveConnected(2) || !drive.isTurnConnected(2)) {
            if (!drive.isDriveConnected(2)) {
              SmartDashboard.putString(
                  "Tester/Drive/Back Left Color Status",
                  Constants.NetworkTables.red.kBlack.toHexString());
              this.currentBLStatus = this.currentBLStatus + " Drive Not Connected";
            }
            if (!drive.isTurnConnected(2)) {
            SmartDashboard.putString(
                "Tester/Drive/Back Left Color Status",
                Constants.NetworkTables.red.kBlack.toHexString());
            this.currentBLStatus = this.currentBLStatus + " Turn Not Connected";
            }
          } else if (!drive.isDriveCorrectSpeed(2) || !drive.isCorrectAngleSpeed(2)) {
            if (!drive.isDriveCorrectSpeed(2)) {
            SmartDashboard.putString(
                "Tester/Drive/Back Left Color Status",
                Constants.NetworkTables.red.kOrange.toHexString());
            this.currentBLStatus =
                this.currentBLStatus
                    + " Too Slow by "
                    + (100 - ((drive.getModuleVelocity(2) / drive.requestedSpeed) * 100))
                    + "%";
            } 
            if (!drive.isCorrectAngleSpeed(2)) {
              SmartDashboard.putString(
                  "Tester/Drive/Back Left Color Status",
                  Constants.NetworkTables.red.kOrange.toHexString());
              this.currentBLStatus = this.currentBLStatus + " Incorrect Angle Speed by " + (100 - ((drive.getModuleAngle(3) / drive.anglePerSecondRequested) * 100)) + "%";
            }
          } else {
            SmartDashboard.putString(
                "Tester/Drive/Back Left Color Status",
                Constants.NetworkTables.green.toHexString());
          }
          
          if (!drive.isDriveConnected(3) || !drive.isTurnConnected(3)) {
            if (!drive.isDriveConnected(3)) {
              SmartDashboard.putString(
                  "Tester/Drive/Back Right Color Status",
                  Constants.NetworkTables.red.kBlack.toHexString());
              this.currentBRStatus = this.currentBRStatus + " Drive Not Connected";
            }
            if (!drive.isTurnConnected(3)) {
            SmartDashboard.putString(
                "Tester/Drive/Back Right Color Status",
                Constants.NetworkTables.red.kBlack.toHexString());
            this.currentBRStatus = this.currentBRStatus + " Turn Not Connected";
            }
          } else if (!drive.isDriveCorrectSpeed(3) || !drive.isCorrectAngleSpeed(3)) {
            if (!drive.isDriveCorrectSpeed(3)) {
            SmartDashboard.putString(
                "Tester/Drive/Back Right Color Status",
                Constants.NetworkTables.red.kOrange.toHexString());
            this.currentBRStatus =
                this.currentBRStatus
                    + " Too Slow by "
                    + (100 - ((drive.getModuleVelocity(2) / drive.requestedSpeed) * 100))
                    + "%";
            } 
            if (!drive.isCorrectAngleSpeed(3)) {
              SmartDashboard.putString(
                  "Tester/Drive/Back Right Color Status",
                  Constants.NetworkTables.red.kOrange.toHexString());
              this.currentBRStatus = this.currentBRStatus + " Incorrect Angle Speed by " + (100 - ((drive.getModuleAngle(3) / drive.anglePerSecondRequested) * 100)) + "%";
            }
          } else {
                        SmartDashboard.putString(
                "Tester/Drive/Back Right Color Status",
                Constants.NetworkTables.green.toHexString());
          }
          SmartDashboard.putString("Tester/Drive/Front Left Status", this.currentFLStatus);
          SmartDashboard.putString("Tester/Drive/Front Right Status", this.currentFRStatus);
          SmartDashboard.putString("Tester/Drive/Back Left Status", this.currentBLStatus);
          SmartDashboard.putString("Tester/Drive/Back Right Status", this.currentBRStatus);
    }
    
    @Override
    public void end(boolean interrupted) {
    }
    
}
