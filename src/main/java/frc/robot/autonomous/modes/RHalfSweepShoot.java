package frc.robot.autonomous.modes;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

public class RHalfSweepShoot extends SequentialCommandGroup {

  public RHalfSweepShoot() {
    setName("R_HALF_SWEEP_SHOOT");
    PathPlannerPath path = Robot.R_StartR_To_NeutralR_Intake;
    Pose2d startPoseBlue = path.getStartingHolonomicPose().get();
    Pose2d startPoseRed = path.getStartingHolonomicPose().get();
  }
}
