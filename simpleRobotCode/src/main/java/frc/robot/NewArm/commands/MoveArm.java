package frc.robot.NewArm.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.NewArm.NewArm;
import frc.robot.NewArm.Utils.ArmConstants.STATEHEIGHT;
public class MoveArm extends Command{
    NewArm arm;
    int height;
    
    public MoveArm(NewArm arm){
        this.arm = arm;
    }

    @Override
    public void initialize() {
        //arm.hadCalibrated();
    }

    @Override
    public void execute() {
        switch (arm.getState()) {
            case L1, L2, L3, PRE_ALGAE_BOTTOM, PRE_ALGAE_TOP, AFTER_ALGAE_BOTTOM, AFTER_ALGAE_TOP, CORAL_STATION, CLIMB, STARTING:
              arm.setPositionVoltage(arm.getState().armAngle, arm.getState().gripperAngle);
              break;
      
            case TESTING:
              //arm.setPositionVoltage(testArmAngle, testGripperAngle);
              break;
      
            case IDLE:
              arm.stop();
              break;
      
            default:
              arm.setState(STATEHEIGHT.IDLE);
              arm.stop();
          }
    }

    @Override
    public void end(boolean interrupted) {
        arm.setPowerTest(0);
    }

    @Override
    public boolean isFinished() {
        return false; /**TODO return */
    }
}
