package frc.robot.NewArm;

import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.NewArm.Utils.ArmConstants.STATEHEIGHT;
import frc.robot.robot1.arm.constants.ArmConstants.ArmAngleMotorConstants;
import frc.robot.robot1.arm.constants.ArmConstants.MaxErrors;
import frc.robot.utils.LogManager;
import frc.robot.utils.TalonMotor;
import static frc.robot.NewArm.Utils.ArmConstants.*;

public class NewArm extends SubsystemBase {
    TalonMotor armMotor;
    TalonMotor gripperMotor;

    boolean isCalibrated;
    STATEHEIGHT armHeights;

    //withRadians when config


    private double lastArmAngleTarget;
    private boolean hasArmAngleReachedTarget;

    public NewArm(TalonMotor armMotor, TalonMotor gripperMotor){
        this.armMotor = armMotor;
        this.gripperMotor = gripperMotor;
        isCalibrated = false;
        lastArmAngleTarget = Double.MAX_VALUE;
        hasArmAngleReachedTarget = false;

    }

    public void stop(){
        armMotor.stopMotor();
    }

    public void setPowerTest(double power){
        armMotor.setDuty(power);
    }

    public void setVelocity(double vel){
        armMotor.setVelocity(vel);
    }

    public void hadCalibrated(){
        armAngleSetPosition(BASE_ANGLE);
        isCalibrated = true;
        armHeights = STATEHEIGHT.IDLE;
    }

    private void armAngleSetPosition(double angle) {
        armMotor.setPosition(angle);
    }

    public STATEHEIGHT getState(){
        return armHeights;
    }

    public void setState(STATEHEIGHT state){
        this.armHeights = state;
    }

    public void setPositionVoltage(double armAngle, double gripperAngle) {
        armAngleMotorSetPositionVoltage(armAngle);
        //gripperAngleMotorSetPositionVoltage(gripperAngle);
    }

    public double getArmAngle() {
        return armMotor.getCurrentPosition();
      }

    public void armAngleMotorSetPositionVoltage(double targetAngle) {
        if (!isCalibrated) {
        // LogManager.log("Can not move motor before calibration", AlertType.kError);
        return;
        }
        if (Double.isNaN(targetAngle)) {
        LogManager.log("arm target Angle is NaN", AlertType.kError);
        return;
        }

        if (lastArmAngleTarget != targetAngle) {
        hasArmAngleReachedTarget = false;
        lastArmAngleTarget = targetAngle;
        }

        if (targetAngle > getArmAngle()) {
        targetAngle += 2.5*MaxErrors.ARM_ANGLE_DOWN_ERROR;
        }

        if (Math.abs(targetAngle - getArmAngle()) <= Math.toRadians(1.75)) {
        hasArmAngleReachedTarget = true;
        }

        if (targetAngle < ArmAngleMotorConstants.BACK_LIMIT) {
        targetAngle = ArmAngleMotorConstants.BACK_LIMIT;
        }
        if (targetAngle > ArmAngleMotorConstants.FWD_LIMIT) {
        targetAngle = ArmAngleMotorConstants.FWD_LIMIT;
        }

        if (hasArmAngleReachedTarget) {
        if (getArmAngle() > targetAngle) {
            if (getArmAngle() - targetAngle > MaxErrors.ARM_ANGLE_UP_ERROR) {
            armMotor.setPositionVoltage(targetAngle);
            hasArmAngleReachedTarget = false;
            } else {
                armMotor.stopMotor();
            }
        } else {
            if (targetAngle - getArmAngle() > MaxErrors.ARM_ANGLE_DOWN_ERROR) {
                armMotor.setPositionVoltage(targetAngle);
            hasArmAngleReachedTarget = false;
            } else {
                armMotor.stopMotor();
            }
        }
        } else {
            armMotor.setPositionVoltage(targetAngle);
        }

    // armAngleMotor.setPositionVoltage(targetAngle);
    }
}
