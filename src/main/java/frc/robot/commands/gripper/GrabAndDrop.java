// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.gripper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GrabAndDrop extends Command {
  /** Creates a new GrabAndDrop. */
  private Gripper griper;
  private boolean isIn;
  private boolean done = false;
  public GrabAndDrop(Gripper gripper) {
    this.griper = gripper;
    addRequirements(griper);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isIn = griper.isIn();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(isIn){
      if(griper.isIn()){
        griper.setPower(1);
      }
      else{
        done = true;
      }
    }
    else{
      if(!griper.isIn()){
        griper.setPower(1);
      }
      else{
        done = true;
      }
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    griper.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
