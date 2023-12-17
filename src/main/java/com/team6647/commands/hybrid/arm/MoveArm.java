/**
 * Written by Juan Pablo Gutiérrez
 * 06 10 2023
 */
package com.team6647.commands.hybrid.arm;

import com.team6647.subsystems.ArmPivotSubsystem;
import com.team6647.subsystems.ArmPivotSubsystem.ArmPivotState;

import edu.wpi.first.wpilibj2.command.Command;

public class MoveArm extends Command {
  private ArmPivotSubsystem armPivotSubsystem;
  private ArmPivotState armPivotState;

  public MoveArm(ArmPivotSubsystem armPivotSubsystem, ArmPivotState armPivotState) {
    this.armPivotSubsystem = armPivotSubsystem;
    this.armPivotState = armPivotState;

    addRequirements(armPivotSubsystem);
  }

  @Override
  public void initialize() {
    armPivotSubsystem.changeArmState(armPivotState);
  }

  
  @Override
  public boolean isFinished() {
    return armPivotSubsystem.isInTolerance();
  }


}
