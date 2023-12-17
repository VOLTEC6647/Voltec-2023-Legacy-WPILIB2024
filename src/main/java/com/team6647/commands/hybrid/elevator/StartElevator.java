/**
 * Written by Juan Pablo Gutiérrez
 */
package com.team6647.commands.hybrid.elevator;

import com.team6647.subsystems.ElevatorSubsystem;
import com.team6647.subsystems.ElevatorSubsystem.ElevatorPositionState;
import com.team6647.subsystems.ElevatorSubsystem.ElevatorState;

import edu.wpi.first.wpilibj2.command.Command;

public class StartElevator extends Command {

  private ElevatorSubsystem elevatorSubsystem;

  public StartElevator(ElevatorSubsystem elevatorSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    
    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize(){
    elevatorSubsystem.changeElevatorState(ElevatorState.MANUAL);
  }

  @Override
  public void execute() {
    elevatorSubsystem.manualMoveElevator(-0.25);
  }

  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.manualMoveElevator(0);
    elevatorSubsystem.resetElevatorPosition();
    elevatorSubsystem.changeElevatorPositionState(ElevatorPositionState.HOMED);
    elevatorSubsystem.changeElevatorState(ElevatorState.MANUAL);
  }

  @Override
  public boolean isFinished() {
    return elevatorSubsystem.getLimitState();
  }
}
