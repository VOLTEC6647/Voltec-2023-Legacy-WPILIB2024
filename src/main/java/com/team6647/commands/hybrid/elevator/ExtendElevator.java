/**
 * Written by Juan Pablo Gutierrez
 */
package com.team6647.commands.hybrid.elevator;

import com.team6647.subsystems.ElevatorSubsystem;
import com.team6647.subsystems.ElevatorSubsystem.ElevatorPositionState;

import edu.wpi.first.wpilibj2.command.Command;

public class ExtendElevator extends Command {
  private ElevatorSubsystem elevatorSubsystem;
  private ElevatorPositionState elevatorState;

  public ExtendElevator(ElevatorSubsystem elevatorSubsystem,
      ElevatorPositionState elevatorState) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.elevatorState = elevatorState;

    addRequirements(elevatorSubsystem);
  }

  @Override
  public void initialize() {
    elevatorSubsystem.changeElevatorPositionState(elevatorState);
  }

  @Override
  public boolean isFinished(){
    return elevatorSubsystem.isInTolerance();
  }
}
