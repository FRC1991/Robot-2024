package frc.robot.subsystems;

public interface StateSubsystem {

  void update();

  void handleStateTransition();

  void setWantedState(State a);
}
