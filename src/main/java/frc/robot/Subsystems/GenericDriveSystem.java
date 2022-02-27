package frc.robot.Subsystems;

public interface GenericDriveSystem {
    void moveForward(double distance);
    void moveBackward(double distance);
    void moveLeft(double distance);
    void moveRight(double distance);
    void spinClockwise(double degrees);
    void spinCounterclockwise(double degrees);
    void stop();
}
