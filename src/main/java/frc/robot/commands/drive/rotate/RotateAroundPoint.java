package frc.robot;

public class RotateAroundPoint extends CommandBase {
    private double pointX;
    private double pointY;
    private double angle;

    public RotateAroundPoint(double angle, double pointX, double pointY) {
        addRequirements(RobotContainer.swerveDrive);
        this.pointX = pointX;
        this.pointY = pointY;
        this.angle = angle;
    }

    @Override
    public void execute() {
        RobotContainer.swerveDrive.rotateToAngleInPlace(angle);
    }
}