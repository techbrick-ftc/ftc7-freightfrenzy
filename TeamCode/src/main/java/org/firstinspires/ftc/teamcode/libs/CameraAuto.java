package org.firstinspires.ftc.teamcode.libraries;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.jetbrains.annotations.NotNull;

public class CameraAuto {
    private final CameraMain MAIN = new CameraMain();

    /**
     * Sets up internal variables for driving
     * @param motors Array of motors
     * @param angles Array of angles (referenced in same order as motors)
     * @param imu IMU
     * @param axesReference Axes reference for getting the IMU angles
     */
    public void setUp(DcMotor[] motors, double[] angles, BNO055IMU imu, AxesReference axesReference) {
        this.MAIN.setUpInternal(motors, angles, imu, axesReference, null);
    }

    /**
     * Sets up internal variables for driving
     * @param motors Array of motors
     * @param angles Array of angles (referenced in same order as motors)
     * @param imu IMU
     * @param axesReference Axes reference for getting the IMU angles
     * @param telemetry (Optional) Telemetry from OpMode for seeing details
     */
    public void setUp(DcMotor[] motors, double[] angles, BNO055IMU imu, AxesReference axesReference, Telemetry telemetry) {
        this.MAIN.setUpInternal(motors, angles, imu, axesReference, telemetry);
    }

    public void setPose(Pose2d pose) {
        this.MAIN.setPoseInternal(new Pose2d(pose.getTranslation().getX() * 0.0254, pose.getTranslation().getY() * 0.0254, pose.getRotation()));
    }

    public void goToPosition(double moveX, double moveY, TeleAuto callback) {
        goToPosition(moveX, moveY, 1, callback);
    }

    public void goToPosition(double moveX, double moveY, double speed, @NotNull TeleAuto callback) {
        double angle = this.MAIN.getRotation().firstAngle;

        goTo(moveX, moveY, angle, speed, callback);
    }

    public void goToRotation(double theta, TeleAuto callback) {
        goToRotation(theta, 1, callback);
    }

    public void goToRotation(double theta, double speed, @NotNull TeleAuto callback) {
        Translation2d current = this.MAIN.getPosition();
        double currentX = current.getX();
        double currentY = current.getY();

        goTo(currentX, currentY, theta, speed, callback);
    }

    public void goTo(double moveX, double moveY, double theta, TeleAuto callback) {
        goTo(moveX, moveY, theta, 1, callback);
    }

    public void goTo(double moveX, double moveY, double theta, double speed, @NotNull TeleAuto callback) {
        boolean complete;
        while (callback.opModeIsActive()) {
            complete = this.MAIN.goToInternal(moveX, moveY, theta, speed);
            if (complete) { return; }
        }
    }
}
