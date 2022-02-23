// The algorithm that handles driving via the T265

package org.firstinspires.ftc.teamcode.libs;
import static org.firstinspires.ftc.teamcode.libs.Globals.*;

// Setting up importations
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.libs.TeleAuto;

import static java.lang.Math.abs;
import static java.lang.Math.copySign;

public class SlamraDrive {

    // Defines globally used variables
    private T265Camera camera;
    private DcMotor[] motors;
    private double currentX;
    private double currentY;
    private Rotation2d rotation;
    private double currentRadian;
    private double currentDegree;
    private T265Camera.PoseConfidence confidence;
    private Telemetry telemetry;
    private double startingRadian;
    private double startingDegree;

    private double[] wheelPowers;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    // Function which is called to pass variables and hardware to this class
    public void setUp(DcMotor[] motors, Telemetry telemetry) {
        this.motors = motors;
        this.wheelPowers = new double[motors.length];
        this.telemetry = telemetry;
        this.camera = getCamera();
    }

    public Pose2d getPose () {
        T265Camera.CameraUpdate up = camera.getLastReceivedCameraUpdate();
        Pose2d pose = new Pose2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254, up.pose.getRotation());
        return pose;
    }

    public void drive(double targetX, double targetY, double targetDegree, double speed, TeleAuto callback) {
        drive(targetX, targetY, targetDegree, speed, 0, callback, true, true, false, false);
    }

    // Main function, called to go to a target X and Y position, at a set speed and angle
    public void drive(double targetX, double targetY, double targetDegree, double speed, int timeout, TeleAuto callback, boolean doSlow, boolean doAccel, boolean ignoreX, boolean ignoreY) {

        double flPower, frPower, rlPower, rrPower;
        System.out.println("Starting Angle: " + startingDegree + "\nStarting X: ");

        double prevX = 0;
        double prevY = 0;

        double threshold;
        if (doSlow) {
            threshold = 1;
        } else {
            threshold = 2;
        }

        ElapsedTime timeoutTimer = new ElapsedTime();

        while (callback.opModeIsActive() && !callback.driverAbort()) {
            System.out.println("Start of Loop");

            // Updates position variables
            if (!getPosition()) continue;

            // Calculates the current difference between the target and current positions (or the
            // distance between them)
            double diffX = targetX - currentX;
            double diffY = targetY - currentY;
            double diffAngle = wrap(targetDegree - currentDegree);

            double diffAvg = (abs(diffX) + abs(diffY) + (abs(diffAngle) / 6)) / 3;

            if (ignoreX && abs(diffX) > abs(diffY)) {
                diffX = copySign(diffY, diffX);
            } else if (ignoreY && abs(diffY) > abs(diffX)) {
                diffY = copySign(diffX, diffY);
            }

            // Stops robot and ends the loop if the target positions and angle had been completed
            if ((abs(diffX) < threshold && abs(diffY) < threshold && abs(diffAngle) < 2 && !(ignoreX || ignoreY)) ||
                    (ignoreX && abs(diffY) < threshold && abs(diffAngle) < 2) ||
                    (ignoreY && abs(diffX) < threshold && abs(diffAngle) < 2)) {
                System.out.println("Breaking out of Loop");
                halt();
                break;
            }

            // Stops robot and ends the loop if timeout has been reached
            if (timeout != 0 && timeoutTimer.milliseconds() > timeout) {
                halt();
                break;
            }

            // Holds program if camera stops sending new data
            if (currentX == prevX && currentY == prevY) {
                halt();
                prevX = currentX; // sets prev positions
                prevY = currentY;
                System.out.println("Camera stopped sending data, reiterating loop");
                continue;
            }

            // Translates the target point's location from a field centric view to a robot centric
            // one, based on the robot's position and orientation
            double rotatedX = diffX * Math.cos(-currentRadian) - diffY * Math.sin(-currentRadian);
            double rotatedY = diffY * Math.cos(currentRadian) - diffX * Math.sin(currentRadian);

            // Calculates individual wheel powers, based on the above values
            // diffAngle is divided by a value so that it has less of an effect on wheel power
            flPower = rotatedY + rotatedX - (diffAngle / 6);
            rlPower = rotatedY - rotatedX - (diffAngle / 6);
            frPower = rotatedY - rotatedX + (diffAngle / 6);
            rrPower = rotatedY + rotatedX + (diffAngle / 6);

            // Scales down the wheel powers, so that the highest value is 1
            double max = Math.max(Math.abs(flPower), Math.abs(rlPower));
            max = Math.max(Math.abs(frPower), max);
            max = Math.max(Math.abs(rrPower), max);
            flPower /= max;
            frPower /= max;
            rlPower /= max;
            rrPower /= max;

            // Lowers wheel powers as it approaches target position
            double newSpeed = speed;
            if (doSlow) {
                doAccel = false;
                newSpeed *= clamp(0.5, 1, diffAvg / 10);
            }

            // Slowly ramps up wheel powers as it begins moving
            if (doAccel) {
                speedClimb(motors[0], flPower, newSpeed);
                speedClimb(motors[1], frPower, newSpeed);
                speedClimb(motors[2], rrPower, newSpeed);
                speedClimb(motors[3], rlPower, newSpeed);
            } else {
                motors[0].setPower(flPower * newSpeed);
                motors[1].setPower(frPower * newSpeed);
                motors[2].setPower(rrPower * newSpeed);
                motors[3].setPower(rlPower * newSpeed);
            }

            // Sets prev positions
            prevX = currentX;
            prevY = currentY;

            // Updates all telemetries
            telemetryUpdate(currentX, currentY, diffX, diffY, newSpeed);
            dashUpdate(currentX, currentY, diffX, diffY, diffAngle, newSpeed);

            System.out.println("Current X: " + currentX + "\nCurrent Y: " + currentY + "\nDiff X: " + diffX + "\nDiff Y: " + diffY + "\nCurrent Radian: " + currentRadian + "\nCurrent Degree: " + currentDegree + "\nDiff Angle: " + diffAngle + "\nConfidence: " + confidence + "\nSlamra Rotate: " + rotation + "\nNew Speed: " + newSpeed + "\nDiff Avg: " + diffAvg + "\nMotor 1 Power: " + motors[0].getPower() + "\nMotor 2 Power: " + motors[1].getPower() + "\nMotor 3 Power: " + motors[2].getPower() + "\nMotor 4 Power: " + motors[3].getPower() + "\nflPower: " + flPower + "\nfrPower: " + frPower + "\nrlPower: " + rlPower + "\nrrPower: " + rrPower);
            System.out.println("-");
        }
    }

    // Function which stops all motors, used before exiting the drive loop
    private void halt() {
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    private boolean getPosition() {
        // Gathers data from the T265 camera, saving it as a translation2d used to get the current X and Y positions
        T265Camera.CameraUpdate up = camera.getLastReceivedCameraUpdate();
        if (up.confidence == T265Camera.PoseConfidence.Failed) {
            System.out.println("Skipping loop");
            return false;
        }

        //Translation2d pose = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        Pose2d pose = new Pose2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254, up.pose.getRotation());
        // Saves the robot's current position
        currentX = pose.getX();
        currentY = pose.getY();

        rotation = pose.getRotation();
        confidence = up.confidence;

        // Saves the robot's current position
        currentRadian = /*rotation.getRadians();*/ getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        currentDegree = /*rotation.getDegrees();*/ getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        return true;
    }

    private double clamp(double min, double max, double value) {
        return Math.max(min, Math.min(value, max));
    }

    private double speedCap(double motorSpeed) {
        if (motorSpeed < 0.05 && motorSpeed > -0.05) { motorSpeed = 0; } else
        if (motorSpeed >= 0.05 && motorSpeed < 0.2) { motorSpeed = 0.2; } else
        if (motorSpeed <= -0.05 && motorSpeed > -0.2) { motorSpeed = -0.2; }
        return motorSpeed;
    }

    private void speedClimb(DcMotor motor, double targetPower, double newSpeed) {
        final double X = 0.03;
        double currentPower = motor.getPower();
        targetPower = speedCap(targetPower * newSpeed);
        if ((abs(targetPower - currentPower) > X) && (abs(targetPower) - abs(currentPower) > 0)) {
            System.out.println("Doing speedClimb");
            if (targetPower - currentPower > 0) {
                targetPower = currentPower + X;
            } else {
                targetPower = currentPower - X;
            }
        }

        motor.setPower(targetPower);
    }

    // Function which handle all telemetry
    private void telemetryUpdate(double currentX, double currentY, double diffX, double diffY, double newSpeed) {
        telemetry.addData("Current X", currentX);
        telemetry.addData("Current Y", currentY);
        telemetry.addData("Distance to X", diffX);
        telemetry.addData("Distance to Y", diffY);
        telemetry.addData("Current Angle", currentDegree);
        telemetry.addData("Current Speed", newSpeed);
        for (int i = 0; i < wheelPowers.length; i++) {
            telemetry.addData("Motor " + i + " Power", wheelPowers[i]);
        }
        telemetry.update();
    }

    private void dashUpdate(double currentX, double currentY, double diffX, double diffY, double diffAngle, double newSpeed) {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("confidence", confidence);
        packet.put("currentX", currentX);
        packet.put("currentY", currentY);
        packet.put("diffX", diffX);
        packet.put("diffY", diffY);
        packet.put("angle (degrees)", currentDegree);
        packet.put("angle (radians)", currentRadian);
        packet.put("diff angle", diffAngle);
        packet.put("m1 power", motors[0].getPower());
        packet.put("m2 power", motors[1].getPower());
        packet.put("m3 power", motors[2].getPower());
        packet.put("m4 power", motors[3].getPower());

        // Y is negated because digital y is special
        Canvas field = packet.fieldOverlay();
        final int robotRadius = 9;
        field.strokeCircle(currentX, currentY, robotRadius);
        double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
        double x1 = currentX + arrowX  / 2, y1 = currentY + arrowY / 2;
        double x2 = currentX + arrowX, y2 = currentY + arrowY;
        field.strokeLine(x1, y1, x2, y2);

        dashboard.sendTelemetryPacket(packet);
    }

    // Angle wrap functions
    private double wrap(double theta) {
        double newTheta = theta;
        while(abs(newTheta) > 180) {
            if (newTheta < -180) {
                newTheta += 360;
            } else {
                newTheta -= 360;
            }
        }
        return newTheta;
    }

    private double wrapRadians(double theta) {
        double newTheta = theta;
        while(abs(newTheta) > Math.PI) {
            if (newTheta < -Math.PI) {
                newTheta += Math.PI * 2;
            } else {
                newTheta -= Math.PI * 2;
            }
        }
        return newTheta;
    }
}