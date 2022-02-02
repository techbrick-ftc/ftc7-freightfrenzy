// abdominal planet
// Our parent auto program, which is used in other auto programs for init and stuff.

package org.firstinspires.ftc.teamcode.libs;
import static org.firstinspires.ftc.teamcode.libs.Globals.*;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.libs.SimpleSlamra;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;

public class AutoImport extends LinearOpMode implements TeleAuto {
    // Defines vars
    protected DcMotor fr = null;
    protected DcMotor rr = null;
    protected DcMotor rl = null;
    protected DcMotor fl = null;
    protected DcMotor armX = null;
    protected DcMotor armY = null;
    protected DcMotor intake = null;
    protected Servo hatch = null;
    protected Servo fork = null;
    protected CRServo spinner = null;

    protected ColorRangeSensor colorRange = null;
    protected DcMotor intakeLight = null;

    protected SimpleSlamra slauto = new SimpleSlamra();
    protected EasyOpenCVImportable camera = new EasyOpenCVImportable();

    protected FtcDashboard dashboard = FtcDashboard.getInstance();
    protected TelemetryPacket packet = new TelemetryPacket();

    protected ElapsedTime timer = new ElapsedTime();

    // vars used in program
    protected int elementPosition;
    protected int startingPoseX;
    protected int startingPoseY;
    protected int camera1X;
    protected int camera1Y;
    protected int camera2X;
    protected int camera2Y;
    protected int[] armYPositions = {-34, -63, -85, -110};
    protected int[] armYEncPositions = {0, -1930, -2800, -3700};

    protected AtomicBoolean isAsyncing = new AtomicBoolean(false);
    protected AtomicInteger targetDegree = new AtomicInteger();
    protected CompletableFuture driveUsingIMUReturn;

    protected AtomicBoolean isAsyncing2 = new AtomicBoolean(false);
    protected AtomicInteger targetDegree2 = new AtomicInteger();
    protected CompletableFuture driveUsingIMUReturn2;

    public AutoImport(int startX, int startY, int cam1X, int cam1Y, int cam2X, int cam2Y) {
        startingPoseX = startX;
        startingPoseY = startY;
        camera1X = cam1X;
        camera1Y = cam1Y;
        camera2X = cam2X;
        camera2Y = cam2Y;
    }

    public boolean driverAbort() {
        return false;
    }

    public void runOpMode() {
        // configures hardware
        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        rl = hardwareMap.get(DcMotor.class, "rl");
        rr = hardwareMap.get(DcMotor.class, "rr");
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);

        armX = hardwareMap.get(DcMotor.class, "armX");
        armX.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armX.setDirection(DcMotorSimple.Direction.REVERSE);
        armY = hardwareMap.get(DcMotor.class, "armY");
        armY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        hatch = hardwareMap.get(Servo.class, "hatch");
        fork = hardwareMap.get(Servo.class, "fork");

        spinner = hardwareMap.get(CRServo.class, "spinner");

        colorRange = hardwareMap.get(ColorRangeSensor.class, "colorRange");
        intakeLight = hardwareMap.get(DcMotor.class, "light");

        // initializes imu
        setupIMU(hardwareMap);
        telemetry.addLine("IMU Done");
        telemetry.update();

        // initializes easyopencv
        // width and height of vision box are hardcoded here
        camera.init(EasyOpenCVImportable.CameraType.WEBCAM, hardwareMap, camera1X, camera1Y, camera2X, camera2Y, 18, 18);

        // initializes slamra
        Pose2d startingPose = new Pose2d(new Translation2d(startingPoseX * 0.0254, startingPoseY * 0.0254), new Rotation2d(0));
        setupCamera(hardwareMap, startingPose);
        sleep(10000);
        startCamera();

        // passes hardware to slamra class
        DcMotor[] motors = {fr, rr, rl, fl};
        slauto.setUp(motors, telemetry);

        telemetry.addLine("Cameras Done");
        telemetry.update();

        // adds start telemetry
        telemetry.addLine("hardware configured");
        telemetry.update();
        packet.addLine("hardware configured");
        dashboard.sendTelemetryPacket(packet);

        // sets servos to starting positions
        hatch.setPosition(1);
        fork.setPosition(1);

        camera.startDetection();

        while (!isStarted()) {
            // loops this until start is pressed
            elementPosition = getElementPosition(10);
            packet.put("Position", elementPosition);
            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("Position", elementPosition);
            telemetry.update();
        }

        packet.addLine("program started");
        dashboard.sendTelemetryPacket(packet);

        timer.reset();
    }

    // Function which pushes the robot spinner into the wall, before running it. True = red
    public void setSpinny(boolean redSide, int timeout) {
        if (redSide) { // red
            slauto.drive(65, -60, -90, 0.5, timeout, this, false, true);
            spinner.setPower(-0.7);
            sleep(3000);
            spinner.setPower(0);

        } else { // blue
            slauto.drive(65, 60, 0, 0.5, timeout, this, false, true);
            spinner.setPower(0.7);
            sleep(3000);
            spinner.setPower(0);
        }
    }

    // Function which raises the arm to the required shipping hub positions
    public void setArm(int height, double power) {
        driveUsingIMU(armYPositions[height], power, armY, AxesOrder.XYZ, getImu2());
    }

    // Function which runs the intake for a certain period of time. -1 = intake, 1 = outtake
    public void runIntake(double power, int timeout) {
        intake.setPower(power);
        sleep(timeout);
        intake.setPower(0);
    }

    // Function which deposits a thing. true = opening, false = closing
    public void deposit(boolean opening) {
        if (opening) {
            hatch.setPosition(-1);
        } else {
            hatch.setPosition(1);
        }
    }

    // Function which uses the IMU to drive a motor
    // speed must be greater than 0!
    public void driveUsingIMU(double targetDegree, double speed, DcMotor motor, AxesOrder axisOrder, BNO055IMU imu){
        // Uses an atomic class variable to hold any targetDegree parameters, required to update
        // it in an existing thread.
        this.targetDegree.set((int)targetDegree);

        // Starts the thread if it isn't running.
        if (!isAsyncing.get()) {
            driveUsingIMUReturn = CompletableFuture.runAsync(() -> {
                isAsyncing.set(true);
                double imuDegree;
                double diffDegree;
                double newSpeed = speed;

                // Moves motor
                do {
                    imuDegree = imu.getAngularOrientation(AxesReference.EXTRINSIC, axisOrder, AngleUnit.DEGREES).firstAngle;
                    diffDegree = this.targetDegree.get() - imuDegree;

                    // gets a double, being 1 or -1 based on direction the motor needs to go
                    double direction = (diffDegree) / abs(diffDegree);

                    // slows down as it approaches for more precise movements
                    newSpeed = Range.clip(Math.abs(diffDegree) / 5, 0.5, 1);

                    // sets the motor to the speed, in the correct direction
                    motor.setPower((speed * newSpeed) * direction);

                    // does telemetry
                    packet.put("newSpeed", newSpeed);
                    packet.put("effective speed", (speed * newSpeed) * direction);
                    packet.put("direction", direction);
                    dashboard.sendTelemetryPacket(packet);

                    sleep(50);

                    // if the imu dies from static or whatnot, it exits
                    if (imu.getSystemStatus() == BNO055IMU.SystemStatus.IDLE) { break; }

                } while (abs(diffDegree) > 5 && opModeIsActive());
                motor.setPower(0);
                isAsyncing.set(false);
            });
        }
    }

    // Another one
    public void driveUsingIMU2(double targetDegree, double speed, DcMotor motor, AxesOrder axisOrder, BNO055IMU imu){
        // Uses an atomic class variable to hold any targetDegree parameters, required to update
        // it in an existing thread.
        this.targetDegree2.set((int)targetDegree);

        // Starts the thread if it isn't running.
        if (!isAsyncing2.get()) {
            driveUsingIMUReturn2 = CompletableFuture.runAsync(() -> {
                isAsyncing2.set(true);
                double imuDegree;
                double diffDegree;
                double newSpeed = speed;

                // Moves motor
                do {
                    imuDegree = imu.getAngularOrientation(AxesReference.INTRINSIC, axisOrder, AngleUnit.DEGREES).firstAngle;
                    diffDegree = this.targetDegree2.get() - imuDegree;

                    // gets a double, being 1 or -1 based on direction the motor needs to go
                    double direction = (diffDegree) / abs(diffDegree);

                    // slows down as it approaches for more precise movements
                    newSpeed = Range.clip(diffDegree / 10, 0.3, 1);

                    // sets the motor to the speed, in the correct direction
                    motor.setPower((speed * newSpeed) * direction);

                    // does telemetry
                    packet.put("newSpeed", newSpeed);
                    packet.put("direction", direction);
                    dashboard.sendTelemetryPacket(packet);

                    sleep(50);

                    // if the imu dies from static or whatnot, it exits
                    if (imu.getSystemStatus() == BNO055IMU.SystemStatus.IDLE) { break; }

                } while (abs(diffDegree) > 5 && opModeIsActive());
                motor.setPower(0);
                isAsyncing2.set(false);
            });
        }
    }

    public void driveUntilFull(double power) {
        ElapsedTime et = new ElapsedTime();
        while (colorRange.getLightDetected() <= 0.11 && et.milliseconds() < 2000) {
            fl.setPower(power);
            fr.setPower(power);
            rl.setPower(power);
            rr.setPower(power);
        }
        fl.setPower(0);
        fr.setPower(0);
        rl.setPower(0);
        rr.setPower(0);
    }

    public void shimmy(double power, int amount, int delay) {
        for (int i = 0; i < amount; i++) {
            fl.setPower(power);
            fr.setPower(power);
            rl.setPower(power);
            rr.setPower(power);
            sleep(delay);
            fl.setPower(-power);
            fr.setPower(-power);
            rl.setPower(-power);
            rr.setPower(-power);
            sleep(delay);
        }
        fl.setPower(0);
        fr.setPower(0);
        rl.setPower(0);
        rr.setPower(0);
    }


    // Function which uses the webcam to return the team element's position
    public int getElementPosition(long delay) {
        sleep(delay);
        return camera.getDetection();
    }

    // Function which sets encoder values to 0, and waits until they have reset
    public void resetEnc(DcMotor motor) {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (motor.getCurrentPosition() != 0) {
            sleep(10);
        }
    }

    public double wrap(double theta) {
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
}

