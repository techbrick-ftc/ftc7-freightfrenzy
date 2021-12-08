// Our parent auto program, which is used in other auto programs.

package org.firstinspires.ftc.teamcode.libs;
import static org.firstinspires.ftc.teamcode.libs.Globals.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.SimpleSlamra;

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
    protected CRServo spinner = null;

    protected TouchSensor armBoundaryMin = null;

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
    protected int[] armYPositions = {0, -1930, -2800, -3700};

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
        armY = hardwareMap.get(DcMotor.class, "armY");
        armY.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //armY.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        intake = hardwareMap.get(DcMotor.class, "intake");
        hatch = hardwareMap.get(Servo.class, "hatch");

        spinner = hardwareMap.get(CRServo.class, "spinner");

        armBoundaryMin = hardwareMap.get(TouchSensor.class, "touch");

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
        sleep(5000);
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
            slauto.drive(65, -60, -90, 0.3, timeout, this, false, true);
            spinner.setPower(-0.8);
            sleep(4000);
            spinner.setPower(0);

        } else { // blue

        }
    }

    // Function which raises the arm to the required shipping hub positions
    public void setArm(int height, double power) {
        armY.setTargetPosition(armYPositions[height]);
        armY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armY.setPower(power);
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
}

