package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.libs.Globals.*;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.libs.AutoImport;
import org.firstinspires.ftc.teamcode.libs.FieldCentric;
import org.firstinspires.ftc.teamcode.libs.Globals;

@TeleOp(name="MainTele", group="teleop")
public class MainTele extends AutoImport {

    public MainTele() { super(31, -56, 225, 150, 255, 150); }
    FieldCentric drive = new FieldCentric();
    public boolean driverAbort() {
        return gamepad1.y;
    }

    @Override
    public void runOpMode() {
        super.runOpMode();

        int loops = 0;

        // adds start telemetry
        telemetry.addLine("hardware ready");
        telemetry.update();

        // Defines motor configs
        final double PI = Math.PI;
        DcMotor[] motors = {fr, rr, rl, fl};
        double[] motorAngles = {3*PI/4, 5*PI/4, 7*PI/4, PI/4};

        // Sets up motor configs
        try {
            drive.setUp(motors, motorAngles);
        } catch (Exception e) {
            packet.put("SETUP ERROR", true);
            dashboard.sendTelemetryPacket(packet);
            e.printStackTrace();
        }

        // Configures prev1 & prev2
        Gamepad prev1 = new Gamepad();
        Gamepad prev2 = new Gamepad();
        Gamepad cur1 = new Gamepad();
        Gamepad cur2 = new Gamepad();

        // Set up variables
        boolean speedy = true;
        boolean intaking = false;
        boolean outtaking = false;
        boolean hatchOpen = false;
        double armXMin = -0.8;
        double armXMax = 0.8;
        int armYSetting = 0;
        double robotAngle = getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double armYAngle = getImu2().getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        double armXAngle = getImu2().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double armXToRobot = wrap(armXAngle - robotAngle);
        double forkRange = 0;
        double armYRange = 0;
        double forkConst = 0;
        boolean forkUp = false;
        boolean imuExited = false;
        double armXPower; // this is just global bc telemetry

        // Starting servo & motor positions

        waitForStart();
    
        while (opModeIsActive()) {
            // Updates cur1 & 2
            try {
                cur1.copy(gamepad1);
                cur2.copy(gamepad2);
            } catch (RobotCoreException e) {
                packet.put("COPY ERROR", true);
                dashboard.sendTelemetryPacket(packet);
                idle();
            }

            // Gets IMUs
            robotAngle = getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            armYAngle = getImu2().getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
            armXAngle = getImu2().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            armXToRobot = wrap(armXAngle - robotAngle);

            // Gives FieldCentric the stick positions based off of speed setting
            if (cur1.b && !prev1.b ) {
                if (!speedy) {
                    speedy = true;
                    telemetry.addLine("Speedy: On");
                } else if (speedy){
                    speedy = false;
                    telemetry.addLine("Speedy: Off");
                }
                telemetry.update();
            }
            if (!speedy) {
                drive.Drive(
                        Range.clip(gamepad1.left_stick_x, -0.55, 0.55),
                        Range.clip(-gamepad1.left_stick_y, -0.5, 0.5),
                        Range.clip(-gamepad1.right_stick_x, -0.25, 0.25));

            } else {
                drive.Drive(
                        Range.clip(gamepad1.left_stick_x, -0.95, 0.95),
                        -gamepad1.left_stick_y,
                        Range.clip(-gamepad1.right_stick_x, -0.75, 0.75));

            }

            // Controls arm
            if (getImu2().getSystemStatus() == BNO055IMU.SystemStatus.RUNNING_FUSION) {
                // Increments vertical position each dpad input via IMU
                if (cur2.dpad_up && !prev2.dpad_up && (armYSetting < 3)) {
                    armYSetting++;
                    driveUsingIMU(armYPositions[armYSetting], 1, armY, AxesOrder.XYZ, getImu2());
                } else if (cur2.dpad_down && !prev2.dpad_down && (armYSetting > 0)) {
                    armYSetting--;
                    driveUsingIMU(armYPositions[armYSetting], 1, armY, AxesOrder.XYZ, getImu2());
                } else if (gamepad2.left_stick_button) {
                    // Manual control for armY
                    if (isAsyncing.get() && driveUsingIMUReturn != null) {
                        driveUsingIMUReturn.cancel(true);
                    }
                    double armYPower = gamepad2.left_stick_y;
                    armY.setPower(armYPower);
                } else if (!isAsyncing.get()) {
                    armY.setPower(0);
                }

                // Controls arm horizontal axis
                // Enforces imu barriers at 90 and -90 degrees of starting position horizontally
                if (armXToRobot <= -85) { // Its 85 to account for the arm being really fast
                    armXMax = 0.8;
                    armXMin = 0;
                } else if (armXToRobot >= 85) {
                    armXMax = 0;
                    armXMin = -0.8;
                } else {
                    armXMax = 0.8;
                    armXMin = -0.8;
                }

                armXPower = Range.clip(-gamepad2.right_stick_x, armXMin, armXMax);
                armX.setPower(armXPower);

            } else {
                // Switches Encoder, in the case that IMU fails
                if (!imuExited) {
                    System.out.println("**************************falling back on encoder***");
                    telemetry.addLine("IMU HAS DIED");
                    telemetry.update();

                    // Stops arm
                    if (driveUsingIMUReturn != null) {
                        driveUsingIMUReturn.cancel(true);
                        sleep(500);
                    }
                    armY.setPower(0);

                    imuExited = true;
                }

                // Increments vertical position each dpad input via encoder
                if (cur2.dpad_up && !prev2.dpad_up && (armYSetting < 3)) {
                    armYSetting++;
                    armY.setTargetPosition(armYEncPositions[armYSetting]);
                    armY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armY.setPower(1);
                } else if (cur2.dpad_down && !prev2.dpad_down && (armYSetting > 0)) {
                    armYSetting--;
                    armY.setTargetPosition(armYEncPositions[armYSetting]);
                    armY.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    armY.setPower(1);
                }

                // Manual control for armY
                if (gamepad2.left_stick_button) {
                    double armYPower = gamepad2.left_stick_y;
                    armY.setPower(armYPower);
                } else if (!cur2.left_stick_button && prev2.left_stick_button) {
                    armY.setPower(0);
                }

                armXPower = Range.clip(-gamepad2.right_stick_x, -0.8, 0.8);
                armX.setPower(armXPower);
            }

            // Controls fork servo
            if (cur2.y && !prev2.y && !forkUp) {
                forkConst = -0.40;
                forkUp = true;
            } else if (cur2.y && !prev2.y && forkUp) {
                forkConst = 0;
                forkUp = false;
            }

            // Transposes 0 to -180 into -1 to 1. This is unneeded for servos configured 0 to 1
            // armYRange = (Math.abs(armYAngle) / 90) - 1;

            // Transposes 0 to -180 into 0 to 1
            armYRange = Math.abs(armYAngle) / 215; // not divided by 180 to correct for inconsistencies in the fork servo configuration
            forkRange = (1 - armYRange) + forkConst;
            fork.setPosition(forkRange);

            // Toggles intake
            if (cur2.right_bumper && !prev2.right_bumper) {
                if (!intaking) {
                    intake.setPower(-1);
                    intaking = true;
                    outtaking = false;
                }
                else {
                    intake.setPower(0);
                    intaking = false;
                    outtaking = false;
                }
            } else if (cur2.left_bumper && !prev2.left_bumper) {
                if (!outtaking) {
                    intake.setPower(1);
                    intaking = false;
                    outtaking = true;
                }
                else {
                    intake.setPower(0);
                    intaking = false;
                    outtaking = false;
                }
            }

            // Toggles intake hatch
            if (!hatchOpen && (cur2.right_trigger > 0.1) && (prev2.right_trigger < 0.1)) {
                hatch.setPosition(-1);
                hatchOpen = true;
            } else if (hatchOpen && (cur2.right_trigger > 0.1) && (prev2.right_trigger < 0.1)) {
                hatch.setPosition(1);
                hatchOpen = false;
            }

            // Toggles spinner
            if (gamepad2.a) {
                spinner.setPower(1);
            } else if (gamepad2.x) {
                spinner.setPower(-1);
            } else {
                spinner.setPower(0);
            }

            // Activates light if something is in intake
            if (colorRange.getLightDetected() > 0.11) {
                intakeLight.setPower(1);
            } else {
                intakeLight.setPower(0);
            }

            // Reset Field Centric button
            if (cur1.a && !prev1.a) {
                drive.newOffset();
            }

            // Updates prev1 & 2
            try {
                prev1.copy(cur1);
                prev2.copy(cur2);
            } catch (RobotCoreException e) {
                packet.put("COPY ERROR", true);
                dashboard.sendTelemetryPacket(packet);
                idle();
            }

            // Send FTC Dashboard Packets
            packet.put("loop count", loops);
            packet.put("armXAngle", armXAngle);
            packet.put("armYAngle", armYAngle);
            packet.put("robotAngle", robotAngle);
            packet.put("armXToRobot", armXToRobot);
            packet.put("fr power", fr.getPower());
            packet.put("fl power", fl.getPower());
            packet.put("rr power", rr.getPower());
            packet.put("rl power", rl.getPower());
            packet.put("armXPower", armXPower);
            packet.put("armYStick", -gamepad2.right_stick_y);
            packet.put("armXStick", -gamepad2.left_stick_x);
            packet.put("armXEnc", armX.getCurrentPosition());
            packet.put("armYEnc", armY.getCurrentPosition());
            packet.put("armYSetting", armYSetting);
            packet.put("intakeControl", cur2.right_bumper);
            packet.put("intakePower", intake.getPower());
            packet.put("zArmRange", armYRange);
            packet.put("zAttemptForkRange", forkRange);
            packet.put("zActualForkRange", fork.getPosition());
            packet.put("aaaaArmYStatus", getImu2().getSystemStatus());
            dashboard.sendTelemetryPacket(packet);

            System.out.println("**start of print**");
            System.out.println("armYAngle: " + armYAngle);
            System.out.println("armYEnc: " + armY.getCurrentPosition());
            System.out.println("armYPower: " + armY.getPower());
            System.out.println("armYError: " + getImu2().getSystemError());
            System.out.println("armYStatus: " + getImu2().getSystemStatus());
            System.out.println("**end of print**");
            System.out.println("**");

            loops++;
        }
    }
}
