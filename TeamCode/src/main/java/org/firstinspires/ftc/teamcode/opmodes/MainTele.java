// The program used in teleop

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

        // Sets up motor configs
        drive.setUp(new DcMotor[] {fl, rl, fr, rr});

        // Configures prev1 & prev2
        Gamepad prev1 = new Gamepad();
        Gamepad prev2 = new Gamepad();
        Gamepad cur1 = new Gamepad();
        Gamepad cur2 = new Gamepad();

        // Set up variables
        boolean speedy = true;
        boolean armSpeedy = false;
        double armSpeedMult = 1;
        boolean intaking = false;
        boolean outtaking = false;
        boolean hatchOpen = false;
        double spinnerSpeed = 0;
        double armXMin = -0.8;
        double armXMax = 0.8;
        int armYSetting = 0;
        double robotAngle = getImu().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double armYAngle = getImu2().getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
        double armXAngle = getImu2().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double armXToRobot = wrap(armXAngle - robotAngle);
        boolean imuExited = false;
        double armXPower; // here because telemetry

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
            if (cur1.right_bumper && !prev1.right_bumper ) {
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
                        Range.clip(gamepad1.right_stick_x, -0.25, 0.25));

            } else {
                drive.Drive(
                        Range.clip(gamepad1.left_stick_x, -0.95, 0.95),
                        -gamepad1.left_stick_y,
                        Range.clip(gamepad1.right_stick_x, -0.75, 0.75));

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
                    double armYPower = gamepad2.left_stick_y * armSpeedMult;
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

                armXPower = Range.clip(-gamepad2.right_stick_x * armSpeedMult, armXMin, armXMax);
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

                // Manual control for arm
                double armYPower = gamepad2.left_stick_y * armSpeedMult;
                armY.setPower(armYPower);
                armXPower = Range.clip(-gamepad2.right_stick_x * armSpeedMult, -0.8, 0.8);
                armX.setPower(armXPower);
            }

            // toggles arm speed levels
            if (cur2.a && !prev2.a) {
                if (armSpeedy) {
                    armSpeedy = false;
                    armSpeedMult = 0.5;
                } else {
                    armSpeedy = true;
                    armSpeedMult = 1;
                }
            }

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
                    intake.setPower(0.3);
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

            // Activates spinner
            if (gamepad1.dpad_left) {
                spinnerSpeed += 0.04;
            } else if (gamepad1.dpad_right) {
                spinnerSpeed -= 0.04;
            } else {
                spinnerSpeed = 0;
            }

            spinner.setPower(Range.clip(spinnerSpeed, -1, 1));

            // Controls tape measure
            if (gamepad2.dpad_left) {
                tape.setPower(-1);
            } else if (gamepad2.dpad_right) {
                tape.setPower(1);
            } else {
                tape.setPower(0);
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
            packet.put("hatchOpen", hatchOpen);
            packet.put("hatchPosition", hatch.getPosition());
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
            packet.put("aaaaArmYStatus", getImu2().getSystemStatus());
            packet.put("x (vertical)", Globals.getImu2().getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle);
            packet.put("y (roll?)", Globals.getImu2().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES).firstAngle);
            packet.put("z (horizontal)", Globals.getImu2().getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
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
