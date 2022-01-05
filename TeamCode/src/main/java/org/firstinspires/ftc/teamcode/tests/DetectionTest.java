package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.libs.AutoImport;

@TeleOp(name="DetectionTest", group="test")
public class DetectionTest extends AutoImport {

    public DetectionTest() {
        // change cam values
        super(0, 0, 72, 215, 160, 215);
    }

    public void runOpMode() {
        super.runOpMode();

        dashboard.startCameraStream(camera.getWebCamera(), 0);

        // Configures prev1 & prev2
        Gamepad prev1 = new Gamepad();
        Gamepad prev2 = new Gamepad();
        Gamepad cur1 = new Gamepad();
        Gamepad cur2 = new Gamepad();

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

            // Telemetrys
            packet.put("status", camera.getDetecting());
            packet.put("detected", camera.getDetection());
            packet.put("analysis 1", camera.getAnalysis1());
            packet.put("analysis 2", camera.getAnalysis2());
            dashboard.sendTelemetryPacket(packet);

            // Try moving arm bc wtf aaaaaaaaaaa
            if (cur1.a && !prev1.a) {
                setArm(camera.getDetection() + 1, 1);
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
        }
    }
}