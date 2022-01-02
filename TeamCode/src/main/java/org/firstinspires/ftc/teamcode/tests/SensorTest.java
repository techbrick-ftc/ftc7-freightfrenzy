/*
yellow: > 0.12
white : > 0.13
none  : > 0.09
*/

package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.libs.Globals.*;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.libs.AutoImport;

@TeleOp(name="SensorTest", group="test")
public class SensorTest extends AutoImport {

    public SensorTest() { super(0, 0, 0, 0, 0, 0); }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                packet.put("LightAmount", colorRange.getLightDetected());
                packet.put("is detecting", colorRange.getLightDetected() > 0.11);
                dashboard.sendTelemetryPacket(packet);
            }
        }
    }
}


