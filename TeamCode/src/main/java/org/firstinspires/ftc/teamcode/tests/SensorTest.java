package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.libs.AutoImport;
@TeleOp(name = "SensorTest", group = "test")
public class SensorTest extends AutoImport {
    ColorRangeSensor colorRange;

    public SensorTest() { super(0, 0, 0, 0, 0, 0); }

    @Override
    public void runOpMode() {
        // Get the color sensor from hardwareMap
        colorRange = hardwareMap.get(ColorRangeSensor.class, "ColorRange");

        // Wait for the Play button to be pressed
        waitForStart();

        // While the Op Mode is running, update the telemetry values.
        while (opModeIsActive()) {
            telemetry.addData("LightAmount", colorRange.getLightDetected());

            telemetry.update();
        }
    }
}