package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.libs.AutoImport;
import org.firstinspires.ftc.teamcode.libs.Globals;

@Disabled
@Autonomous(name="ForkTest", group="test")
public class ForkTest extends AutoImport {

    public ForkTest() { super(0, 0, 0, 0, 0, 0); }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            packet.put("forkRange", fork.getPosition());
            dashboard.sendTelemetryPacket(packet);

            fork.setPosition(1);

            packet.put("forkRange", fork.getPosition());
            dashboard.sendTelemetryPacket(packet);

            sleep(1000);
            fork.setPosition(0);

            packet.put("forkRange", fork.getPosition());
            dashboard.sendTelemetryPacket(packet);

            sleep(1000);
            fork.setPosition(1);

            packet.put("forkRange", fork.getPosition());
            dashboard.sendTelemetryPacket(packet);

            sleep(1000);
            fork.setPosition(0);

            packet.put("forkRange", fork.getPosition());
            dashboard.sendTelemetryPacket(packet);
        }
    }
}