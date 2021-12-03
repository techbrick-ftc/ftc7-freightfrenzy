package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.libs.Globals.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.libs.AutoImport;
import org.firstinspires.ftc.teamcode.libs.Globals;

@Autonomous(name="SimpleSlamraDrive", group="test")
public class SimpleSlamraDrive extends AutoImport {

    public SimpleSlamraDrive() { super(0, 0, 225, 150, 0, 0); }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {

            setArm(3, 1);

            packet.addLine("Starting Travel");
            dashboard.sendTelemetryPacket(packet);
            slauto.drive(-70, 0, 0, 0.5, this);
            packet.addLine("Arrived");
            dashboard.sendTelemetryPacket(packet);

            deposit(true);
            runIntake(1, 2000);

            stopCamera();
        }
    }
}
