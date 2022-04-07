// Allows drive team to increase T265 confidence from medium to high
// Run and push the robot back and forth.

package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.libs.Globals.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.libs.AutoImport;
import org.firstinspires.ftc.teamcode.libs.Globals;

@Autonomous(name="Calibration", group="setup")
public class Calibration extends AutoImport {

    public Calibration() { super(0,  0, 0, 0, 0, 0); }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            packet.addLine("Starting Calibration - Move Robot");
            dashboard.sendTelemetryPacket(packet);
            slauto.drive(100, 0, 0, 0, this);
        }
    }
}
