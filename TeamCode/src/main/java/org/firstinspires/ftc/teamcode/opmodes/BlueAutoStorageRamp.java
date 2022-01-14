package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.libs.Globals.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.libs.AutoImport;

@Autonomous(name="BlueAutoStorageRamp", group="blueAuto")
public class BlueAutoStorageRamp extends AutoImport {

    public BlueAutoStorageRamp() { super(65,  27, 68, 215, 158, 215); }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            // Goes to spinner and does spinny
            slauto.drive(50, 60, 0, 0.75, 4000, this, true, true);
            setSpinny(false, 1000);

            // Goes to the shipping hub and delivers based on the team element position
            // 1 is added to elementPosition because height's 0 is ground level, not the first layer
            setArm(elementPosition + 1, 1);
            sleep(1000);
            slauto.drive(25, 55, 90, 0.75, 0, this, false, true);
            slauto.drive(25, 33, 90, 0.75, 0, this, true, false);
            deposit(true);

            // Does a little shimmy if it is in the highest goal, as it needs a bit of help to drop
            sleep(500);
            if (elementPosition == 2) {
                shimmy(0.8, 1, 100);
            } else {
                sleep(200);
            }

            deposit(false);

            // Goes over to the warehouse
            slauto.drive(25, 55, 90, 0.75, 0, this, false, true);
            setArm(3, 1);
            slauto.drive(47, 57, 0, 0.75, 0, this, true, false);

            // gains clearance
            setArm(5, 1);

            // Waits for other team before moving
            while (timer.seconds() < 20) {
                sleep(10);
            }

            // gains clearance
            setArm(5, 1);

            // Goes into the warehouse
            packet.put("zooming", "commenced");
            dashboard.sendTelemetryPacket(packet);
            slauto.drive(47, -40, 0, 1, this);

            // Lowers arm
            setArm(0, 1);

            while (timer.seconds() < 30) {
                sleep(100);
            }

            stopCamera();
        }
    }
}
