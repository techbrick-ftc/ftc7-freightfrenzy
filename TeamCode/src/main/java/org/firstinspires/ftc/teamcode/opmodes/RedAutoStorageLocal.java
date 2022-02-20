package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.libs.Globals.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.libs.AutoImport;

@Autonomous(name="R2 - RedAutoStorageLocal", group="redAuto")
public class RedAutoStorageLocal extends AutoImport {

    public RedAutoStorageLocal() { super(65, -40, 68, 215, 158, 215); }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            // Goes to spinner and does spinny
            slauto.drive(50, -60, -90, 0.75, 4000, this, true, true, false, false);
            setSpinny(true, 1000);

            // Goes to the shipping hub and delivers based on the team element position
            // 1 is added to elementPosition because height's 0 is ground level, not the first layer
            setArm(elementPosition + 1, 0.5);
            sleep(1000);
            slauto.drive(25, -60, -90, 0.75, 0, this, false, true, false, false);
            slauto.drive(25, -33, -90, 0.75, 0, this, true, false, false, false);
            deposit(true);

            // Does a little shimmy if it is in the highest goal, as it needs a bit of help to drop
            sleep(500);
            if (elementPosition == 2) {
                shimmy(0.8, 1, 100);
            } else {
                sleep(200);
            }

            deposit(false);

            // Parks in storage
            slauto.drive(23, -55, 0, 0.75, this);
            slauto.drive(42, -60, 0, 0.75, this);

            // Lowers arm
            setArm(0, 1);

            while (timer.seconds() < 30) {
                sleep(100);
            }
        }
        stopCamera();
    }
}