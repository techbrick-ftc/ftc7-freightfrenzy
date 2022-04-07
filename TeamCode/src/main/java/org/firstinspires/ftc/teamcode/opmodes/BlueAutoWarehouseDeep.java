package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.libs.Globals.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.libs.AutoImport;

@Autonomous(name="B5 - BlueAutoWarehouseDeep", group="blueAuto")
public class BlueAutoWarehouseDeep extends AutoImport {

    public BlueAutoWarehouseDeep() { super(65, -17, 20, 165, 120, 165); }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            // Goes to the shipping hub and delivers based on the team element position
            // 1 is added to elementPosition because height's 0 is ground level, not the first layer
            setArm(armY, armYEnc[elementPosition + 1], 1);
            sleep(1000);
            slauto.drive(47, 12, 0, 0.75, this);
            deposit(true);

            // Does a little shimmy if it is in the highest goal, as it needs a bit of help to drop
            sleep(500);
            if (elementPosition == 2) {
                shimmy(0.8, 1, 100);
            } else {
                sleep(200);
            }

            deposit(false);

            // Goes into the warehouse
            slauto.drive(64, 0, 0, 0.5, 0, this, false, true, false, false);
            slauto.drive(70, -40, 0, 0.7, 0, this, false, false, true, false);
            slauto.drive(40, -40, 0, 0.7, this);

            // Lowers arm
            setArm(armY, armYEnc[0], 1);

            while (timer.seconds() < 30) {
                sleep(100);
            }
        }
        stopCamera();
    }
}
