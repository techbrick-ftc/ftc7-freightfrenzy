package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.libs.Globals.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.libs.AutoImport;

@Autonomous(name="R6 - RedAutoWarehouseNabbing", group="redAuto")
public class RedAutoWarehouseNabbing extends AutoImport {

    public RedAutoWarehouseNabbing() { super(65, 5, 65, 180, 165, 180); }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            // Goes to the shipping hub and delivers based on the team element position
            // 1 is added to elementPosition because height's 0 is ground level, not the first layer
            setArm(elementPosition + 1, 1);
            sleep(1000);
            slauto.drive(47, -14, 0, 0.75, this);
            deposit(true);

            // Does a little shimmy if it is in the highest goal, as it needs a bit of help to drop
            sleep(500);
            if (elementPosition == 2) {
                shimmy(0.8, 1, 100);
            } else {
                sleep(200);
            }

            deposit(false);

            // Gets extra blocks for a certain amount of time.
            while (opModeIsActive()) {
                // Attempts to get a block and score it
                driveUsingIMU2(-90, 0.8, armX, AxesOrder.ZYX, getImu2());
                sleep(750);
                slauto.drive(63, 0, 0, 0.7, 0, this, false, true, false, false);
                setArm(0, 1);
                sleep(100);
                slauto.drive(64, 0, 0, 0.7, 0, this, false, true, false, false);
                slauto.drive(70, 30, 0, 0.9, 0, this, false, false, true, false);

                intake.setPower(-1);
                driveUntilFull(0.5);

                sleep(250);

                // resets thingies
                slauto.drive(60, 40, 0, 0.7, 3000, this, false, true, false, false);
                sleep(250);
                setArm(3, 1);
                sleep(600);
                intake.setPower(0);
                driveUsingIMU2(0, 0.7, armX, AxesOrder.ZYX, getImu2());

                if (timer.seconds() > 25) {
                    break;
                }

                // exits warehouse
                slauto.drive(65, 35, 0, 0.75, 3000, this, false, true, false, false);
                slauto.drive(75, 0, 0, 0.75, 3000, this, true, false, true, false);

                // scores any block held
                slauto.drive(47, -9, 0, 0.75, this);
                deposit(true);
                sleep(500);
                shimmy(0.8, 1, 100);
                sleep(500);
                deposit(false);
            }
        }
        stopCamera();
    }
}
