package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.libs.Globals.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.libs.AutoImport;

@Autonomous(name="B6 - BlueAutoWarehouseNabbing", group="blueAuto")
public class BlueAutoWarehouseNabbing extends AutoImport {

    public BlueAutoWarehouseNabbing() { super(65, -17, 20, 165, 120, 165); }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            // Goes to the shipping hub and delivers based on the team element position
            // 1 is added to elementPosition because height's 0 is ground level, not the first layer
            setArm(armY, armYEnc[elementPosition + 1], 1);
            sleep(300); //waitForArmY(armYPositions[elementPosition + 1] * 0.85);
            slauto.drive(47, 12, 0, 0.8, this);
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
                setArm(armX, armXEnc[0], 1); //driveUsingIMU2(-90, 0.8, armX, AxesOrder.ZYX, getImu2());
                sleep(400);
                slauto.drive(63, 0, 0, 0.8, 0, this, false, true, false, false);
                setArm(armY, armYEnc[0], 1);
                sleep(500); //waitForArmY(-70);
                //slauto.drive(64, 0, 0, 0.8, 0, this, false, true, false, false);
                slauto.drive(70, -30, 0, 0.9, 0, this, false, false, true, false);

                intake.setPower(-1);
                driveUntilFull(-0.4);

                sleep(250);

                // resets thingies
                slauto.drive(63, -40, 0, 0.8, 3000, this, false, true, false, false);
                setArm(armY, armYEnc[3], 1);
                sleep(350);
                intake.setPower(0);
                setArm(armX, armXEnc[1], 1); //driveUsingIMU2(0, 0.8, armX, AxesOrder.ZYX, getImu2());

                if (timer.seconds() > 25) {
                    break;
                }

                // exits warehouse
                slauto.drive(65, -35, 0, 0.8, 3000, this, false, true, false, false);
                slauto.drive(75, 0, 0, 0.9, 3000, this, true, false, true, false);

                // scores any block held
                slauto.drive(47, 12, 0, 0.8, this);
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
