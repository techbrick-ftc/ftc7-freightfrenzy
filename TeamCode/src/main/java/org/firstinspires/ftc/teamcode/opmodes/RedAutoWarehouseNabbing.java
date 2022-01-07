package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.libs.Globals.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.libs.AutoImport;

@Autonomous(name="RedAutoWarehouseNabbing", group="redAuto")
public class RedAutoWarehouseNabbing extends AutoImport {

    public RedAutoWarehouseNabbing() { super(65, 5, 72, 215, 170, 215); }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            // Goes to the shipping hub and delivers based on the team element position
            // 1 is added to elementPosition because height's 0 is ground level, not the first layer
            setArm(elementPosition + 1, 1);
            sleep(1000);
            slauto.drive(47, -14, 0, 0.75, this);
            deposit(true);
            sleep(1000);
            //runIntake(1, 1000);
            //sleep(1000);
            slauto.drive(49, -14, 0, 1, 0, this, false, false);
            slauto.drive(47, -14, 0, 1, 0, this, false, false);
            deposit(false);

            // Attempts to get a block and score it
            slauto.drive(60, 0, 0, 0.5, 0, this, false, true);
            slauto.drive(65, 15, 0, 0.75, 3000, this, false, false);
            slauto.drive(65, 30, 0, 0.75, 3000, this, false, false);

            driveUsingIMU(90, 1, armX, AxesOrder.ZYX, getImu2());
            setArm(0, 1);
            sleep(3000);

            while(colorRange.getLightDetected() <= 0.11 && timer.seconds() < 25) {
                intake.setPower(-1);
                slauto.drive(60, 32, 0, 0.75, 3000, this, true, true);
                slauto.drive(60, 55, 0, 0.8, 3000, this, false, false);
            }

            // resets thingies
            intake.setPower(0);
            setArm(3, 1);
            sleep(1000);
            driveUsingIMU2(0, 1, armX, AxesOrder.ZYX, getImu2());
            sleep(3000);

            // tries to score any block held
            slauto.drive(65, 30, 0, 0.75, 3000, this, false, true);
            slauto.drive(65, 15, 0, 0.75, 3000, this, false, false);
            slauto.drive(60, 0, 0, 0.5, 3000, this, true, false);
            slauto.drive(47, -14, 0, 0.75, this);
            deposit(true);

            // Goes into the warehouse
            slauto.drive(60, 0, 0, 0.5, 0, this, false, true);
            slauto.drive(65, 15, 0, 0.75, 3000, this, false, false);
            slauto.drive(65, 30, 0, 0.75, 3000, this, false, false);

            // Lowers arm
            setArm(0, 1);
            while (armY.isBusy()){
                sleep(10);
            }

            stopCamera();
        }
    }
}
