package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.libs.Globals.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.libs.AutoImport;

@Autonomous(name="RedAutoWarehouseNabbing", group="redAuto")
public class RedAutoWarehouseNabbing extends AutoImport {

    public RedAutoWarehouseNabbing() { super(65, 5, 68, 215, 158, 215); }

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            // Goes to the shipping hub and delivers based on the team element position
            // 1 is added to elementPosition because height's 0 is ground level, not the first layer
            setArm(elementPosition + 1, 1);
            sleep(1000);
            slauto.drive(47, -14, 0, 0.75, this);
            deposit(true);
            sleep(500);
            //runIntake(1, 1000);
            //sleep(1000);
            shimmy(1, 1, 500);
            deposit(false);

            driveUsingIMU2(-90, 0.5, armX, AxesOrder.ZYX, getImu2());

            // Attempts to get a block and score it
            slauto.drive(60, 0, 0, 0.5, 0, this, false, true);
            setArm(0, 1);
            slauto.drive(67, 15, 0, 0.75, 3000, this, false, false);
            slauto.drive(65, 30, 0, 0.75, 3000, this, false, false);

            intake.setPower(-1);
            driveUntilFull(0.5);
            intake.setPower(0);

            // resets thingies
            slauto.drive(65, 30, 0, 0.75, 3000, this, false, true);
            setArm(3, 1);
            sleep(2000);
            driveUsingIMU2(0, 0.5, armX, AxesOrder.ZYX, getImu2());
            sleep(3000);

            // tries to score any block held
            slauto.drive(65, 30, 0, 0.75, 3000, this, false, true);
            slauto.drive(67, 15, 0, 0.75, 3000, this, false, false);
            slauto.drive(60, 0, 0, 0.5, 3000, this, true, false);
            slauto.drive(47, -14, 0, 0.75, this);
            deposit(true);
            sleep(500);
            shimmy(1, 1, 500);
            //slauto.drive(50, -14, 0, 1, 0, this, false, false);
            //slauto.drive(47, -14, 0, 1, 0, this, false, false);
            sleep(500);
            deposit(false);

            // Goes into the warehouse
            slauto.drive(60, 0, 0, 0.5, 0, this, false, true);
            slauto.drive(67, 15, 0, 0.75, 3000, this, false, false);
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
