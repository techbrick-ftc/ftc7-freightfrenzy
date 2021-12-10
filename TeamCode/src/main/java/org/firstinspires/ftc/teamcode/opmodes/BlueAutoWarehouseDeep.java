package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.libs.Globals.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.libs.AutoImport;

@Autonomous(name="BlueAutoWarehouseDeep", group="blueAuto")
public class BlueAutoWarehouseDeep extends AutoImport {

    public BlueAutoWarehouseDeep() { super(65, -17, 60, 170, 158, 170); } // change

    public void runOpMode() {
        super.runOpMode();

        if (opModeIsActive()) {
            // Goes to the shipping hub and delivers based on the team element position
            // 1 is added to elementPosition because height's 0 is ground level, not the first layer
            setArm(elementPosition + 1, 1);
            sleep(1000);
            slauto.drive(47, 14, 0, 0.75, this);
            deposit(true);
            sleep(1000);
            //runIntake(1, 1000);
            //sleep(1000);
            slauto.drive(49, 14, 0, 1, 0, this, false, false);
            slauto.drive(47, 14, 0, 1, 0, this, false, false);
            deposit(false);

            // Goes into the warehouse
            slauto.drive(60, 0, 0, 0.5, 0, this, false, true);
            slauto.drive(65, -15, 0, 0.75, 3000, this, false, false);
            slauto.drive(65, -35, 0, 0.75, 3000, this, true, false);
            slauto.drive(40, -35, 0, 0.75, this);

            // Lowers arm
            setArm(0, 1);
            while (armY.isBusy()){
                sleep(10);
            }

            stopCamera();
        }
    }
}
