# Tactical Sheep 2021-2022
Welcome to FTC Team 7's Github codebase for the Freight Frenzy season.

## Information
FTC Team 7 is a part of Techbrick Robotics, competing alongside sister teams 4234, 11215, and 19876.

### Contacts
Our outreach and support server can be found [here](https://discord.gg/BQSTuyTTUf).

Email us at <7@techbrick.org>

## Overview
Descriptions of important things.

### T265 Driving
During autonomous, we make use of the T265 for visual odometry. All driving logic for this can be found in [libs.SlamraDrive](https://github.com/techbrick-ftc/ftc7-freightfrenzy/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/libs/SlamraDrive.java), with [libs.Globals](https://github.com/techbrick-ftc/ftc7-freightfrenzy/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/libs/Globals.java) being used to hold the object.

### Autonomous Structure
All autonomous programs are derived from [libs.AutoImport](https://github.com/techbrick-ftc/ftc7-freightfrenzy/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/libs/AutoImport.java), where initialization is done and functions used are defined. Individual programs can be found in [opmodes](https://github.com/techbrick-ftc/ftc7-freightfrenzy/tree/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes).

### Teleop
Our teleop program can be found at [opmodes.MainTele](https://github.com/techbrick-ftc/ftc7-freightfrenzy/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/MainTele.java). We use [libs.FieldCentric](https://github.com/techbrick-ftc/ftc7-freightfrenzy/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/libs/FieldCentric.java) to handle the driving logic, and we can use [libs.SlamraDrive](https://github.com/techbrick-ftc/ftc7-freightfrenzy/blob/main/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/libs/SlamraDrive.java) to handle T265 driving logic during teleop.

## Important Notes
- We are temporarily using a modified version of ftc265 in our repo as a wrapper for the t265, located [here](https://github.com/techbrick-ftc/ftc7-freightfrenzy/blob/main/TeamCode/lib/lib-release.aar). It includes modifications to setPose. We are likely to continue using this unless sparib's pull request is acknowledged.
