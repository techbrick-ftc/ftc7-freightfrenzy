# Tactical Sheep 2021-2022
Welcome to FTC Team 7's Github codebase for the Freight Frenzy season.

## Information
FTC Team 7 is a part of Techbrick Robotics, competing alongside sister teams 4234, 11215, and 19876.

### Contacts
Our outreach and support server can be found [here](https://discord.gg/BQSTuyTTUf).
Email us at <7@techbrick.org>

## Programs
Brief descriptions of programs found in teamcode.

### libs
Contains classes that are imported elsewhere.

#### AutoImport.java
Acts as a parent to all programs, containing the code that is run during init and methods used in auto programs.

#### EasyOpenCVImportable.java
Holds methods used to interface with EasyOpenCV, most of which arent really used. Used in barcode detection.

#### FieldCentric.java
Handles teleop driving logic, utilizing field centricity.

#### Globals.java
Holds most global variables and methods.

#### SlamraDrive.java
Handles autonomous driving logic, utilizing the T265.

### opmodes
Contains all opmodes intended for competition use.

#### *side*Auto*niche*.java
A generic auto program, belonging to a side of the field and doing a specific set of tasks.

#### MainTele.java
The teleop program, used during driver control.

### tests
Contains all opmodes not intended for competition use.