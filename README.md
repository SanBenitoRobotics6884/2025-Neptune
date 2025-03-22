# **2025 Neptune | REEFSCAPE**
### IDS - [ CURRENTLY OUTDATED 3/12/2025 ]


**Swerve Configuration**

Drive Forward
- Push left joystick forward
- Ensure that the drive motors are rotating in the correct direction
- Ensure the angles are all zeroed

Strafe at 45 degrees
- Ensure all the angles are facing the correct direction

Rotate in place
- Ensure the angle are perpendicular to the center of the bot
- Ensure the drive motors are rotating in the correct direction (left counterclockwise)

Troubleshooting
- Ensure CAN IDs are set correctly
- Ensure that the correct motor type is set
- Ensure the angle offsets are correct
-- Physically straighten wheels
-- in SwerveMod.resetToAbsolute, get the initial getCANcoder().getDegrees(); for each module
-- enter that value into the angle offset in Constants
-- Sometimes that value lies and is 0, ignore those cases


**CAN IDs**
Can Coder Mappings

We are using a bitmapped addressing scheme when selecting CAN ids.

2 bits for mechanism
2 bits for location
4 bits for function

Mechanisms
00 - Swerve
01 - Pidgeon
10 - Elevator
11 - Coral

ex.
Mechanism = Swerve = 00
Location = Front Right = 01
Function = drive = 010
0001010 = 10



**Mechanisms**

```
Climb Kraken: 5
Left Elevator Motor ID: 9
Right Elevator Motor ID: 10
```


