# How to Document SIGNFICANT changes

**This File should have entries made whenever there are significant changes**

### 12/24/2023
1. Moving Configuration file to *TeamCode/src/main/res/xml* Folder. See (Link on Game manual Zero)[https://gm0.org/en/latest/docs/software/getting-started/using-android-studio.html?highlight=res#wireless-configuration]
    1.1 ALL OTHER XML FILES ARE NOW DELETED FOR CLAENUP (Rajesh Jain)
2. Hardware Change
   2.1 We reset the Servo Horn (Maximum allowed by goBuilda is 300-degrees in Angular Mode). So, we turned the servo to Zero, and re-mounted the horn to where it should be pointing. The 1 position is clockwise 300 degrees folded down and 0 is up and folded down a bit.
3. Pivot Servo Code change
    3.1 Now, the servo can be controlled by the D-Pad on Gamepad 2. If you hold the X button on gamepad2, the motion is more precise (slow), otherwise it will move fast. This should give a balanced speed and fine control to the servo.