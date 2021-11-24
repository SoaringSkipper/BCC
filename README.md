# BCC
Boat Cruise Control

I'm Soaring Skipper and developing a Cruise Control for my RIB boat. 

Existing car cruise controls are unsuitable, because they operate via a pull cable. And once activated the gas pedal returns to starting position. The cruise control has the throttle all to itself and can pull it open as much as needed.
In a boat the cable is a stiff push-pull type, and the throttle stays fixed at the position in which you let go of it. So the throttle is also fixed, so a cruise control cannot manipulate it directly.

My solution is to "chop through" the gas cable in the middle, and place a device in between that lengthens or shortens the cable. This is done by stepper motor with a linear movement, driven by a Arduino Mega via a stepper driver TMC 2130.

A GPS signal (U-blox Neo 6M) is used to monitor the speed (SOG). A OLED display (1.3 inch) is used to display SOG, Setspeed and indications for when busy up or down, and warnings.

Three buttons are programmed, Plus, Minus and Stop. There is also a angle sensor on the input cable from the throttle lever. When the lever moves, the CC stops and resets. Just like when the Stop button is pressed. Plus and Minus are the start signal when first pushed. There after the Set Speed will be raised / lowered with 1 khm.

Since this CC is used mainly to comply with the widly imposed speed limit of 20 kmh in Holland, it will function between a trolling speed of 5 kmh and 30 kmh. Once the speed is set, only 5 kmh plus or minus is allowed. 
This is to safeguard that the extra movement this CC is exating on the throttle cable is not very much. So in case someone decides to slam the trottle, the extra length introduced by the CC won't directly be a mechanical problem, and will be compensated by extra springs in the CC mechanism. So I will allow no more extra throttle cable length than the springs will be able to handle.

