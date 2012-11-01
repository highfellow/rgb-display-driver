rgb-display-driver
==================

RGB LED display driver with various display modes.

This is the driver software I'm writing for an RGB LED panel. It uses software PWM to control red, green, and blue LEDs, with 10 bits of colour per channel. It has 3 input potentiometers which control the display. The effect of the controls depends on which of several display modes you are in:

* Individual control of the LEDs, with each control directly controlling the brightness on a channel.
* Hue, saturation and value. In this mode, the value (brightness) is controlled by one potentiometer, and the other two control the colour saturation and value.
* Colour cycling. The hue and saturation fade randomly from one colour to the next, with one potentiometer controlling the brightness, and the other two controlling the speed, and the amount of randomness in the speed.
* Colour organ. I'm also working on the code to make the display respond to music, with the low frequencies contrlling the red channel, medium controlling the green, and high contrlling the blue. This code is not finished yet.

If you just want a working version with the first 3 display modes, you should probably check out the commit labelled ['with speed control on colour cycling'](https://github.com/highfellow/rgb-display-driver/commit/ddcc3dfe65e3f99b1fbe4268865e5ed50fde6672). The latest commit is where I'm working on the colour organ mode; however for this to work the whole program will need rewriting using integer maths, as float maths is too slow.

When the whole project is finished, I'll try to find the time to post a circuit diagram.
