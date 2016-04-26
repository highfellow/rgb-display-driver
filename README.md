rgb-display-driver
==================

This is a project to make an open hardware display driver for 12v RGB LED strips, with various nice effects including a sound to light mode (colour organ). It is still very much work in progress, so don't expect that it will be usable for any particular purpose at the moment. I've mostly finished designing the electronics and testing them on breadboard, with circuit as shown in the KiCAD schematic. I'm just starting to work on laying out the PCB.

Below are some very rough instructions for compiling the software. I'll put up some better docs soon hopefully.

* The main arduino project is in the `code/` subdirectory.
* The `menu/` subdirectory contains a library which drives the LCD menu system. This needs to be symlinked to the libraries directory of your arduino sketchbook.
* The menu system is currently dependent on the Standard Template Library for Arduino released here: http://andybrown.me.uk/2011/01/15/the-standard-template-library-stl-for-avr-with-c-streams/ . Follow the instructions there to install this. You may need to make the patch described in this comment for it to compile. http://andybrown.me.uk/2011/01/15/the-standard-template-library-stl-for-avr-with-c-streams/#comment-2504271649 . At some point I may well rewrite the library to remove this dependency.

The `pcb/` subdirectory contains a couple of KiCAD projects - one for the main pcb and one for a smaller pcb with the controls attached.

The code files (menu and arduino project) are released under the GPL version 3: http://www.gnu.org/licenses/gpl-3.0.en.html . The KiCAD projects are released under the Creative Commons Attribution-ShareAlike license: http://creativecommons.org/licenses/by-sa/4.0/
