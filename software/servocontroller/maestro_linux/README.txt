Pololu Maestro USB Servo Controller Linux Software

Release Date: 2010-05-07
http://www.pololu.com/


== Summary ==

This binary release contains the Maestro Control Center
(MaestroControlCenter) and the Maestro command-line utility
(UscCmd).  These programs allow you to configure and control the
Maestro over USB.


== Prerequisites ==

You will need to download and install these packages:

  libusb-1.0-0-dev mono-runtime libmono-winforms2.0-cil

In Ubuntu, you can do this with the command:

  sudo apt-get install libusb-1.0-0-dev mono-runtime libmono-winforms2.0-cil


== USB Configuration ==

You will need to copy the file 99-pololu.rules to /etc/udev/rules.d/
in order to grant permission for all users to use Pololu USB devices.
If you already plugged in a Pololu USB device, you should unplug it at
this point so the new permissions will get applied later when you plug
it back in.


== Running the programs ==

You can run the programs by typing one of the following commands:

   ./MaestroControlCenter
   ./UscCmd

If you get an error message that says "cannot execute binary file",
then try running the program with the mono runtime, for example:

   mono ./UscCmd


== Source Code ==

The C# source code for UscCmd is available in the Pololu USB Software
Development Kit, available at:

  http://www.pololu.com/docs/0J41


== Text Display Problem in Ubuntu 9.10 ==

If you run MaestroControlCenter and some of the UI text is missing,
your problem might be caused by a bug in a graphics driver that comes
with Ubuntu 9.10 (Karmic Koala).

The driver is in the package xserver-xorg-video-intel.  The driver is
for the Intel i8xx and i9xx family of chipsets, including i810, i815,
i830, i845, i855, i865, i915, i945 and i965 series chips.  You can see
if your computer has one of those chipsets by running `lspci` and
finding your graphics card.

Version 2.9.0 of the driver is known to have a bug.
Upgrading to Version 2.9.1 seems to fix the bug.

You can determine what version of the driver you have by running:

  dpkg -l | grep xserver-xorg-video-intel

You can determine what version of the driver your X.org server is
actually using by looking in /var/log/Xorg.0.log for a message like:

  (II) Module intel: vendor="X.Org Foundation"
          compiled for 1.6.4, module version = 2.9.1
          Module class: X.Org Video Driver
          ABI class: X.Org Video Driver, version 5.0


One way to fix the problem is to compile the new version of the driver
(2.9.1) from source and use it.  Here are the instructions for doing
that:

1) Go to http://xorg.freedesktop.org/archive/individual/driver/
   and get the latest version of the xf86-video-intel driver.
   At the time of this writing (2009-12-23), the latest version
   was xf86-video-intel-2.9.1.tar.gz.

2) Unzip the archive and install the archive by running:

     tar -xzvf xf86-video-intel-2.9.1.tar.gz

3) Install the required dev packages:

     sudo apt-get install xserver-xorg-dev libdrm-dev x11proto-gl-dev x11proto-xf86dri-dev

4) Install the driver by running:

     cd xf86-video-intel-2.9.1
     ./configure
     make
     sudo make install

   This installs the drivers to /usr/local/lib/xorg/modules/drivers,
   but that is not the location where the X.org server searches for
   drivers.

5) (Optional) Make a backup up the existing drivers so that if
   something goes wrong you can revert to them:
     cp -Ri /usr/lib/xorg/modules/drivers ~/backup_xorg_drivers
 
6) Close all of your graphical applications and log out because we must
   temporarily shut down the Gnome Desktop Manager (gdm).

7) Go to a text console by pressing Ctrl+Alt+F1 or Ctrl+Alt+F2.

8) Run these commands:
     sudo service gdm stop
     sudo cp /usr/local/lib/xorg/modules/drivers/* /usr/lib/xorg/modules/drivers
     sudo service gdm start

9) You should see the Ubuntu log-in screen appear.  If it does not, try
   pressing Ctrl+Alt+F7.  The graphic driver bug should now be fixed!

For more information, see:
https://bugzilla.novell.com/show_bug.cgi?id=549882
https://bugs.launchpad.net/ubuntu/+source/xserver-xorg-video-intel/+bug/462349
http://intellinuxgraphics.org/
