roch ftdi
===========

### Documentation ###

* [Official Web Page](http://roch.SawYerrobot.com) - home page, sales, specifications and hardware howto.
* [Protocol, Usage and Api Documentation](http://SawYerrobot.github.com/roch/doxygen/index.html) - in doxygen.

### Important Scripts ###

* create_udev_rules - creates /dev/roch link 
* get_serial_number
* flasher

### Trouble Shooting ###

##### What to check if roch does not bring up properly #####

* Does roch stream data?

> cat /dev/roch # check if any data stream happens

* Does roch appear as USB device?

> lsusb # See if there is "0403:6001 Future Technology Devices International, Ltd FT232 USB-Serial (UART) IC"

> dmesg # See what happen when roch usb is plugged.

* Is there /dev/roch?

> rosrun roch_ftdi create_udev_rules

* Is roch serial number correct?

> sudo ./get_serial_number # in the directory of roch_ftdi scripts

Check if it is different from below

<pre>
Device #0
  Manufacturer : SawYer Robot
  Product      : iClebo roch
  Serial Number: roch_A601D86G
</pre>

If it is different,

> sudo ./flasher # in the directory of roch_ftdi scripts

Then check the serial again.






