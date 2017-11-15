# TixClockHack
Some years ago you could buy the "Tix Clock" for around $40US.
This original one was described as
 Brushed metal faceplate
 Available in Silver or Black
 LED illuminators last indefinitely
 Three display update options (every second, every 4 seconds, every minute)
 Three dimming levels
 May be mounted horizontally or vertically
 May be placed on desk or wall mounted
 Runs on 6V AC with included (US) wall transformer. Auto-sensing. 50 or 60 Hz.
 Actual size 10.2″ x 3.0″ (x 1″ deep)

One thing you don't see is battery backup so when the power goes the time is lost.
Note there the "AC" adapter. A little investigation shows that this clock use
the A/C as timebase and that is one reason it is no battery backup.

The basic build is a simple 3x9 led matrix controller by a PIC16F628. At first
I was considering reprogramming the pic to read time from an RTC but it's only
one free pin plus I suspect it might be tight on program memory. While I could
possible remove the AC sensor pin and hook up a RTC module to those two pins,
replace the pic with some different model (for more mmeory) and possible get
it talking I2C over those two pins it would be hard for me since I have no
knowledge at all regarding PIC programming and it would also be the end of the
road since it would be no way I could add a wifi connection to it or so.

Did think about options for a while and noted that the PIC is in a socket
which means all required io and even power is easily available there. I
decided to make a breakout board that will connect to the existing socket and
then a small board on the side with an arduino nano, rtc module and possible
also a wifi connection and/or radio receiver for correct time.

The code here is what is put on the arduino nano.


