# Raspberry Pi Background System Monitor For Shutdown and Program Admin

The intent is this is started in background as cron job and can start and stop the system

To use this monitor define the leds near allLeds array and the switches near allSwitches array
The following are the default jumpers/switches and status LED required for some features

    - Pins 40 & 39:  A normally open 'reset'/'reboot' switch 
    - Pins 30 & 29:  A jumper that when installed allows auto-start of your app
    - Pins 26 & 25:  A Status LED that gives feedback to the user. 
                     USE A RESISTOR and have cathode to pin 25 which is ground.
    - Pin  22        This is used by the Ubiquity Robotics kernel for a shutdown
                     but this program does not know about that ability.

# Typical Usage 
When setup with the 'statusLed', 'resetSwitch', and 'autostartJumper' these things can be used

    - Feedback on script operation.
      o On Powerup the statusLed will go from off to a couple short blinks and then
        left on to mean linux is up.
    
    - Reboot or full shutdown 
      o Reboot is done by a press and hold reset switch until you see statusLed blink
        (about 3 sec) then release the switch.  This will then cause a 'shutdown -r' in linux
        Just before the shutdown you will see 6 or so short blinks and the led will turn off
      o Press and hold reset switch till you see several short blinks and then long 1 sec blinks.
        Release the switch after at least one long 1 second blinks for a 'shutdown -h now' 
        Just before the full shutdown (halt) happens several medium blinks
        the led will turn off at shutdown but note the Pi often can take several seconds
        to shutdown so wait 5 seconds after the RED sysMonitor led and note the Pi green led
        has stopped blinking prior to removal of power for a clean shutdown.
    
    - AutoStart Your Application
      o See the Customization note below if you want to start your code at bootup and 
        stop your code prior to this script doing a shutdown.
      o The autostartJumper when installed at boot will start your code automagically
      You would see several short blinks then 3 or so medium blinks then your code starts
    
    - There are some other modes not documented here so look at the script
      o sysMonitor can monitor for lost processes you care about and auto-reboot if desired
        That has not been tested for a while so verify on your own.

# Installation Of The system monitor
    - make a folder in /home/yourUserName/config 
    - In the config folder form two folders called called 'logs' and one called 'bin'
    - Place this script in /home/yourUserName/config/bin
    - As root edit /etc/rc.local and have this line at the end just before the exit 0 line
      python /home/ubuntu/config/bin/sys_monitor.py  (There are other ways to do this)

# Customization of GPIO lines and other features

    - Edit and set resetSwitch to your reset switch GPIO line
    - Edit and set statusLed to your GPIO line that will drive an LED with resistor
      where a high turns on the LED
    - If you want to autostart anything:
       o define an autostartJumper GPIO line if you want a jumper to do the start
       o Form executable script in /home/yourUserName/config/bin/startSystemScript
       o Form executable script in /home/yourUserName/config/bin/stopSystemScript

# Features triggered by existance of special files in a config folder
Some of these are used and set by config_server process so maintain this AND config_server for changes!

    - stop_and_reboot.now     Causes a stop of system and reboot if it appears as this script is running
                              Will not happen within a deadtime after this script first starts
    - display_test.now        Just do continuous display test and do not start the system
    - no_auto_start.now       Causes this monitor to not start the the system.  File is NOT removed by script.
    - no_auto_restart.now     Causes this monitor to not auto-restart the system when some processes are missing
    - stop_system.now         Causes this monitor stop the system. File is removed as this is done.
    - forced_reboot.now       Causes a stop of system and reboot if it appears some time after script start
    - exit_monitor.now        Causes a stop of the monitor but not the system. File is removed on exit. 
    - suspend_led_updates.now Touched if reset is pressed so main does not update leds for a while
