#
# Raspberry Pi Background System Monitor For Reset, Shutdown and autostart your app
#
# The intent is this is started in background as cron job and can start and stop the system
#
# To use this monitor define the leds near allLeds array and the switches near allSwitches array
#
# Typical Usage: (when setup with the 'statusLed', 'resetSwitch', and 'autostartJumper')
#  - Use of just the reset switch for reboot or full shutdown (For when autostartJumper is not to ground)
#    o On Powerup the statusLed will go from off to a couple short blinks and be left on to mean linux is up.
#    o Once the system is running you have a clean reset or clean shutdown possible.
#      For reset: Press and hold reset switch until you see statusLed blink (about 3 sec) then release fast.
#      This will then cause a 'shutdown -r' in linux and the system will reboot.
#      Just before the shutdown you will see 6 or so short blinks and the led will turn off then the restart happens.
#    o For shutdown: Press and hold reset switch till you see short blinks then start to see long 1 sec blinks.
#      Release the switch when you see the long 1 second blinks and a 'shutdown -h now' will happen.
#      Just before the full shutdown (halt) happens several medium blinks and the led will turn off at shutdown.
#  - See the Customization note if you want to start your code at bootup and stop your code prior to shutdown
#    If you install the autostartJumper and have configured to start your code it brings up your code at boot time.
#    You would see several short blinks then 3 or so medium blinks then your code starts if autostart jumper is on.
#  - There are some other modes such as monitoring for lost processes you care about and auto-reboot if
#    you have lost processes that are critical to your usage.  That has not been tested for a while so use as example.
#
# Installation:
#  - make a folder in /home/yourUserName/config and in that folder have folder called logs and one called bin.
#  - Place this script in /home/yourUserName/config/bin
#  - As root edit /etc/rc.local and have this line at the end just before the exit 0 line
#    python /home/ubuntu/config/bin/sys_monitor.py
#
# Customization:
#  - Edit and set resetSwitch to your reset switch GPIO line
#  - Edit and set statusLed to your GPIO line that will drive an LED with resistor where a high turns on the LED
#  - If you want to autostart anything:
#     o define an autostartJumper GPIO line if you want a jumper to do the start
#     o Form executable script in /home/yourUserName/config/bin/startSystemScript. Do a sudo chmod 775 on file
#     o Form executable script in /home/yourUserName/config/bin/stopSystemScript   Do a sudo chmod 775 on file
#     o Edit or form as required /etc/rc.local with these lines just before the final line that has 'exit 0'
#       python /home/ubuntu/config/bin/sys_monitor.py      # Use proper path, in this case in user 'ubuntu'
#
# Special Startup Options Are Done using front panel switches and/or a autostart jumper if defined
#   - Install the autostartJumper to enable autostart feature IF you have done required config steps for autostart
#   - Hold down 'MODE' switch on bootup to not autostart system
#
# Switch Features Once The System is Started
#   - Hold down 'RESET' for for 3 seconds and release as statusLed starts to blink. Status blinks 4 time and we restart
#   - Hold down 'RESET' for for 6 seconds through statusLed blinking and Status blinks 8 times and we shutdown
#   - Hold down 'MODE'  and 'RESET' switch for over 4 seconds to do halt to do power-off clean
#
# Process Watchdog Feature
#   - This monitor will watch for a failed process and if a process has stopped will
#     lead to a reboot in 4 minutes.  If more than 4 are stopped auto-reboot is disabled
#
# Features triggered by existance of special files in a config folder
# Some of these are used and set by config_server process so maintain this AND config_server for changes!
#   - stop_and_reboot.now     Causes a stop of system and reboot if it appears as this script is running
#                             Will not happen within a deadtime after this script first starts
#   - display_test.now        Just do continuous display test and do not start the system
#   - no_auto_start.now       Causes this monitor to not start the the system.  File is NOT removed by script.
#   - no_auto_restart.now     Causes this monitor to not auto-restart the system when some processes are missing
#   - stop_system.now         Causes this monitor stop the system. File is removed as this is done.
#   - forced_reboot.now       Causes a stop of system and reboot if it appears some time after script start
#   - exit_monitor.now        Causes a stop of the monitor but not the system. File is removed on exit.
#   - suspend_led_updates.now Touched if reset is pressed so main does not update leds for a while
#
# Key revisions of this script
#   20181129.1     First published version that ran on Python 2.7
#   20221105.1     Updated for python 3.x.  Added some more install notes in header of the file
#

import time
import os

import sys
import getopt
import subprocess as sp
import re
import RPi.GPIO as GPIO

# Version for this background monitor script. DO NOT CHANGE THE 3-FIELD Form!
scriptVersion = '20181129.1'

# Get a starting time in unix time in seconds for this script
scriptStartTime = time.time()

# Define option state variables here
opt_autostartSystem       = 0        # if 1 we will autostart even without autostartJumper unless another switch disallows it
opt_forcedRebootDelay     = 300.0
opt_autoRebootDelay       = 300.0

# Setup GPIO defines to be used in the script
# If a switch is 0 we are not using it's functionality but it must be present due to script usage
# You MUST DEFINE resetSwitch as this whole program uses that as minimal functionality supplied
accessSwitch=0
modeSwitch=0
resetSwitch=21        # Held less than 3 sec for restart and longer than 4 sec for shutdown -h
autostartJumper=5     # If installed will autostart the system
allSwitches= [autostartJumper, resetSwitch]

# Define the LEDS in terms of GPIO lines then define array to hold all leds
statusLed=7
errorLed=19
allLeds= [statusLed, errorLed]
shutdownLed = statusLed

ledOn=0
ledOff=1

# Define Paths used by this tool all up front when possible
systemRoot            = '/home/ubuntu/'
logFolder             = systemRoot + 'config/logs/'
logFilePath           = logFolder  + 'syslog'
exeBinPath            = systemRoot + 'bin/'
scriptPath            = systemRoot + 'config/'
exitMonitorCmdFile    = systemRoot + 'config/exit_monitor.now'
copyExeCmdFile        = systemRoot + 'config/copy_executables.now'
displayTestCmdFile    = systemRoot + 'config/display_test.now'
noAutoStartCmdFile    = systemRoot + 'config/no_auto_start.now'
noAutoRestartCmdFile  = systemRoot + 'config/no_auto_restart.now'
startSystemCodeScript = systemRoot + 'config/bin/startSystemScript'
stopSystemCodeScript  = systemRoot + 'config/bin/stopSystemScript'
stopSystemCmdFile     = systemRoot + 'config/stop_system.now'
stopAndRebootCmdFile  = systemRoot + 'config/stop_and_reboot.now'
forcedRebootCmdFile   = systemRoot + 'config/forced_reboot.now'
suspendLedUpdatesFile = systemRoot + 'config/suspend_led_updates.now'

g_normalProcsRunning  = True
g_lastTimeAllRunning  = time.time()

# If you want to detect for a missing process and then reboot if one is missing define these below
process1              = 'system_process_1'
process2              = 'system_process_2'
process3              = 'system_process_3'
requiredProcesses     = []   # [process_1, process_2, process_3]
secondsTillAutoReset  = 240.0

GPIO.setwarnings(False)

# configure GPIO lines we will use for control
GPIO.setmode(GPIO.BCM)

# ---------------------------  Required System Hooks -------------------------

# startSystemCode() can be empty or contain what it takes to start the system code
# you can use a line right here or use a script in config bin folder
def startSystemCode():
    lineForLog = time.asctime(time.gmtime()) + '  Starting system code.\n'
    logLine(lineForLog)
    # os.system(" INSERT LINE TO START SYSTEM CODE")
    if os.path.isfile(startSystemCodeScript):
        os.system(startSystemCodeScript)
    lineForLog = time.asctime(time.gmtime()) + '  System code started.\n'
    logLine(lineForLog)

# stopSystemCode() can be empty or contain what it takes to stop the system code before shutdown
# you can use a line right here or use a script in config bin folder
def stopSystemCode():
    lineForLog = time.asctime(time.gmtime()) + '  Stopping system code.\n'
    logLine(lineForLog)
    # os.system(" INSERT LINE TO STOP  SYSTEM CODE")
    if os.path.isfile(stopSystemCodeScript):
        os.system(stopSystemCodeScript)

# A common operation is to stop the system so have a routine for that
def stopSystemWithDelay(delayInSec):
    stopSystemCode()
    time.sleep(delayInSec)

# ---------------------------  Helper Routines  ------------------------------

# readSwitch reads a GPIO switch which if not pressed OR does not exist returns 1
# Activated switch that is present returns a 0
def readSwitch(gpioSwitch):
  if (gpioSwitch > 0):
       return GPIO.input(gpioSwitch)
  else:
       return 1


# Log output from this script to a log file
def logLine(line):
    try:
        f = open(logFilePath, 'a')
        try:
            lineForLog = time.asctime(time.gmtime()) + '  ' + line + '\n'
            f.write(lineForLog)
        finally:
            f.close()
    except IOError:
        pass

# Blink a single led the number of times requested with requested blink period
# The led line is first set low then high and after done are all set to final value
def blinkLed( ledGpioLine, numBlinks, timeOfBlinks, finalLedState ):
    for x in range (0, numBlinks):
        GPIO.output(ledGpioLine, ledOn)
        time.sleep(timeOfBlinks)
        GPIO.output(ledGpioLine, ledOff)
        time.sleep(timeOfBlinks)
    GPIO.output(ledGpioLine, finalLedState)

# Blink all leds the number of times requested with requested blink period
# The leds are first set low then high and after done are all set to final value
def blinkAllLeds( numBlinks, timeOfBlinks, finalLedState ):
    for x in range (0, numBlinks):
        setAllLeds(ledOn)
        time.sleep(timeOfBlinks)
        setAllLeds(ledOff)
        time.sleep(timeOfBlinks)
    setAllLeds(finalLedState)

# Set all leds on or off
def setAllLeds( ledValue ):
    for led in allLeds:
        GPIO.output(led, ledValue)

# Check of one of our main executable processes is running
def isBinProcessRunning( process_name ):
    proc = os.popen("ps -eaf | grep "+process_name)
    output = proc.read()
    proc.close()
    if re.search(exeBinPath+process_name, output) is None:
        return False
    else:
        return True

# Start just one process
def startBinProcess( process_name ):
    cmd = 'sudo ' + exeBinPath+process_name + ' > /dev/null '
    logLine('DEBUG: !!! FTP Process to be restarted with ' + cmd)
    os.spawnl(os.P_DETACH, cmd)
    logLine('DEBUG: !!! FTP Process restarted')
    return True


# check for required number of processes for operation
# WARNING: Uses globals so this is just to keep code looking cleaner
def checkProcesses(procCheckTimer):
    global g_normalProcsRunning
    global g_lastTimeAllRunning

    numRunningProcesses = 0
    for process in requiredProcesses:
        if isBinProcessRunning( process ) == True:
            numRunningProcesses += 1

    numCheckedProcesses = len(requiredProcesses)
    if numRunningProcesses == numCheckedProcesses:
        g_lastTimeAllRunning = time.time()
    else:
        # To prevent undesired reboots for planned non-running times
        # we only activate auto-restart if at least a few are running
        if numRunningProcesses > numCheckedProcesses - int(4):
            if g_normalProcsRunning == True:
                # TODO: It would be nice to show when more die but lets limit scope to showing just first one
                logLine("ALERT: Only " + str(numRunningProcesses) + " out of " + str(numCheckedProcesses) + " processes running!")
                # First time seen missing
                g_normalProcsRunning = False
            else:
                # If down for long time we restart the system
                if (time.time() - g_lastTimeAllRunning) > float(secondsTillAutoReset):
                    logLine("ERROR! Only " + str(numRunningProcesses) + " out of " + str(numCheckedProcesses) + " processes running!  REBOOT NOW!")
                    g_normalProcsRunning = True
                    g_lastTimeAllRunning = time.time()
                    os.system('touch ' + stopAndRebootCmdFile + ' 2> /dev/null');
        else:
            # if system gets stopped we reset to as if all is well
            if g_normalProcsRunning == False:
                logLine("ALERT: Now " + str(numRunningProcesses) + " running out of " + str(numCheckedProcesses) + ". Allow this case.")
            g_normalProcsRunning = True

def printUsage():
    print ('seismo_monitor usage:  seismo_monitor.py [options]')
    print ('  -r    Set forced reboot delay to 0' )
    print ('  -s    Do not start the system on this run')
    print ('  -h    print this help menu' )

# -----------------------------   Start Main ----------------------------------
#
# Setup for log lines to a file
if not os.path.isdir(logFolder):
    os.system('mkdir ' + logFolder + ' 2> /dev/null');
if not os.path.isfile(logFilePath):
    os.system('touch ' + logFilePath + ' 2> /dev/null');

retValue = 0;

logLine(os.path.basename(__file__) + ' version ' + scriptVersion + ' background monitor starting now.')

# Get options for this run of the script
try:
    opts, args = getopt.getopt(sys.argv[1:], "hrsf")
except getopt.GetoptError as err:
    print (str(err))  # will print something like "option -a not recognized"
    printUsage()
    logLine(os.path.basename(__file__) + ' background monitor exiting due to bad option entererd.')
    sys.exit(2)

output = None
verbose = False
for opt, arg in opts:
    if opt in ("-s"):
        logLine(os.path.basename(__file__) + ' background monitor option -s to not start the system.')
        opt_autostartSystem = 0
    elif opt in ("-r"):
        logLine(os.path.basename(__file__) + ' background monitor option -r for no forced reboot delay.')
        opt_forcedRebootDelay = 0
    elif opt in ("-h"):
        printUsage()
        logLine(os.path.basename(__file__) + ' background monitor exiting due to just help selection.')
        sys.exit()
# ...


# Remove the need to stop and reboot as we only recognize that after startup
if os.path.isfile(stopAndRebootCmdFile):
    logLine('Remove existing stop and reboot command file')
    os.system('rm ' + stopAndRebootCmdFile + ' 2> /dev/null');

if os.path.isfile(forcedRebootCmdFile):
    logLine('Remove existing forced reboot command file')
    os.system('rm ' + forcedRebootCmdFile + ' > /dev/null 2> /dev/null');

# Configure Pi GPIO lines used to read front panel switches or Seismo2 Board jumpers
for gpioSwitch in allSwitches:
    GPIO.setup(gpioSwitch, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Configure GPIO used to drive the front panel LEDS
for gpioLed in allLeds:
    GPIO.setup(gpioLed, GPIO.OUT)

# An initial set of quick blinks just so we know monitor is going to run
setAllLeds( ledOff )
blinkLed( statusLed, 4, 0.1, ledOff )

# ------------------------ Check switches for display test or exit  --------------------

# if the force display test file is present or we have an accessSwitch and it is held down do continual display test
# display test is stopped by hitting reset key and we do full system shutdown in that case
if (os.path.isfile(displayTestCmdFile) > 0) or (not readSwitch(accessSwitch)):
    for gpioLed in allLeds:
        GPIO.output(gpioLed, 1)
    testLeds = allLeds
    while True:
        logLine("System display test running a cycle of LED tests. Press RESET to shutdown unit.")
        for led in testLeds:
            if readSwitch(resetSwitch) == 0:
                logLine("System will be shutdown due to reset key pressed. Ok to shut off power in 30 sec")
                blinkLed( shutdownLed, 8, 0.15, ledOff )
                os.system('shutdown -h now')
            # To exit the script if in display mode use the stop_system.now file. Stop file will be be deleted
            if os.path.isfile(stopSystemCmdFile) > 0:
                os.system('rm ' + stopSystemCmdFile)
                blinkLed( statusLed, 6, 0.15, ledOff )
                quit()
            blinkLed( led, 2, 1.0, ledOff )
            time.sleep(1.0)
            blinkAllLeds(2, 1.0, ledOff)
        time.sleep(1.0)
        blinkAllLeds(2, 1.0, ledOff)

# If autostart is allowed and the special file to disable autostart is not found then we have a few ways to autostart
#  Use of defined and installed autostartJumper connected to ground does autostart
#  if opt_autostartSystem is 1 and  MODE or ACCESS are not pressed
#
# In any case if the noAutoStartCmdFile is present we do not autostart in any case
# If we do the autostart then blink quickly the status led a bunch more times
if (readSwitch(autostartJumper) == 0) or ((opt_autostartSystem > 0) and (readSwitch(modeSwitch) == 1 and  readSwitch(accessSwitch) == 1)):
        if not os.path.isfile(noAutoStartCmdFile):
            logLine(os.path.basename(__file__) + ' starting the system now')
            setAllLeds( ledOff )
            blinkLed( statusLed, 6, 0.2, ledOff )
            os.chdir(scriptPath)
            startSystemCode()
        else:
            logLine("System will not be started due to no autostart file or option -s.")
            setAllLeds( ledOff )
            blinkLed( statusLed, 3, 0.1, ledOff )

# --------------------------  mainloop: Background Loop  -------------------------------
procCheckTimer = 0

# Wrap to catch exceptions such as KeyboardInterrupt so we can indicate in log when done
try:
    keepGoing=1

    # Start of the main while loop ==============================================================

    while keepGoing == 1:

        # ----------------  Check Special Files Or Switch To Exit Monitor  ----------------------
        # If accessSwitch and mode held down abort THIS monitor
        if readSwitch(accessSwitch) == 0 and  readSwitch(modeSwitch) == 0:
            logLine(os.path.basename(__file__) + ' exiting due to select and mode both pressed.')
            blinkLed( errorLed, 4, 0.1, ledOff )
            sys.exit(0)

        if os.path.isfile(exitMonitorCmdFile):
            os.system('rm ' + exitMonitorCmdFile)
            logLine(os.path.basename(__file__) + ' exiting due to file ' + exitMonitorCmdFile)
            blinkLed( errorLed, 4, 0.1, ledOff )
            sys.exit(0)

        # ------------  Check Speical Files For Shutdown Or Firmware Load  ----------------------

        # ----------------------------    Stop the system   -------------------------------------
        if os.path.isfile(stopSystemCmdFile):
            os.system('rm ' + stopSystemCmdFile)
            stopSystemWithDelay(5.0)

        # -------------------  Check For need to stop or Restart   ------------------------------
        try:
            procCheckTimer += 1
            if (procCheckTimer > 100) and not os.path.isfile(noAutoRestartCmdFile):
                checkProcesses(procCheckTimer)
                procCheckTimer = 0
        except:
            logLine('Exception in process maintenance.')
            procCheckTimer = 0
            allProcessesRunning = True

        # Act on the stop and reboot file do it unless access switch is down then disable this cycle
        # If we want to restart script we could use:    os.execv(sys.executable, ['python'] + sys.argv)
        if os.path.isfile(stopAndRebootCmdFile):
            os.system('rm ' + stopAndRebootCmdFile)
            if readSwitch(accessSwitch) == 1:
                logLine("System will be stopped and system restarted now due to stop and reboot file.")
                stopSystemWithDelay(5.0)
                setAllLeds( ledOff )
                blinkLed( errorLed, 10, 0.3, ledOn )
                os.system("sudo shutdown -r now")
                sys.exit(0)

        # Act on the forced reboot file unless it has been too soon since the start of this script.
        # The intention of this sort of reboot is to command a reboot from the backend as debug
        if os.path.isfile(forcedRebootCmdFile):
            if (time.time() - scriptStartTime) > opt_forcedRebootDelay:
                logLine("System is being commanded to force a stop and then reboot.")
                os.system('rm ' + forcedRebootCmdFile);
                stopSystemWithDelay(5.0)
                setAllLeds( ledOff )
                blinkLed( errorLed, 10, 0.3, ledOn )
                os.system("sudo shutdown -r now")
                sys.exit(0)


        # Reset switch can do a reset or a full shutdown depending on how long it is held.
        # - Hold for 2 seconds and errorLed will blink a few times fast.
        #   Release as it blinks fast and we reboot
        # - Continue to hold for 3 seconds more and a full halt happens
        # If we see reset pressed for at least a couple seconds we also stop LED updates for 15 seconds
        if readSwitch(resetSwitch) == 0:
            time.sleep(2.0)
            if readSwitch(resetSwitch) == 0:
                os.system('touch ' + suspendLedUpdatesFile + ' 2> /dev/null');
                time.sleep(1.0)
                setAllLeds( ledOff )
            if readSwitch(resetSwitch) == 0:
                logLine("System will be rebooted if reset key released or shutdown if reset held longer")
                blinkLed( shutdownLed, 8, 0.1, ledOff )
                setAllLeds( ledOff )
                time.sleep(2.0)
                if readSwitch(resetSwitch) == 0:
                    logLine("System will be shutdown and halted due to reset switches held down long time.")
                    blinkLed( shutdownLed, 3, 0.6, ledOff )
                    stopSystemCode()
                    logLine("System code has been stopped. Warn then do halt")

                    # now system should be off so turn off all leds
                    # then blink status led many times leaving it OFF for halt
                    setAllLeds( ledOff )
                    blinkLed( shutdownLed, 5, 0.6, ledOff )

                    os.system("sudo shutdown -h now")
                    keepGoing = 0
                else:
                    logLine("System will be restarted since reset was held only a few seconds.")
                    blinkLed( shutdownLed, 4, 0.25, ledOff )
                    stopSystemCode()
                    logLine("System code has been stopped. Warn then do reboot")

                    # now system should be off so turn off all leds
                    # then blink status led a few times leaving it ON for restart
                    setAllLeds( ledOff )
                    blinkLed( shutdownLed, 8, 0.25, ledOff )
                    os.system("sudo shutdown -r now")
                    keepGoing = 0

        time.sleep(0.1)

    # End of the main while loop   ==============================================================

except KeyboardInterrupt:
    logLine(os.path.basename(__file__) + ' keyboard exception. background monitor done.')
    raise
except:
    logLine(os.path.basename(__file__) + ' exception or program exit. background monitor done.')

sys.exit(0)
