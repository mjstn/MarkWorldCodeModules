#
# This is a simplified version of keyboard reader class for 
# the Rii i8S Mini Keyboard  RF dongle keyboard
#
# for test you can just run it but normally another python app 
# imports this and uses the read_key() method awaiting non zero len string
# To use this you must have installed evdev: sudo pip3 install evdev
#
import evdev
from subprocess import call
import atexit
import time
import sys
from selectors import DefaultSelector, EVENT_READ
import threading

class miniRfKeyboard:
    def __init__(self):
        self.selector = DefaultSelector()
        self.mouse = evdev.InputDevice('/dev/input/event1')
        self.keybd = evdev.InputDevice('/dev/input/event0')
        self.sysctrl = evdev.InputDevice('/dev/input/event2')
        self.conctrl = evdev.InputDevice('/dev/input/event3')
        self.readingKeys = False
        self.dbgPrint = 0
        atexit.register(self.keybd.ungrab)  # Don't forget to ungrab the keyboard on exit!
        atexit.register(self.mouse.ungrab)
        self.keybd.grab()  # Grab, i.e. prevent the keyboard from emitting original events.#
        self.mouse.grab()
        # This works because InputDevice has a `fileno()` method.
        self.selector.register(self.mouse, EVENT_READ)
        self.selector.register(self.keybd, EVENT_READ)
        self.selector.register(self.sysctrl, EVENT_READ)
        self.selector.register(self.conctrl, EVENT_READ)


    def read_keys_loop(self):
        print("Starting keyboard loop. Use Ctrl-C to quit")
        self.readingKeys = True
        #signal ready
        newKey = ''
        while self.readingKeys:
            newKey = self.read_key()
            if newKey != '':
                if self.dbgPrint != 0:  print("New Key: ", newKey)

    # Read for a key. If no key found return empty string
    def read_key(self):
        keyChar = ''
        for key, mask in self.selector.select(0.05):
            device = key.fileobj
            for event in device.read():
                if event.type == evdev.ecodes.EV_KEY:
                    if self.dbgPrint != 0: print("event val: " + str(event.value))
                    #print(device)
                    #print(evdev.ecodes.bytype[evdev.ecodes.EV_KEY][event.code])
                    keyChar = self.key_decode(event, device)
                else:
                    pass
                    # print(event)
            return keyChar

    def key_decode(self, ev, dev):
        keyCode = ''
        # EVENTS CALLED ON PRESS AND ON HOLD
        if ev.value == 1 or ev.value == 2:
            if ev.value == 2:
                #flush the buffer
                while dev.read_one() is not None:
                    pass

            if ev.code == evdev.ecodes.KEY_A:            keyCode = 'A'
            elif ev.code == evdev.ecodes.KEY_B:          keyCode = 'B'
            elif ev.code == evdev.ecodes.KEY_C:          keyCode = 'C'
            elif ev.code == evdev.ecodes.KEY_D:          keyCode = 'D'
            elif ev.code == evdev.ecodes.KEY_E:          keyCode = 'E'
            elif ev.code == evdev.ecodes.KEY_F:          keyCode = 'F'
            elif ev.code == evdev.ecodes.KEY_G:          keyCode = 'G'
            elif ev.code == evdev.ecodes.KEY_H:          keyCode = 'H'
            elif ev.code == evdev.ecodes.KEY_I:          keyCode = 'I'
            elif ev.code == evdev.ecodes.KEY_J:          keyCode = 'J'
            elif ev.code == evdev.ecodes.KEY_K:          keyCode = 'K'
            elif ev.code == evdev.ecodes.KEY_L:          keyCode = 'L'
            elif ev.code == evdev.ecodes.KEY_M:          keyCode = 'M'
            elif ev.code == evdev.ecodes.KEY_N:          keyCode = 'N'
            elif ev.code == evdev.ecodes.KEY_O:          keyCode = 'O'
            elif ev.code == evdev.ecodes.KEY_P:          keyCode = 'P'
            elif ev.code == evdev.ecodes.KEY_Q:          keyCode = 'Q'
            elif ev.code == evdev.ecodes.KEY_R:          keyCode = 'R'
            elif ev.code == evdev.ecodes.KEY_S:          keyCode = 'S'
            elif ev.code == evdev.ecodes.KEY_T:          keyCode = 'T'
            elif ev.code == evdev.ecodes.KEY_U:          keyCode = 'U'
            elif ev.code == evdev.ecodes.KEY_V:          keyCode = 'V'
            elif ev.code == evdev.ecodes.KEY_W:          keyCode = 'W'
            elif ev.code == evdev.ecodes.KEY_X:          keyCode = 'X'
            elif ev.code == evdev.ecodes.KEY_Y:          keyCode = 'Y'
            elif ev.code == evdev.ecodes.KEY_Z:          keyCode = 'Z'

            elif ev.code == evdev.ecodes.KEY_UP:         keyCode = 'UP'
            elif ev.code == evdev.ecodes.KEY_DOWN:       keyCode = 'DOWN'
            elif ev.code == evdev.ecodes.KEY_LEFT:       keyCode = 'LEFT'
            elif ev.code == evdev.ecodes.KEY_RIGHT:      keyCode = 'RIGHT'
            elif ev.code == evdev.ecodes.KEY_VOLUMEUP:   keyCode = 'VOLUP'
            elif ev.code == evdev.ecodes.KEY_VOLUMEDOWN: keyCode = 'VOLDOWN'
            elif ev.code == evdev.ecodes.KEY_1:          keyCode = '1'
            elif ev.code == evdev.ecodes.KEY_2:          keyCode = '2'
            elif ev.code == evdev.ecodes.KEY_3:          keyCode = '3'
            elif ev.code == evdev.ecodes.KEY_4:          keyCode = '4'
            elif ev.code == evdev.ecodes.KEY_5:          keyCode = '5'
            elif ev.code == evdev.ecodes.KEY_6:          keyCode = '6'
            elif ev.code == evdev.ecodes.KEY_7:          keyCode = '7'
            elif ev.code == evdev.ecodes.KEY_8:          keyCode = '8'
            elif ev.code == evdev.ecodes.KEY_9:          keyCode = '9'
            elif ev.code == evdev.ecodes.KEY_F1:         keyCode = 'F1'
            elif ev.code == evdev.ecodes.KEY_F2:         keyCode = 'F2'
            elif ev.code == evdev.ecodes.KEY_F3:         keyCode = 'F3'
            elif ev.code == evdev.ecodes.KEY_F4:         keyCode = 'F4'
            elif ev.code == evdev.ecodes.KEY_F5:         keyCode = 'F5'
            elif ev.code == evdev.ecodes.KEY_F6:         keyCode = 'F6'
            elif ev.code == evdev.ecodes.KEY_F7:         keyCode = 'F7'
            elif ev.code == evdev.ecodes.KEY_F8:         keyCode = 'F8'
            elif ev.code == evdev.ecodes.KEY_F9:         keyCode = 'F9'
            elif ev.code == evdev.ecodes.KEY_F10:        keyCode = 'F10'
            elif ev.value == 2: pass


            # PROG FUNCTIONS
            elif ev.code == evdev.ecodes.KEY_LEFTMETA: self.close()
            elif ev.code == evdev.ecodes.KEY_END: self.shutdown_pi()
            elif ev.code == evdev.ecodes.KEY_SYSRQ: self.reboot_pi()

            else:
                print("UNUSED KEY CODE")
                print(evdev.ecodes.bytype[evdev.ecodes.EV_KEY][ev.code])
        # flush backed up key presses
        while dev.read_one() != None:
            pass
        return keyCode


    def close(self):
        time.sleep(0.2)
        self.readingKeys = False
        self.selector.unregister(self.mouse)
        self.selector.unregister(self.keybd)
        self.selector.unregister(self.conctrl)
        self.selector.unregister(self.sysctrl)
        self.led.colorWipe(self.led.strip, Color(0,0,0),0)
        # kbd should be ungrabbed by atexit
        # but belt and braces
        try:
            self.keybd.ungrab
            self.mouse.ungrab
        except:
            pass
        sys.exit()

    def shutdown_pi(self):
        call("sudo nohup shutdown -h now", shell=True)

    def reboot_pi(self):
        call("sudo nohup reboot", shell=True)

if __name__ == '__main__':
    kb = miniRfKeyboard()
    try:
        kb.read_keys_loop()
    except KeyboardInterrupt:
        print("calling close")
        kb.close()
