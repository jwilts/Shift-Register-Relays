#!/usr/bin/env python

"""
shiftreg.py
I use this to control up to 8 relays off of 3 PINS on either a PICO or a Raspberry PI.
Utilize threading in any application that calls this to make sure you do not create a blocking condition.
I use SN74HC595N shift registers
Shift register control module.  Provides the :class:`Shifter` class which
makes it easy to control shift registers.
I use this in my CubCar (Pinewood Derby) timers to control starting servos in the lanes.
.. note::
    The default pin values are 16, 20, and 21 for data (DS), latch (ST_CP), and
    clock (SH_CP).
"""

import RPi.GPIO as GPIO
import time
from threading import Timer

# Define constants for relay states
HIGH = True
LOW = False

# Global variable to store the states of each relay
relay_states = 0b00000000

class Shifter(object):
    """
    A class for controlling one or more shift registers.  When instantiated you
    can choose which pins to use for the *data_pin*, *latch_pin*. and,
    *clock_pin*.
    Optionally, if the *invert* keyword argument is ``True`` then all bitwise
    operations will be inverted (bits flipped) before being sent to the shift
    register.  This is a convenient way to invert things like LED matrices and
    deal with relays that are "active low" (i.e. all relays are on when the
    pins are low instead of high).
    .. note::
        You must ensure that the numbers you pass for the pins match the mode
        you're using with `RPi.GPIO`.
    """
        
    def __init__(self, data_pin=16, latch_pin=20, clock_pin=21, invert=False):
        """
        Initialize the Shifter object with default pin numbers and GPIO setup.
        """
        self.data_pin = data_pin
        self.latch_pin = latch_pin
        self.clock_pin = clock_pin
        self.invert = invert
        GPIO.setup(self.latch_pin, GPIO.OUT)
        GPIO.setup(self.clock_pin, GPIO.OUT)
        GPIO.setup(self.data_pin, GPIO.OUT)

    def shift_out(self, *values):
        """
        Shifts out an arbitrary number of *values* which should be integers
        representing binary values for the shift register.
        """
        global relay_states
        bits = {"0": False, "1": True}
        if self.invert:
            bits = {"1": False, "0": True}
        GPIO.output(self.latch_pin, LOW)
        for val in reversed(values):
            for bit in '{0:08b}'.format(val):
                GPIO.output(self.clock_pin, LOW)
                GPIO.output(self.data_pin, bits[bit])
                GPIO.output(self.clock_pin, HIGH)
        GPIO.output(self.latch_pin, HIGH)
        relay_states = values[-1]  # Update relay_states

    def update_relay(self, relay_number, state, pulse_time=None):
        """
        Update the state of a specific relay. Optionally, pulse the relay for a given duration.
        """
        global relay_states
        if state:
            relay_states |= (1 << relay_number)
        else:
            relay_states &= ~(1 << relay_number)
        self.shift_out(relay_states)
        
        # If pulse_time is provided, start a timer to revert the relay state after the given time
        if pulse_time is not None:
            Timer(pulse_time, self.update_relay, args=(relay_number, not state)).start()

    def query_relay(self, relay_number):
        """
        Query the state of a specific relay.
        """
        global relay_states
        state = bool(relay_states & (1 << relay_number))
        return state

    def test(self):
        """
        Performs a test of the shift register by setting each pin HIGH for .25
        seconds then LOW for .25 seconds in sequence.
        """
        for i in range(8):
            self.shift_out(1 << i)
            time.sleep(0.25)
            self.shift_out(0)
            time.sleep(0.25)

    def all(self, state=LOW):
        """
        Sets all pins on the shift register to the given *state*. Can be either HIGH or LOW (1 or 0, True or False).
        """
        if state:
            self.shift_out(0b11111111)
        else:
            self.shift_out(0)

    def get_all_states(self):
        """
        Returns the states of all relays as a binary string.
        """
        global relay_states
        return bin(relay_states)[2:].zfill(8)  # Convert to binary string and ensure it's 8 characters long


if __name__ == "__main__":
    print("Testing shift register connection...")
    print("Each pin should go HIGH and LOW in order until you press Ctrl-C.")
    
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
       
    s = Shifter()
    try:
        while True:
            print("\nSetting all relays to HIGH...")
            s.all(HIGH)
            print("\nGetting state of all relays...")
            all_states = s.get_all_states()
            print("All relay states:", all_states)
            time.sleep(3)

            print("\nSetting all relays to LOW...")
            s.all()
            time.sleep(3)

            print("\nSetting relay 0 and 1 to HIGH with pulse for 2 seconds...")
            s.update_relay(0, True)
            s.update_relay(1, True, pulse_time=2)  # Pulse relay 1 for 2 seconds
            all_states = s.get_all_states()
            print("All relay states:", all_states)
            time.sleep(1)

            print("\nSetting relay 0 to LOW...")
            s.update_relay(0, False)
            time.sleep(1)

            print("\nQuerying state of relay 1...")
            state = s.query_relay(1)
            print("Relay 1 state:", state)
            print("\nQuerying state of relay 0...")
            state = s.query_relay(0)
            print("Relay 0 state:", state)

            print("\nGetting state of all relays...")
            all_states = s.get_all_states()
            print("All relay states:", all_states)

            print("\nTesting shift register...")
            s.test()

            print("\nGetting state of all relays...")
            all_states = s.get_all_states()
            print("All relay states:", all_states)
    except KeyboardInterrupt:
        print("\nCtrl-C detected. Quitting...")
