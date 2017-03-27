Shoes Software
==============

The shoes software is a BLE app for the Nordic nRF51822. It is not a typical
nRF BLE app because it ignores most of the nRF5x SDK and operates essentially right
on top of the raw radio.

Software Overview
-----------------

This app is designed to implement a flooding protocol based on BLE advertisements.
When the accelerometer detects a "stomp" like motion it starts a flood. The
flood starts with a BLE advertisement sent to all neighboring nodes.
This packet is transmitted several times and includes a timestamp so that
the receivers can keep synchronized. When a receiver hears a packet,
it waits until the next transmission window and re-transmits the packet
several times to all of its neighbors. As nodes hear the flood packet
they update their LEDs and you should be able to see the flood propagate.

Time Slicing with the Softdevice
--------------------------------

The Nordic softdevice includes an API for getting time slices where the softdevice
(the BLE stack provided by Nordic) gives up control of the radio and gives the
application exclusive access to the radio. The app uses this to be able to freely
send and listen for BLE advertisements without the softdevice intruding.
The API does require that the softdevice gets _some_ time to use the radio, so
we do ask for the longest slice we can but do give some portion of time to the
softdevice.

Other Info
----------

The software is reasonable well commented for what each portion is doing.

The button can be used to control which color flood that particular shoe
initiates.


Compiling and Programming
-------------------------

Running `make` should build the app. If make fails, ensure you have the submodule
checked out.

    $ make

To program the app, follow [these instructions](https://github.com/lab11/nrf5x-base#program-a-nrf51822)
and set the ID (where the last octet changes):

    $ make flash ID=c0:98:e5:b0:00:01




