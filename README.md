# Closed-Loop Model Train Closed Loop Speed Control Using Bidirectional Dshot
## Summary
This project uses ATMega32u4 'pro micro' processor to achieve closed loop speed control of a G-scale model train. The motor is a [5010 360KV outrunner](https://www.amazon.com/dp/B0C2YRNX1Z) driven by a [LANRC 35A 2S-6S BLHeli_S ESC](https://www.amazon.com/dp/B0F8P985VQ?ref_=ppx_hzsearch_conn_dt_b_fed_asin_title_1) flashed to Bluejay 0.21.0 w/ 98kHz PWM. Supply voltage is monitored by the ATMega ADC using a voltage divider w/ a capacitor for low-pass filtering. Speed, direction, and mode (open vs. closed loop) are communicated by a standard RC radio. The ESC provides feedback to the system via bidirectional dshot, reporting the **period** of the most recent commutation cycle (1/ERPM). The main control algorithm uses a velocity-target feed-forward, proportional and integral control, then a simple voltage compensation algorithm (compensated=output*v_nom/v_actl).

## Background
The 1999 Lionel Holiday Special train set originally ran on an RS-385-class brushed motor directly from rail voltage. This ancient mid-grade holiday novelty had many issues over the years as its load grew from pulling a single gondola car and caboose around a tree in a four-foot circle to pulling a dozen Christmas Boxcars around the perimeter of two rooms. Over the years, various upgrades were attempted by my father and myself, including periodically repairing the worn-out metal-on-plastic 'bushings' with cast-in-place JB-Weld, loading down the engine with lead shot, steel, and brass, and attempting to improve electrical connection to the track (including replacing the brushes).

Robust to any of these attempted fixes was its electrical connection issues. It would often get suck or violently surge in power as it gained and lost connection due to its 0-6-0 layout. Using parts I had laying around, I converted it to remote control. The rails would now provide constant DC from a standard switch-mode supply then the the gondola car behind the locomotive would contain an RC receiver with a brushed motor speed controller. This enabled the addition of a 1F 27V ultracapacitor pack to smooth out the spikes of rail voltage through the faulty pickups. Unfortunately, the ultracap requires care in charge and discharge, so simply throwing it straight across the rails would be unwise. Instead, a large power resistor and some 10-amp Schottky diodes were fitted. One diode ensured input polarity was correct, then the other diode was wired opposite the resistor. The capacitor could drain to the ESC through the diode, while it had to charge through the resistor. When the rails were making contact, only a single diode drop was wasted.

However, this caused another issue. Around this time, I had changed room configurations and necessitated the addition of extra small turns. Because I was cheap and new the train had some energy storage capacity, I 3D-printed rail sections. While these rails routed a through-wire soldered between the previous brass rail and the next one, the turn itself was not conductive, so the train slowed audibly while passing the junction. This was annoying.

Maybe three steps ago, a wise person would perhaps try to buy a locomotive. Unfortunately, the G in G-scale stands for Garden. Or maybe Grand, which wouldn't be a crazy price for a loco. Said fancy locos, to the first meaning of G, are meant for very large sweeping turns, not these two-foot radii. A more reasonable solution would be to just chuck a battery in there, but I'm not a reasonable person.

After all this faffing about, the train started to run for long enough without getting stuck that I noticed a new issue: the motor quickly overheated. I originally tried replacing the motor with a Mabuchi RS-385PH-2465 for the 2023 season, but it had the same struggle. The next plan for the 2025 season was to purchase the above-mentioned brushless motor and o-ring-belt it to backshaft of the brushed motor via [3D-printed pulleys](https://cad.onshape.com/documents/4253d626caec1bce4acd5581/w/97daf85e286b58ed47cc3d8e/e/a23712bbd4e7cdbac714f3cb). This worked quite well but was still not without issue. The slowing-down-when-losing contact issue was no longer just an annoying slowdown, but a very noticeable change in coil whine. Also, my desired operation speed was so slow that it would noticeable change speed on straight vs. complex areas of the track layout. A more complicated issue was that the ESC would violently regen brake when decelerating, overvolting the ultracapacitor.

I had hoped that the stock BLHeli_S firmware would allow disabling this 'damped light' (I have no clue where that term comes from). Unfortunately, that is not possible without Bluejay, which requires DSHOT communication instead of the terrible classic RC PWM.

For the 2026 season, I decided to actually try Bluejay and, since I had to add a micro in the loop anyway, why not use bidirectional DSHOT to do actual closed-loop speed control. I was originally just going to use an Arduino and this project that I forked here, but was unable to use the 'arduino 4-way-interface' projects to talk to esc-configurator. Wanting to move away from my university-induced preference for the obsolete Stellaris/TI Tiva series of Cortex M4F microprocessors, I bit the bullet and ordered an ST NUCLEOG474RE. This board is supported (with a bunch of screwing around) by Betaflight and would work as a bridge to ESC-Configurator. With that nice wonderful 32-bit micro in the party, I was going to try to hack my control algorithm into the open-source Betaflight firmware, but that was not working too well. One odd finding was that, at least using the GUI because I could never get RC PWM input to work, it would aggressive brake when neutral throttle was requested, instead of coasting as the Arduino DSHOT implementation would. Perhaps it is not sending the appropriate zero-speed signals (although I probably ain't either). I instead decided to perform some kind of ascetic exercise programming for 8-bit AVR. At least it has two-cycle multiply!

## Hardware Resources

### I/O
The most critical I/O for this project is the DSHOT pin. This digital pin is used as a plain digital output to bitbang the DSHOT signal, then in input capture mode to time the pulses of the feedback signal. The only input capture pin routed to the 'pro micro' footprint is PD4, with Timer 1.

The R/C PWM I/O is achieved through the external interrupt pins. Unlike the pin-change-interrupt available for most pins, the special external interrupt pins go to their own unique ISR. Of course, the right way to do this is with input capture, but the only other available input capture pin is not routed on the pro micro. Also, two R/C channels are required, one throttle channel (channel 3) which uses throttle map tables to send either 50-100 or 50-0 for easy bidirectional control and a second 'gear' switch channel (channel 5) to switch between closed-loop and open-loop mode. Using the PCINT means that interrupt service latency is critical, as one microsecond latency corresponds to 0.2% of signal. It was observed that even after turning off TIMER0 (the one used for Arduino timing), the USB controller still caused periodic latency spikes of a few microseconds. Without the USB connection, the glitches went away.

The final functional I/O for this project is the ADC, operating in single-ended free-running mode against the 2.56V internal reference. The chosen resistor dividers result in full-scale approximately matching the nominal system voltage, so a simple left-shift is all that is required for the multiplication side of the compensation formula.

The RXLED is flashed to indicate various statuses.

Several pins are twiddled back and forth for debugging with Pulseview, but they are not used functionally.


### Timers
TIMER0 is disabled to prevent spurious Arduino interrupts, TIMER3 is unused, TIMER1 is used for the DSHOT input capture, and TIMER2 is used both as a replacement for TIMER0 to orchestrate the main-loop rate as well as the time references the RC PWM interrupts record. Since TIMER2 runs at 2 ticks per microsecond, the elapsed timer ticks correspond directly to the 0-1000 throttle value sent over DSHOT (midpoint to full throttle is 1500us to 2000us, thus 1000 counts).

## Software Architecture
The original project from which this is forked used serial I/O on an Uno for a complex user interface. This user interface ran independently of the critical DSHOT transaction, which ran the entire 100us procedure in a timer ISR. It quickly became apparent that running the main loop asynchronously from the DSHOT transaction would not be ideal for this project. The time spent copying data in and out of mutexes was becoming overwhelming. Since I was willing to be very careful about my computational complexity, I could instead run the main loop synchronously with the DSHOT transactions in a very Arduino way (not delayMicroseconds, but just spin-waiting for elapsed timer ticks). The DSHOT transaction still takes place within a critical section, but no data integrity protections are required.

The R/C PWM input is, naturally, asynchronous to the main control loop. Since I can't use input capture and measurement accuracy is so important (since it is only sending a 1.5ms pulse on a 20ms period), I had to add some weird features. The DSHOT critical section is definitely really critical, it's having to send really tight transitions with sub-microsecond timing. Any failed dshot transaction would harm the PID loop performance. So, the RC PWM system intead takes a back seat. Whenever the DSHOT critical section ends, it sees if an RC edge has not been serviced, and, if there is one, it throws it away and resets the measurement cycle. Speaking of throwing away, any rising edge captured at a timer count of exactly is thrown away, because I was too lazy to implement a second flag. Since the timer overflows slower than the RC signal is sent, a missed edge doesn't require too much extra recording aside from setting the start-count register to 0. If a rising edge is skipped, the falling edge will do nothing, since it sees the start count was 0. If a falling edge is missed, the next rising edge will reset the count to the next edge time. If somehow the next rising edge is *also* missed, the elapsed time will be around 20 milliseconds (40,000 counts), which will be rejected before being sent to the filter.

Data transfer was fun. It may not be immediately obvious, but sending an ASCII base-10 integer over serial is incredibly expensive computationally. Sending "65535" requires almost a half-dozen INT32 divisions/modulos. Since Arduino doesn't provide any nice system for sending raw bytes, I instead created my own with a companion Python script that reads the data transfer structure from the C code upon launch. The raw bytes are simply chucked onto the wire, delimited by a literal comma. At the end of the packet, a sequence of five zeros are sent. Since all the real data is one, two, or four bytes of data followed by a comma, five zeros is not possible. Thus, Python can read the frame until it sees that. This took the data transfer from several hundred microseconds to 10-20 (note the 32u4 always screams USB CDC serial packets as fast as it can, no FTDI to talk slow to). It is important to note that the current implementation is kinda dumb in that it splits the input string by commas, which could cause extra readings if data value happens to be the numerical value of comma. Should probably fix that.





# Original Docs
# Arduino bi-directional DShot
> Bi-directional Dshot300 on an Arduino.

This is an implementation of DShot 300 on an Arduino. It can transmit (uninverted) DShot and inverted Dshot in order for the ESC to respond with a telemetry packet.

## Motivation
The motivation behind this project is to have an easy way to debug DShot communication with an ESC, especially bi-directional DShot. This is not meant as a library to use Dshot on your Arduino but is rather intended for testing purposes.

Apart from that I was interested to see how far I can push an Arduino when it comes to very timing sensitive tasks without resorting to assembler, and I must say, I was very surprised how well this worked out. Although some tricks (like directly reading ports instead of digitalRead) have been used.

> Yes - it is definitely possible to have a better optimized version in assembler. I want this to be easily understandable so I can come back to it in a couple of years and still understand what is happening.

## Limitations
The Arduino as such is very limited in regards to receiving bi-directional packages since they come in very short burst - on Dshot 300 the response frame arrives in less than 60us and can effectively only be processed via pin D8 - which is the "Input Capture" pin.

D8 is responsible for sending the DShot signal and receiving the response package.

### Manipulation
The Dshot value sent by default is **0**. Via serial console other values (up to 2047) might be provided.

> Be aware that due to the nature of Dshot frames being sent continuously it might happen that serial Input is not processed immediately or only chunks arrive. There is not much that can be done which would not involve messing with the timing of the DShot frames being sent/received.

### Extended Dshot
If extended Dshot is enable you can activate it by sending command 13. This will then print all available Extended Dshot telemetry values:

* Temperature
* Voltage
* Current
* Debug Value 1
* Debug Value 2
* Debug Value 3
* State flag

> Be aware that not all telemetry values are available on all ESCs. BLheli ESCs on [Bluejay](https://github.com/bird-sanctuary/bluejay) will only display temperature by default.

## How does it work?
A timer triggers sending of the DShot frame via interrupt then the following things happen:

1. D8 is set as output
2. Dshot frame is sent
3. D8 is set as input
4. Input capture on timer 1 is used to capture the time between falling and rising edges
5. Conversion of the captured data to 21 bit response message
6. Decoding of response message
7. Serial print of decoded response message

## Usage
I recommend you use Visual Studio Code with the platform IO plugin. A **platformio.ini** is included in the repository with a working serial monitor configuration.

## C2 Interface
This sketch also includes a C2 interface for your convenience, allowing to read, write and erase an EFM8 MCU. This will require you to install the EFM8 python client. Also by default the Arduino will start in DShot mode, to enable the C2 interface, pull pin 13 low and reset the arduino.

```
D  Pin 2
CK Pin 3
```

To switch back to DShot mode, remove the jumper to low form pin 13 and reset the arduino.

### Installing python EFM8 client
use pipenv to create an virtual env, install dependencies and enter the env:
```
pipenv install
pipenv shell
```

then run the client like so (assuming `/dev/ttyUSB0` is your Arduino):

```
python efm8.py read /dev/ttyUSB0 output.hex
python efm8.py write /dev/ttyUSB0 input.hex
python efm8.py erase /dev/ttyUSB0
```

## Compatibility
This should work on any Arduino Uno or clone running at 16MHz. If you are using a different Arduino board, make sure you are using (and setting) the pin which has the "Input Capture" functionality.

## Resources
* [SiLabs C2 Application note](https://www.silabs.com/documents/public/application-notes/AN127.pdf)
* [DShot - the missing handbook](https://brushlesswhoop.com/dshot-and-bidirectional-dshot/)

## Contributions
Contributions are very welcome. Feel free to submit a PR against the develop branch. If you have any questions or feature requests, feel free to [open an issue](https://github.com/bird-sanctuary/arduino-bi-directional-dshot/issues).
