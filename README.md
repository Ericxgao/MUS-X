# MUS-X

Modules for VCV Rack, with a focus on MIDI-controllable, analog poly-synths, and per-voice variance.

## ADSR
ADSR envelope generator with exponential decay/release, built in velocity scaling and sustain CV.

'Rnd' randomly scales the A, D, S and R parameters per channel. This simulates the behavior of old polysynths with analog envelopes, where each voice/envelope behaves slightly different due to component tolerances. The offsets are individually generated for each ADSR module, and stored with the patch.

During the decay and release phase, a gate signal is output. This can e.g. be used to trigger another envelope generator, that shapes the sustain.

## Delay
A delay inspired by analog bucket-brigade delay pedals.

* 'Time' adjusts the delay time. The range is determined by the 'BBD size' parameter.
The delay time can also be tapped with the 'Tap' button. The delay time is also visualized by the 'Tap'-button LED.
  'Time' can be CV controlled.
* 'Feedback' adjusts the delay feedback. WARNING: this goes well beyond 100% and can cause self oscillation. The output is limited to ±10V.
'Feedback' can be CV controlled.
* 'Cutoff' adjusts the cutoff frequency of the anti-aliasing and reconstruction filters. It makes the delay brighter or darker. The aliasing of the BBD can be audible at higher cutoff frequencies.
* 'Resonance' adjusts the resonance of the anti-aliasing and reconstruction filters. This alters the character of the delay, especially at high feedback amounts.
* 'Noise' adjusts the noise of the BBD line. More noise will result in self oscillation at high feedback levels.
* 'BBD size' adjusts the number of stages of the BBD line. A smaller size can be used for very short delays, or a chorus effect. Many famous hardware units use 2048 or 4096 stages.
A larger BBD size can give longer delay times, and a brighter delay without aliasing.
* 'Poles' adjusts the slope of the anti-aliasing and reconstruction filters from 6 dB/Oct to 24 dB/Oct. A steeper slope gives a 'cleaner', 'duller' sound, a more shallow slope sounds 'dirtier' (with more audible aliasing), but brighter.
* 'Comp' adjusts the reaction time of the compander. Lower values will give the repeats a slight 'fade in' and 'fade out', and can give the delay a 'dreamy' character. This value also slightly affects the low-frequency response of the delay.

* 'Input' adjusts the input level to the delay. It does not affect the dry signal output. A small red LED indicates an overload of the delay circuit and will result in saturation.
* Turning up the 'Stereo' parameter gives a ping-pong delay. It has no effect when 'Inv R' is enabled.
* If 'Inv R' is enabled, the right wet signal will be the inverted left wet signal. You can use this with a small BBD size (e.g. 512), no feedback, and delay time modulation to create a chorus effect.
* 'Mix' adjusts the dry-wet balance.

## Drift
Drift generates subtle constant offset and drift.
The 'Poly' input determines the polyphony channels of the output.
* 'Const' adjusts the amount of a random (per voice) constant offset.
* 'RNG' generates a new set of random offsets. The offsets are stored with the patch.
* 'Drift' adjusts the amount of a random (per voice) drift.
* 'Drift Rate' adjusts the frequency of the random drift.

## Filter
A collection of filters.
* 'Cutoff' sets the filter cutoff frequency.
* 'Reso' sets the filter resonance
	* for non-resonant filters, this has no effect
	* for comb filters, this controls the feedback
	* for diode clippers, this controls the drive
* 'Mode' sets the filter type. A selection of ladder filters, Sallen-Key filters, comb filters and diode clippers are available.
* The 'Mode' can also be set to bypass (if enabled, the 'Post-filter Saturator' will still be active) or mute.

### Context menu options
* 'Oversampling rate' sets the internal oversampling rate.
* 'ODE Solver': The filters are implemented with differential equations, which are solved with numerical methods. 4th order Runkge-Kutta is recommended, the other options use less CPU, but are also less accurate.
* 'Integrator type': This affects the placement of the nonlinearities nl() in the integrators:
	* Linear: dx/dt = ω (in - x)
	* OTA: dx/dt = ω nl(in - x)
	* Transistor: dx/dt = ω (nl(in) - nl(x))
* OTA and Transistor are available with a tanh() nonlinearity, and an alternative softer saturator.
* 'Post-filter Saturator' limits the output to around ±10V.

## Last
A utility module, which allows to map multiple sources to one destination.

VCVRack allows any knob to be mapped by one source. Mapping another source overwrites it.
With this module, you can map up to 4 sources to the 4 knobs. The output is always the value from the knob that is currently moved, or has been moved last. 
The output can be used to map to another knob via [stoermelder's 'µMAP'](https://library.vcvrack.com/Stoermelder-P1/CVMapMicro) or [stoermelder's 'CV-MAP'](https://library.vcvrack.com/Stoermelder-P1/CVMap).

### Context menu options
'Detect changes to same value': If a knob is set to the same value it already had (e.g. the value is 0, and the knob is double-clicked and set to its default value, which is also 0), the output will be updated.

## LFO
A polyphonic low frequency oscillator with CV-controllable frequency, amplitude and phase reset.

### Context menu options
* 'Reduce internal sample rate': The internal sample rate can be reduced. Since LFO signals are usually below audio rate, this can be used to save CPU time.
* 'Bipolar': By default, the LFO is in bipolar mode. The signal is centered around 0V, and the amplitude can be adjusted from 0 to 5V. When unchecked, the LFO is in unipolar mode, the output voltage is between 0V and 10V.

## Mod Matrix
A polyphonic modulation matrix with 13 inputs and 16 outputs.
It is fully MIDI-controllable with 16 knobs and 12 buttons.

The outputs are clamped to ±12V.

### Usage
The recommended usage is to assign up to 16 knobs of your MIDI controller to the 'Control' knobs in the first row, and up to 12 buttons of your MIDI controller to the 'Select' buttons in the right column.
I recommend [stoermelder's 'MIDI-CAT'](https://library.vcvrack.com/Stoermelder-P1/MidiCat). 

Connect modulation sources to the 'Signal' inputs (leave the 'Control knob base values' input disconnected), and modulation destinations to the 'Mix' outputs. 

When no buttons are pressed, the 'Control' knobs control the 'base' value from -5V to 5V (in bipolar mode), or 0V to 10V (in unipolar mode). 
The range of the base value can be changed for all columns, or for each of the 16 columns individually by connecting a 16-channel polyphonic input (e.g. from [stoermelder's 'AFFIX'](https://library.vcvrack.com/Stoermelder-P1/Affix)) to the 'Control knob base values' input.

Selecting a row by holding a button immediately changes the values of the 'Control' knobs to the values of the selected row. You can then control the selected row of the modulation matrix with your MIDI controller knobs.

Once you release the button, the values of the 'Control' knobs revert back to their previous 'base' values.

There are two extra outputs to the right of the control knobs. 'Base control knob values' outputs the 16 base values of the control knobs via 16 polyphonic channels.
'Current control knob values' outputs the 16 current values of the control knobs via 16 polyphonic channels. This can e.g. be used to feed back values to [MindMeld's 'PatchMaster'](https://library.vcvrack.com/MindMeldModular/PatchMaster) via [stoermelder's 'CV-Map'](https://library.vcvrack.com/Stoermelder-P1/CVMap)

### Context menu options
* 'Reduce internal sample rate': The internal sample rate can be reduced. Since modulation signals are usually not audio rate, this can be used to save CPU time.
* 'Latch buttons': The behavior of the buttons can be switched from momentary to latched (this is useful if you want to select the active row with a mouse click). Regardless of the mode, only one row can be selected for editing. The active row is indicated by a light.
* 'Bipolar': The behavior of the knobs can be switched between bipolar (-100% to 100% range) and unipolar (0 to 100% range).
* 'Relative MIDI control mode': If this is not checked, the controls work in absolute mode. This is ideal if you have a controller with encoders, and MIDI feedback.
    With a normal MIDI controller, you will get parameter jumps whenever the positions of your physical knobs don't match the control knob values in Rack.
    With 'Relative MIDI control mode' enabled, your physical knobs control the relative position, so you get no parameter jumps.
    Instead, it can happen that your physical knobs are at the end of their travel, but the on screen knob is not. In that case, you have to turn the physical knob back the whole way, and forth again. 

## OnePole
A simple CV controllable linear 1-pole highpass and lowpass filter.

## OnePoleLP
A simple CV controllable linear 1-pole lowpass filter.

## Oscillators
A pair of analog-style oscillators.
* 'Shape' blends from triangle to sawtooth to pulse wave.
* 'PW' adjusts the phase of the triangle, and the pulse wave from 0% to 100% duty cycle.
* 'Vol' adjusts the oscillator volume.
* 'Sub' adjusts the sub-oscillator volume, a square wave one octave below oscillator 1.
* 'Sync' hard syncs oscillator 2 to oscillator 1.
* 'FM' adjusts the (linear-through-zero) frequency modulation from oscillator 1 to oscillator 2.
Oscillator 2 stays in tune when applying FM.
* 'RM' adjusts the volume of the ring modulator (multiplication of oscillator 1 and 2).

Every parameter is CV controllable.

There are two 'V/Oct' inputs, one for each oscillator. You can use the 'Tune' module to add tuning controls.

'Out' outputs the mix of oscillator 1, the sub-oscillator, oscillator 2 and the ring modulator.

### Context menu options
* 'Oversampling rate': The oscillators use a naive implementation, which is quite CPU friendly, and can therefore be massively oversampled to reduce aliasing.
This is especially useful for FM and sync sounds.
With no oversampling, the oscillators alias a lot.
* 'Anti-aliasing': Apply additional anti-aliasing (with polyBLEPs and polyBLAMPs). This option greatly reduces aliasing, and does not need much additional CPU time. It also works well with sync and FM.
* 'DC blocker': FM and the ring modulator can create a DC offset. Therefore, a DC blocker is enabled by default, but can be disabled in the context menu.
* 'Saturator' limits the output to around ±10V.
* 'LFO mode' lets you use the module as an LFO. It lowers the frequencies of the oscillators to 2 Hz @ 0V, and internally disables oversampling and the DC blocker.

## Synth
A virtual-analogue polyphonic synthesizer with 2 oscillators, dual filters and unique modulation system.

### Motivation
There is no shortage of virtual analogue synthesizers, both in hardware and software. Why did I create a new one?
The point of this synthesizer is its unique modulation system.

In most other synthesizers, part of the modulation is hardcoded (e.g. V/Oct to oscillator pitch, envelope to amp), part is directly accessible from the panel (e.g. envelope to filter cutoff), others are hidden in menus (e.g. pitch bend range).
Some synthesizers have a vintage knob, which applies a predetermined amount of diverge and drift to certain parameters.
And many synthesizers have a separate modulation matrix with a limited number of entries.
Often, this mod matrix seems like an afterthought. You have a nice knob to control the filter cutoff, but in order to modulate it from the matrix, you have to select filter cutoff as a destination from a menu, and dial in the amount with some encoder. Why not use the nice filter cutoff knob also to control the modulation depth?

Here I unified and streamlined the modulation system. There are almost no hardcoded connections, and every source can modulate every destination at the same time. 
Applying a modulation source to a destination is straightforward: press a button, turn a knob.
There are 22 modulation sources, and 50 destinations. That means there is a 1100-slot modulation matrix at your fingertips.

I also find synthesizers with a single filter kind of boring, so I added two filters, with many filter types to chose from.

Since this synthesizer lives inside Rack, it does not have to be a fully self contained synth.
Therefore I deliberately did not add effects, sequencers, an arpeggiator, or voice modes like mono, legato, stack, chord mode etc.
Rack has plenty of options for those, and the modulation system allows external modulation sources to be included easily, as well as to modulate external modules with the 5 individual modulation outputs.

### Overview
On the left side are the (modulation) inputs, on the right side are 5 modulation outputs which can be used to modulate other modules, like a third oscillator or effects.
In the top half are the internal modulation sources: two envelopes, two per-voice LFOs and a global LFO, and two diverge and drift generators.
In the bottom half is the audio path, consisting of two oscillators with FM and ring modulator, a mixer, two filters and an amp.

### Modulation system
All parameters with a ring around them can be modulated. When no assign button is active, the base values of the parameters can be adjusted, and a green ring shows the current value.
If 'Mix Route' is active, a red ring is shown in the mixer section.

Every modulation source has one or more assign buttons. Once an assign button is pressed, it is active and lit bright blue.
The modulation depths for each parameter can be adjusted, the modulation depth is shown with a blue ring.
If 'Mix Route' is active, a purple ring is shown in the mixer section.
By clicking on the assign button once again, the synthesizer goes back into base mode.

The assign buttons are dimly lit, if any modulations are active. Hovering a button with the mouse shows all modulation destinations which are affected. 
In the context menu, the modulations for the source can be cleared.

If a parameter is modulated, a little light below the knob is lit. Hovering a knob with the mouse shows all modulation sources which affect the parameter.
In the context menu, the modulations for the destination can be cleared.

In base mode, Racks 'initialize' and 'randomize' functions only change the base values and non-modulatable parameters.
When an assign button is active, 'initialize' and 'randomize' only affect the modulation assignments for the active modulation source.

#### Caveats
Some modulation assignments have to be made in order for Synth to behave 'normally':
- V/Oct modulates Osc 1 and 2 semi by 100%
- An Envelope or Gate modulates amp by 100%
- For pitch bend, I recommend to turn the pitch bend range in the 'MIDI to CV' module to 'Off', and modulate Osc 1 and 2 semi from 'PW' by 2 semitones

### Signal path
There are 6 audio sources: two [oscillators](#oscillators) with sub-oscillator and ring modulator (multiplication of oscillator 1 and 2), noise, and external input. If no external input is connected, 'Ext' controls the loopback of the mono signal before the amp.

The volume for each source can be adjusted and modulated.
When the 'Mix Route' button is active, each source can be routed to filter 1 (-100%), filter 2 (100%) or anything in between.
The routing can also be modulated.

So there are effectively two mix buses, for filter 1 and filter 2.

The [filters](#filter) can be operated in serial (the output of filter 1 is added to the filter 2 mix bus, filter 1 is not routed to the amp, and filter 1 pan has no effect), or in parallel, or anything in between.

## Tune
Tune by octaves, plus coarse and fine (1 semitone) tuning.

The ranges can be adjusted in the context menu.

The two 'V/Oct' and the 'Fine' input are added and tuned. The 'Fine' input can be used for subtle modulations in the semitone range.

The output is limited to ±12V, so a huge frequency range can be covered.
