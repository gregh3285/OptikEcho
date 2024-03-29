# Interferometer Module by OptikEcho

![Interferometer Panel Image](interferometer-panel.png)

## Introduction

This is a polyphonic piano emulator.

Piano simulation is difficult.  At this point, this module is probably only sufficient for using it as a tool in the context of other work.  I've pushed George Winston midi files at this.  They do okay.  But, it's not a Steinway, by any means.

## Use

### Inputs, Outputs and Parameters

The main I/O to this module are the these:

- **V/OCT** is a polyphonic input for note frequency.  0 volts is C4 and has a frequency of 261.6256 Hz.  V/OCT should only be set at the time of the rising edge of the gate.  It's only looked at when the soundboard file is driving the exciter.  Changing the frequency is computationally intensive and causes delay buffers to be zeroed out re-initializing the channel for that note.  It is not intended to have the V/OCT modulated.  To ensure we the module doesn't update itself on every loop, the frequency needs to change by more than 0.1 Hz and gate needs to be enabled to trigger an update to internal state.

- **GATE** is a polyphonic input for the note.  The rising edge of GATE triggers the start of a note.  It is sustained until GATE is removed.  The threshold for gate is set (without hysteresis) at 0.7 volts. 

- **VELOCITY** (the input) is a polyphonic input for the velocity of the note.  The model currently just updates the gain of the note based upon the input.  This follows the guidance for velocity from 0 volts (*ppp*, quiet) to 10 volts (*fff*, loud).  Sending 0 does *not* shut the note off.  Note on and note off is controlled by the GATE.  Velocity is sampled when the V/OCT causes an update to internal state for a channel.  If nothing is connected to VELOCITY, 50% (5 V) is assumed.

- **OUT** is a *monophonic* output of the piano simulator.

- Light.  I'm an engineer by training.  Everything needs a flashing light.  I liked in the tutorial.  It stays.

- **VELOCITY** (the knob or parameter) either sets the velocity of notes or acts as an attenuator if there's an input to VELOCITY (the input).

Lessor I/O that may not survive include:

- **Brightness** - This is brightness of the sustain.  The effect is subtle when near 1.  See [Length 3 FIR Loop Filter Controlled by "Brightness" and "Sustain"](https://ccrma.stanford.edu/~jos/pasp/Length_FIR_Loop_Filter.html) for a more detailed description.

- **Decay** - If this isn't one, you can make the note decay faster.  This goes against the Brightness parameter above.  This input is on the chopping block.

- **Feedback** - This input controls the low pass filter in the old loop model.  This input is on the chopping block.

### Context Menu

There are several items in the context menu for this module.

- **Loop Model** which model to use.  The old model is just waiting for me to remove it.
  It doesn't have a lot of the stuff the new model has.

- **Dispersion Enabled** set whether is string dispersion enabled.

- **Hammer Enabled** determines if hammer processing is enabled.  if it is, a hammer pulse is convolved with the exciter waveform.  The hammer pulse is based upon velocity and frequency.

- **Strike Position Enabled** determines if the strike position filter is enabled.

### Caveats and Known Issues

- The sound only okay.  The energy decay relief maps look a lot different than a real piano.  Psycho-acoustically it sound muddy.

- I've only tested this under Linux x86_64.

- The long dispersion filter chains add significant delay that can be on the order of the total delay line.  The highest frequency allowed is really limited by the total delay of the filter chain.  This means that high frequency notes may not be achievable with a given filter chain.  For now, if the chain causes a fault, the note just doesn't play.

- The exciter waveform is sampled at 44.1 kHz.  If you change the sample rate in VCV Rack to something other than than, that waveform is not resampled.

- Changing Loop Model or Dispersion Enabled can put individual engines in a confused state with regard to pitch.  It has to do with how the engine caches previous pitch and filter parameters.


## References

Many of the ideas and some of the basic structure of this module come from the work of Karl Hiner.  He has a [piano simulation notebook](https://github.com/khiner/notebooks/blob/master/physical_audio_signal_processing/chapter_9_virtual_musical_instruments_part_2.ipynb) that was crucial in helping get this to where it is.  This notebook is Karl's effort to help decompose the mathematics found in Julius O. Smith's [Physical Audio Signal Processing For Virtual Musical Instruments and Audio Effects, Online version](https://ccrma.stanford.edu/~jos/pasp/).  The dispersion filter used with this piano is based upon [Dispersion modeling in waveguide piano synthesis using tunable allpass filter](http://lib.tkk.fi/Diss/2007/isbn9789512290666/article2.pdf).  This is a neat read.

## The Name

Interferometer is just a name that's unique.  I started working on this shortly after I finished Dr. Aspect and Dr. Brune's Coursera course, [Quantum Optics 1: Single Photons](https://www.coursera.org/learn/quantum-optics-single-photon).  It stuck.  OptikEcho sounded cool and it looks like no one is using it -- at least it wasn't google-able.
