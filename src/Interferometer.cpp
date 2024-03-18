#include "plugin.hpp"
#include <iostream>
#include <fstream>
#include <cstring>
#include <stdint.h>
#include <dsp/filter.hpp>

int load(float *extbuff); // load the soundboard file.

// instantiate an one pole filter for the allpass for dispersion
template <typename T = float>
struct TAllpassFilter : rack::dsp::IIRFilter<2, 2, T> {
    TAllpassFilter() {
        setParameter(0.f);
    }
    void setParameter(float a1) {
      // from https://github.com/khiner/notebooks/blob/master/Filters.py
      //  b0, b1, a1 get set to a1, 0, a1 as shown here:
      //  https://colab.research.google.com/github/khiner/notebooks/blob/master/physical_audio_signal_processing/chapter_9_virtual_musical_instruments_part_2.ipynb#scrollTo=wMHzkzt964i1
      //  a1 here is a[0];
      this->b[0] = a1;
      this->b[1] = 1.f;
      this->a[0] = a1;
    }
    
};
typedef TAllpassFilter<float> AllpassFilter;

//template <typename T = float>
//struct TSecondOrderFilter : rack::dsp::IIRFilter<3, 3, T> {
//    TSecondOrderFilter() {
//    }
//};
//typedef TSecondOrderFilter<float> SecondOrderFilter;

#define ALLPASS_BUF_SIZE (3000)

struct TTwoZero {
  // From
  // https://github.com/khiner/notebooks/blob/master/Filters.py
  float in1, in2, b0, b1, b2;
  TTwoZero()
  {
    set_coefficients();
    in1 = 0.0; in2 = 0.0;
  }
    
  float process(float in_sample)
  {
    float out_sample = b2 * in2 + b1 * in1 + b0 * in_sample;
    in2 = in1; in1 = in_sample;   
    return out_sample;
  }
  
  void set_coefficients(float ib0=1.0, float ib1=0.0, float ib2=0.0)
  {
      b0 = ib0; b1 = ib1; b2 = ib2;
      //INFO("b0 %f, b1 %f, b2 %f", b0, b1, b2);
  } 
};
typedef struct TTwoZero TwoZeroFilter;

struct TAllpassDelay {
  // Translation to c++ of:
  // https://github.com/khiner/notebooks/blob/master/AllpassDelay.py
  
  // This class implements a fractional-length digital delay-line using
  // a first-order allpass filter.  If the delay and maximum length are
  // not specified during instantiation, a fixed maximum length of 4095
  // and a delay of 0.5 is set.

  // An allpass filter has unity magnitude gain but variable phase
  // delay properties, making it useful in achieving fractional delays
  // without affecting a signal's frequency magnitude response.  In
  // order to achieve a maximally flat phase delay response, the
  // minimum delay possible in this implementation is limited to a
  // value of 0.5.
  
  float *inputs;
  float last_out;
  float next_out;
  int in_pointer;
  int out_pointer;
  float allpass_input;
  bool do_next_out;
  float delay_samples;
  float allpass_coefficient;
  
  TAllpassDelay()
  {
    // Writing before reading allows delays from 0 to length-1. 
    inputs = new float[ALLPASS_BUF_SIZE]();
    last_out = 0.0;
    next_out = 0.0;
    in_pointer = 0;
    (void)set_delay_samples(200.0);  // bogus initial value.
    allpass_input = 0.0;
    do_next_out = true;
  }
  
  float tick(float in_sample=0.0)
  {
    inputs[in_pointer] = in_sample;
    in_pointer++;
    if (in_pointer == ALLPASS_BUF_SIZE) {
      in_pointer = 0;
    }

    last_out = get_next_out();
    do_next_out = true;

    allpass_input = inputs[out_pointer];
    out_pointer++;
    if (out_pointer == ALLPASS_BUF_SIZE) {
      out_pointer = 0;
    }

    return last_out;
  }
  
  float get_next_out()
  {
    if (do_next_out) {
      next_out = -allpass_coefficient * last_out;
      next_out += allpass_input + (allpass_coefficient * inputs[out_pointer]);
      do_next_out = false;
    }
    return next_out;
  }
  
  bool set_delay_samples(float new_delay_samples)
  // return true if successful, false if not
  {
    float fractional_out_pointer;
    float alpha;
    
    //INFO("new_delay_samples: %f", new_delay_samples);
    
    if ((new_delay_samples < 0.5) || (new_delay_samples > (ALLPASS_BUF_SIZE-2))) {
      FATAL("new delay samples %f", new_delay_samples);
      return false;
    }

    delay_samples = new_delay_samples;

    fractional_out_pointer = in_pointer - delay_samples + 1.0f; // out_pointer chases in_pointer
    while (fractional_out_pointer < 0.f) {
      fractional_out_pointer += ALLPASS_BUF_SIZE;
    }
    out_pointer = fractional_out_pointer; // cast from float to int.  
    if (out_pointer == ALLPASS_BUF_SIZE) {
      out_pointer = 0;
    }
    alpha = 1.0 + out_pointer - fractional_out_pointer; // fractional part

    if (alpha < 0.5) {
      // The optimal range for alpha is about 0.5 - 1.5 in order to
      // achieve the flattest phase delay response.
      out_pointer++;
      if (out_pointer >= ALLPASS_BUF_SIZE) {
        out_pointer -= ALLPASS_BUF_SIZE;
      }
      alpha += 1.0;
    }

    allpass_coefficient = (1.0 - alpha) / (1.0 + alpha);
    //INFO("allpass_coefficient: %f", allpass_coefficient);
    //INFO("in_pointer: %d", in_pointer);
    //INFO("out_pointer: %d", out_pointer);
    return true;
  }
  
  void clear()
  {
    for(int i = 0; i < ALLPASS_BUF_SIZE; i++) {
      inputs[i] = 0.0f;
    }
    allpass_input = 0.0f;
  }
  
  float tap_out(int tap_delay) {
    int tap = in_pointer - tap_delay - 1;
    while (tap < 0) {
      tap += ALLPASS_BUF_SIZE;
    }
    return inputs[tap];
  }

  void tap_in(float value, int tap_delay) {
    int tap = in_pointer - tap_delay - 1;
    while (tap < 0) {
      tap += ALLPASS_BUF_SIZE;
    }
    inputs[tap] = value;
  }

};
typedef TAllpassDelay AllpassDelay;
  
  
struct Interferometer : Module {
  float phase = 0.f;
  float blinkPhase = 0.f;
  const int TRIG_OFF = 0;
  int exciter=0;
  int loop_model=0;
  bool dispersion_enabled = true;
  float sample_rate;
  bool hammer_enabled = true;
  bool strike_pos_enabled = true;
  bool mode_coupling_enabled = true;
  //static const int TRIG_ON = 1;

  // size of the buffer used for the string
  static const int   BUF_SIZE = 150000;
  static const int PULSE_BUF_SIZE = 256;
  // 
#define NOT_A_NOTE (8675309.f);
  const float LOWEST_FREQUENCY = 27.5f;
  const float HIGHEST_FREQUENCY = 4186.0f;
  
  // A hack to futher emulate a stronger attack.
  // Note that this technically invalidates the sustain t60 value.
  const float long_sustain_excite_gain = 0.75;
  
  float brightness = 0.95;
  float detuning = 0.999;
  float coupling_amount = 0.01;
   
  // soundboard storage
  int soundboard_size = 0;
  float soundboard[BUF_SIZE];

  enum ParamId {
                BRIGHTNESS_PARAM,
		DECAY_PARAM,
		DELAY_FEEDBACK_FREQ_PARAM,
		PARAMS_LEN
  };
  enum InputId {
		VOCT_INPUT,
		TRIG_INPUT,
		VELOCITY_INPUT,
		INPUTS_LEN
  };
  enum OutputId {
		OUT_OUTPUT,
		OUTPUTS_LEN
  };
  enum LightId {
		ACTIVE_LIGHT,
		LIGHTS_LEN
  };
    
  rack::dsp::BiquadFilter master_dcFilter;
  static const int POLY_NUM = 16;
  struct Engine {
    bool faulted = false;
    AllpassDelay delay_buffer;
    
    // sustain, brightness, coupled string, hammer position
    AllpassDelay delay_line_v;
    AllpassDelay delay_line_h;
    float t60_initial; 
    float t60_sustain;
    TwoZeroFilter loop_filter_v;
    TwoZeroFilter loop_filter_h;
    AllpassDelay strike_comb_delay;
    float delay_line_out_v = 0;
    float delay_line_out_h = 0;
    
    float feed_forward_co_h = 0;
    
    // gain  10v velocity -> 100%.  0v = 0%.
    float engine_gain = 1.0f;
    
    // trigger state
    int trig_state = 0;

    // create a biquad filter to control the feedback through the delay filter.
    rack::dsp::BiquadFilter delayFilter;
    rack::dsp::BiquadFilter dcFilter;
    float last_trig = 0.f;
    
    rack::dsp::BiquadFilter loopFilter_v;
    rack::dsp::BiquadFilter loopFilter_h;
    
    // dispersion filter
    float Df0 = 0.f;        // dispersion filter delay and fundamental.
    float a1 = 0.0f;
    float curr_f0 = NOT_A_NOTE;  // current note frequency.
    static const int M = 16;
    AllpassFilter dispersionFilter[M];
    // dispersion filtering for new loop with string coupling.
    float Df0_v = 0.f;        // dispersion filter delay and fundamental.
    float a1_v = 0.0f;
    float Df0_h = 0.f;        // dispersion filter delay and fundamental.
    float a1_h = 0.0f;
    AllpassFilter dispersionFilter_v[M];
    AllpassFilter dispersionFilter_h[M];
    
    // hammer pulse
    int pulse_length = 0;
    float pulse_buf[PULSE_BUF_SIZE];
    float hammer_gain = 0.0f;
    
    float soundboard_gain = 1.0;

  };
  Engine eng[POLY_NUM];
  

  Interferometer() {
    //INFO("initializing");
    config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
    configParam(BRIGHTNESS_PARAM, 0.f, 1.f, 0.95f, "Brightness");
    configParam(DECAY_PARAM, 0.f, 0.125f, 0.f, "Decay");
    configParam(DELAY_FEEDBACK_FREQ_PARAM, 0.f, 0.49f, 0.40f, "Delay Feedback Freq");
    configInput(VOCT_INPUT, "V/Oct");
    configInput(TRIG_INPUT, "Gate");
    configInput(VELOCITY_INPUT, "Velocity");
    configOutput(OUT_OUTPUT, "Output");
    //AllpassDelay::max_size = 0.0;
    
    // DC blocking set to 20.6 Hz
    // See: https://community.vcvrack.com/t/dc-blocker-in-rack-api/8419/6
    
    master_dcFilter.setParameters(rack::dsp::BiquadFilter::Type::HIGHPASS, 20.6f/44000.0, 0.5, 0.0);
    //INFO("dcFilter a[0]; %f",master_dcFilter.a[0]);
    //INFO("dcFilter a[1]; %f",master_dcFilter.a[1]);
    //INFO("dcFilter b[0]; %f",master_dcFilter.b[0]);
    //INFO("dcFilter b[1]; %f",master_dcFilter.b[1]);
    //INFO("dcFilter b[2]; %f",master_dcFilter.b[2]);
    
    // get the sample rate.
    sample_rate = APP->engine->getSampleRate();
    
    // normalized frequency for filter is cutoff_freq/sample_rate. goes unstable above 0.5.
    // frequency, q, gain_labeled_as_v.  
    for (int ch = 0; ch < POLY_NUM; ch++) {
      eng[ch].delayFilter.setParameters(rack::dsp::BiquadFilter::Type::LOWPASS, 0.3, 0.5, 0.0); 
      eng[ch].loopFilter_v.setParameters(rack::dsp::BiquadFilter::Type::LOWPASS, 0.3, 0.5, 0.0); 
      eng[ch].loopFilter_h.setParameters(rack::dsp::BiquadFilter::Type::LOWPASS, 0.3, 0.5, 0.0); 
      // highpass stuff above 4 or 5 Hz
      eng[ch].dcFilter.setParameters(rack::dsp::BiquadFilter::Type::HIGHPASS, 20.6f/sample_rate, 0.5, 0.0); 
    }
    // load the soundboard exciter wave file.
    soundboard_size = load(soundboard);
    //INFO("soundboard_size %d", soundboard_size);
    if (soundboard_size < 0) {
      FATAL("soundboard size < 0");
    }
    //INFO("strike position 30.0: %f", strike_position(30.0));
    //INFO("strike position 400.0: %f", strike_position(400.0));
    //INFO("strike position 500.0: %f", strike_position(500.0));
    //INFO("strike position 1000.0: %f", strike_position(1000.0));
    //INFO("strike position 2000.0: %f", strike_position(2000.0));
    //INFO("strike position 4000.0: %f", strike_position(4000.0));
  }

  void set_frequency(float freq, struct Engine *e)
  {
      
    // this function is only called if an update is made to the frequency.
    // if no value change is made, do nothing
    float B = b_from_freq(freq);
    //INFO("B = %f",B);
    update_dispersion_values(freq, e->M, B, sample_rate, &(e->Df0), &(e->a1));
    update_dispersion_values(freq, e->M, B, sample_rate, &(e->Df0_v), &(e->a1_v));
    update_dispersion_values(freq*detuning, e->M, B, sample_rate, &(e->Df0_h), &(e->a1_h));
    
    e->loopFilter_v.setParameters(rack::dsp::BiquadFilter::Type::LOWPASS, 0.4, 0.5, 0.0); 
    e->loopFilter_h.setParameters(rack::dsp::BiquadFilter::Type::LOWPASS, 0.4, 0.5, 0.0);
    
    // update all M allpass filters in the cascade.
    for (int j = 0; j < e->M; j++) {
      e->dispersionFilter[j].setParameter(e->a1);
      e->dispersionFilter_v[j].setParameter(e->a1_v);
      // Assumption.  The detuning of h is not relevant to setting dispersion filters.
      e->dispersionFilter_h[j].setParameter(e->a1_h);
    }
    
    // set the delay line length accordingly   
    // retune due to dispersion filter delay at the primary frequency.
    if (dispersion_enabled) {
      //INFO("dispersion enabled - f = %f", freq);
      if (!e->delay_buffer.set_delay_samples(sample_rate/freq - e->Df0)) {
        e->faulted = true;
      }
      //INFO("delay line len updated: %f", eng[ch].delay_line_len);
      // index 0 = v
      //INFO("delay_line_v set samples = %f",sample_rate/freq - e->Df0_v);
      // The minus one is tuning. Probably related to the loop filter.
      if(!e->delay_line_v.set_delay_samples(sample_rate/freq - e->Df0_v - 1)) {
        e->faulted = true;
      }
      if(!e->delay_line_h.set_delay_samples(sample_rate/freq*detuning - e->Df0_h - 1)) {
        e->faulted = true;
      }
    } else {
      //INFO("dispersion disabled - f = %f", freq);
      if(!e->delay_buffer.set_delay_samples(sample_rate/freq)) {
        e->faulted = true;
      }
      // index 0 = v
      if(!e->delay_line_v.set_delay_samples(sample_rate/freq-2)) {
        e->faulted = true;
      }
      if(!e->delay_line_h.set_delay_samples(sample_rate/freq*detuning-2)) {
        e->faulted = true;
      }
    }
   
    initial_and_sustained_t60s(freq, &(e->t60_initial), &(e->t60_sustain));
    // index 0 = v
    sustain_brightness_filter(e->t60_initial, brightness, freq,
                              &(e->loop_filter_v));
    // index 1 = h
    sustain_brightness_filter(e->t60_sustain, brightness, freq * detuning,
                              &(e->loop_filter_h));
    
    if(!e->strike_comb_delay.set_delay_samples(strike_position(freq) * sample_rate/freq)) {
      e->faulted = true;
    }
    
    // log that we've updated frequency, so we don't keep doing it again
    // unless it changes.
    e->curr_f0 = freq;
  }
  
  void process(const ProcessArgs &args) override {

    float decay;
    float trigger;
    float y =0.f; // output for the set of channels (summed)
    
    float feedback_filter_param;

    // get the Karplus decay parameter from the knob.
    decay = params[DECAY_PARAM].getValue();
    
    // read the value and update the parameters of the biquad feedback filter.
    feedback_filter_param  = params[DELAY_FEEDBACK_FREQ_PARAM].getValue();

    // Blink light at 1Hz
    blinkPhase += args.sampleTime;
    if (blinkPhase >= 1.f) {
      blinkPhase -= 1.f;
    }
    lights[ACTIVE_LIGHT].setBrightness(blinkPhase < 0.5f ? 1.f : 0.f);
   
    // loop through connected channels. 
    // use the gate (not velocity of voct) 
    // to determine which channels are connected.
    for (int ch=0; ch<inputs[TRIG_INPUT].getChannels(); ch++) {
    
      //if (delay_fractional == 1) {
      //  eng[ch].dispersion_enabled = true;
      //}
      
      float co = 0.f; // output for the given channel.
      float ch_decay = decay;
    
      // Q critically damped is 0.5
      // This is only used for the old model.
      eng[ch].delayFilter.setParameters(rack::dsp::BiquadFilter::Type::LOWPASS, feedback_filter_param, 0.5, 0.0);
    
      
      // a trigger a rising edge.        
      // allow retriggering.  Arbitrarily the threshold is 0.7
      // TODO: This should be renamed GATE_INPUT.
      trigger = inputs[TRIG_INPUT].getVoltage(ch);
      if ((eng[ch].last_trig < 0.7) && (trigger > 0.7)) {
        // set the trigger buffer to 1
        eng[ch].trig_state = 1; 
        // clear any faults
        eng[ch].faulted = false;
        
        // clear the allpass delay buffers
        eng[ch].delay_line_v.clear();
        eng[ch].delay_line_h.clear();
        eng[ch].strike_comb_delay.clear();
      }
      
      // handle note going away (real note decay).
      if (trigger < 0.7) {
        ch_decay = 0.06;
        eng[ch].soundboard_gain *= 0.97; // decay the soundboard sample to 0;

      }
      
      // store away current sample.
      eng[ch].last_trig = trigger;
      
      // if we are currently triggered and outputting an impulse,
      if (eng[ch].trig_state != TRIG_OFF) {
        // sample the frequency at the point where we are triggering
        // sample each time to avoid the race condition between 
        // trigger signal and stabilization of frequency input.
        float pitch = inputs[VOCT_INPUT].getVoltage(ch);

        // The default frequency (0.f volts) is C4 = 261.6256 Hz.
        float freq = dsp::FREQ_C4 * std::pow(2.f, pitch);
        
        // if the note value has changed, update the frequency stuff
        // and set gain from velocity.  
        // TODO: Also need to check for changes to amplitude...
        //       as velocity may change after the gate clock arrives
        //       or changes in frequency occur...
        if ((abs(freq - eng[ch].curr_f0) > 0.1) || (eng[ch].trig_state == 1)) {
          brightness = params[BRIGHTNESS_PARAM].getValue();
          INFO("frequency: %f", freq);
          set_frequency(freq, &eng[ch]);   
          
          eng[ch].soundboard_gain = 1.0f;
          
          // note gain based upon velocity.
          // https://www.hedsound.com/p/midi-velocity-db-dynamics-db-and.html
          // 10 V = 0 dbV
          // Volts = 10^^(dBV/20) 
          // MMA GM MIDI specification:
          // Volume (CC#7) and Expression (CC #11) should be implemented as follows:
          // For situations in which only CC# 7 is used (CC#11 is assumed "127"):
          // L(dB) = 40 log (V/127) where V= CC#7 value
          // For example: CC#7 amplitude
          // 127    0dB    10 V in CV
          // 96 - 4.8dB
          // 64 -11.9dB
          // 32 -23.9dB
          // 16 -36.0dB
          // 0  -oo         0 V in CV
          // -4.12*(10.0 - CV) db
          #define FUDGE_DB (12)
          float cvvel;
          if (inputs[VELOCITY_INPUT].isConnected()) {
            cvvel = math::clamp(inputs[VELOCITY_INPUT].getVoltage(ch), 0.0f, 10.0f); 
          } else {
            cvvel = 5.0f;
          }
          if (hammer_enabled) {
            // gain is set by the hammer waveform (mostly)
            hammer_pulse_and_gain(freq, cvvel/10.0f, &eng[ch]);
            eng[ch].engine_gain = 20.f;  // a fudge constant...
          } else {
            // gain is set by the MIDI db (chart above)
            float cvdb = -4.12 * (10.0 - cvvel) + FUDGE_DB;
            eng[ch].engine_gain = pow(10, (cvdb / 20.0));
          }
          
          //INFO("cvvel %f, cvdb %f, engine_gain %f", cvvel, cvdb, eng[ch].engine_gain);
        }

        // Alternate approach:
        //       http://lib.tkk.fi/Diss/2007/isbn9789512290666/article3.pdf
        //       for a better excitation model that doesn't involve
        //       loading a waveform.
        //       That paper involves a lot of extraction of parameters from
        //       existing pianos notes.
        // The subtraction of 1 is because we start with trig_state = 1.  
        // That should be index zero in the soundboard waveform.
        if (hammer_enabled) {
          // perform the convolution of the hammer pulse and the soundboard
          // at position trig_state -1.
          // The hammer pulse is symmetric, so it doesn't have to be reversed.
          float hammer_out = 0.f;
          for(int con_ndx = 0; con_ndx < eng[ch].pulse_length; con_ndx++) {
            int e_ndx = (eng[ch].trig_state -1) - eng[ch].pulse_length + con_ndx + 1;
            if ((e_ndx >= 0) && (e_ndx < soundboard_size)) { 
              hammer_out += eng[ch].pulse_buf[con_ndx] * soundboard[e_ndx] * eng[ch].hammer_gain;
            }
          }
          co = hammer_out * eng[ch].soundboard_gain;
        } else {
          co = 8.0f * soundboard[eng[ch].trig_state - 1] * eng[ch].soundboard_gain;
        }
        
        eng[ch].trig_state++;
        if ((eng[ch].trig_state - 1) >= soundboard_size) {
          eng[ch].trig_state = TRIG_OFF;
        }

      }

      // move to the next channel, if this one is faulted.
      if(eng[ch].faulted) {
        continue;
      }

      // Old loop model
      if (loop_model == 1) {
      
        float feedback_val = eng[ch].delay_buffer.get_next_out();
        co += (1.0f - ch_decay) * eng[ch].delayFilter.process(feedback_val);
        if (dispersion_enabled) {
          for (int j = 0; j < eng[ch].M; j++) {
            co = eng[ch].dispersionFilter[j].process(co);
          }
        }

        // update the head before we leave.
        eng[ch].delay_buffer.tick(co);
        
        // apply that output DC block filter
        co = eng[ch].dcFilter.process(co) * eng[ch].engine_gain; 

        // nan check
        if (std::isnan(co)) {
          FATAL("channel %d attempting to output nan!", ch);
          // TODO: reset the channel!
        }

      // New loop model       
      } else if (loop_model == 0) {
              
        // exciter is already in co.
        
        // TODO: parameter for tuning this?!
        if (strike_pos_enabled) {
          co -= eng[ch].strike_comb_delay.tick(co);
        }
       
        //float into_loop_filter;
        //float back_into_delay_line; 
        float feedback_val;
        float co_v;
        float co_h;
        
        co_v = co;
        //co_h = co * long_sustain_excite_gain;
        co_h = co;

        // v  index 0 
        // TODO: The order of operation is different from the notebook.  Is that relevant?
        feedback_val = eng[ch].delay_line_v.get_next_out();
        // Loop filter appears broken!
        if (mode_coupling_enabled) {
          feedback_val = eng[ch].loopFilter_v.process(feedback_val*(1-coupling_amount)+
                                                      eng[ch].feed_forward_co_h*coupling_amount);
        } else {
          feedback_val = eng[ch].loopFilter_v.process(feedback_val);
        }
        co_v += eng[ch].loop_filter_v.process(feedback_val);
        if (dispersion_enabled) {
          //back_into_delay_line = self.dispersion_filters[i].tick(back_into_delay_line);
          for (int j = 0; j < eng[ch].M; j++) {
            co_v = eng[ch].dispersionFilter_v[j].process(co_v);
          }
        }
        eng[ch].delay_line_v.tick((1.0 - ch_decay) * co_v);


        // h index 1
        feedback_val = eng[ch].delay_line_h.get_next_out();
        if (mode_coupling_enabled) {
          feedback_val = eng[ch].loopFilter_h.process(feedback_val*(1-coupling_amount)+co_v*coupling_amount);
        } else {
          feedback_val = eng[ch].loopFilter_h.process(feedback_val);
        }
        // Loop filter appears broken!
        co_h += eng[ch].loop_filter_h.process(feedback_val);
        if (dispersion_enabled) {
          //back_into_delay_line = self.dispersion_filters[i].tick(back_into_delay_line);
          for (int j = 0; j < eng[ch].M; j++) {
            co_h = eng[ch].dispersionFilter_h[j].process(co_h);
          }
        }
        eng[ch].feed_forward_co_h = co_h;
        eng[ch].delay_line_h.tick((1.0 - ch_decay) * co_h );

        // loop gain is 1.0 gain
        co = (co_v + co_h) * eng[ch].engine_gain;
        
        // nan check
        if (std::isnan(co)) {
          FATAL("channel %d attempting to output nan!", ch);
          // TODO: reset the channel!
        }
      } 
      
      // sum this channel's output into the master output
      y += co ;
      
    }
    
    // clamp outputs then output.
    //y = math::clamp(-9.f, y, 9.f);
    //y = master_dcFilter.process(y);
    
    // output the master voltage.
    outputs[OUT_OUTPUT].setVoltage(y);
    
  }
  
  // Length-3 FIR filter with sustain and brightness controls
  // From https://ccrma.stanford.edu/~jos/pasp/Length_FIR_Loop_Filter.html
  void sustain_brightness_filter(float sustain_seconds, 
                                 float brightness, 
                                 float frequency,
                                 TwoZeroFilter *loop_filter)
  {
    //INFO("sustain_seconds %f", sustain_seconds);
    //INFO("brightness %f", brightness);
    //INFO("frequency %f", frequency);
    float g0 = exp(-6.91 / (sustain_seconds * frequency));
    float b0 = g0 * (1 + brightness) / 2.0;
    float b1 = g0 * (1 - brightness) / 4.0;
    loop_filter->set_coefficients(b1, b0, b1);
    //INFO("g0 %f", g0);
    //INFO("b0 %f", b0);
    //INFO("b1 %f", b1);
  }
  
  // Initial t60s range from 15 seconds (A0) to 0.3 seconds (C8)
  // Sustained t60s range from 50 seconds (A0) to 0.3 seconds (C8)
  // From:
  // Fletcher and Rossing "The Physics of Musical Instruments", 2nd edition
  // Springer-Verlag, New York, 1998, p. 384
  void initial_and_sustained_t60s(float frequency,
                                  float *t60_initial,
                                  float *t60_sustain,
                                  float max_initial_t60_sec=15.0, 
                                  float max_sustained_t60_sec=50.0)
  {
    float low_freq_log10_t60s_i;
    float low_freq_log10_t60s_s;
    float t60_scalar;
    
    low_freq_log10_t60s_i = log10(max_initial_t60_sec);
    low_freq_log10_t60s_s = log10(max_sustained_t60_sec);

    t60_scalar = (frequency - LOWEST_FREQUENCY) / (HIGHEST_FREQUENCY - LOWEST_FREQUENCY);
    
    *t60_initial = pow(10.0f, (low_freq_log10_t60s_i - t60_scalar * (low_freq_log10_t60s_i - log10(0.3f))));
    *t60_sustain = pow(10.0f, (low_freq_log10_t60s_s - t60_scalar * (low_freq_log10_t60s_s - log10(0.3f))));
    
    return;
  }
  
  // Striking positions range from 0.122 (A0) to 0.115 (A4) to 0.08 (C8)
  // Harold A. Conklin, Jr.
  // "Design and tone in the mechanoacoustic piano. Part I. Piano hammers
  // and tonal effects" Journal of the Acoustical Society of America
  // Vol. 99, No. 6, June 1996, p. 3293
  float strike_position(float frequency)
  {
    float const A4_FREQUENCY = 440.0;
    float strike_scalar;
    if (frequency <= A4_FREQUENCY) {
      strike_scalar = (log10(frequency) - log10(LOWEST_FREQUENCY)) / (log10(A4_FREQUENCY) - log10(LOWEST_FREQUENCY));
      return (0.122 - strike_scalar * (0.122f - 0.115f));
    }
    else {
      strike_scalar = (HIGHEST_FREQUENCY - frequency) / (HIGHEST_FREQUENCY - A4_FREQUENCY);
      return (0.08 + strike_scalar * (0.115f - 0.08f));
    }
  }
  
  // this function is used to calculate B to calculate dispersion filter parameters.
  float b_from_freq(float freq)
  {
    // https://www.phys.unsw.edu.au/jw/notes.html
    // See figure 7 of:
    // http://lib.tkk.fi/Diss/2007/isbn9789512290666/article2.pdf
    // key 88 has B = 2e-2 -> note C8 -> 4186 Hz 
    // key 25 has B = 1e-4 -> note A2 -> 110 Hz 
    // key 1  has B = 2e-4 -> note A0 -> 27.5 Hz  
    // log-log interpolation between freq and b
    // manually calculated the magic numbers by doing the
    // interpolation in wxmaxima.

    if (freq <= 110.f) {
      return exp(-0.792f*log(freq)-5.485f);
    } else {
      return exp(1.265f*log(freq)-15.159f);
    }
  }
  
  
  // see https://colab.research.google.com/github/khiner/notebooks/blob/master/
  //             physical_audio_signal_processing
  //             chapter_9_virtual_musical_instruments_part_2.ipynb#
  //             scrollTo=wMHzkzt964i1&line=2&uniqifier=1
  // and, http://lib.tkk.fi/Diss/2007/isbn9789512290666/article2.pdf
  void update_dispersion_values(float f0, int M, float B, float fs, float *Df0, float *a1)
  {
    // the article above explains the design methodology for the 
    // all-pass filters, what B is, selection of m, etc.
    float wT = 2 * M_PI * f0 / fs;
    float Bc = std::max(B, 0.000001f);
    static const float k1 = -0.00179f; 
    static const float k2 = -0.0233f; 
    static const float k3 = -2.93f;
    float kd;
    kd = std::exp(k1*std::log(Bc)*std::log(Bc) + k2*std::log(Bc) + k3);
    static const float m1 = 0.0126f; 
    static const float m2 = 0.0606f; 
    static const float m3 = -0.00825f; 
    static const float m4 = 1.97f;
    float Cd;
    Cd = std::exp((m1*std::log(M) + m2)*std::log(Bc) + m3*std::log(M) + m4);
    static const float trt = 1.0594630943592953f; // 2^(1/2)
    float Ikey = std::log(f0 * trt / 27.5f) / std::log(trt);
    float D = std::exp(Cd - Ikey*kd);
    //INFO("update dispersion D %f", D);
    *a1 = (1.f - D)/(1.f + D); // D >= 0, so a1 >= 0
    // don't allow a1 above 0.
    if (*a1 >= 0.f) *a1 = 0.f;
    
    // the fundamental delay is not presented in Ruahala's paper.
    *Df0 = std::atan(std::sin(wT)/((*a1)      + std::cos(wT))) / wT - 
           std::atan(std::sin(wT)/(1.0f/(*a1) + std::cos(wT))) / wT;
    *Df0 *= M;  // the above delay is for ONE filter, we cascade M of them.
    // the does align with figure 9 in the paper.
    
    //INFO("update dispersion f %f, Df0 %f, a1 %f", f0, Df0, a1);
    
  }
  
  // Anders Askenfelt and Erik Janson "From touch to string vibrations"
  // Five Lectures on the Acoustics of the Piano
  // http://www.speech.kth.se/music/5_lectures/askenflt/askenflt.html
  // frequency in Hz
  // amplitude 0 to 1
  void hammer_pulse_and_gain(float frequency, float amplitude, struct Engine *e)
  {
    // all units are in milliseconds
    static const float min_duration_hf = 0.5f;
    static const float min_duration_lf = 2.0f;
    static const float max_duration_hf = 1.2f;
    static const float max_duration_lf = 4.0f;
    static const float sample_rate = 44100.f; // TODO: hardcoded sample rate.
    float velocity_scalar = amplitude;
    float frequency_scalar = 0.0f;
    float min_duration;
    float max_duration;
    float pulse_duration;
    float pulse_sum = 0.0f;
    int pulse_length;
    if (frequency < LOWEST_FREQUENCY) {
      frequency_scalar = 0.0f;
    }
    else if (frequency > HIGHEST_FREQUENCY) {
      frequency_scalar = 1.0f;
    }
    else {
      frequency_scalar = (log(frequency) - log(LOWEST_FREQUENCY)) / 
                         (log(HIGHEST_FREQUENCY) - log(LOWEST_FREQUENCY));
    }
    min_duration = min_duration_lf - frequency_scalar * (min_duration_lf - min_duration_hf);
    max_duration = max_duration_lf - frequency_scalar * (max_duration_lf - max_duration_hf);
    pulse_duration = max_duration - velocity_scalar * (max_duration - min_duration);
    pulse_duration *= 0.5f; // My addition - less LP filtering by shrinking every pulse
    pulse_length = pulse_duration * sample_rate / 1000.f;
    e->pulse_length = pulse_length;
    if (pulse_length > PULSE_BUF_SIZE) {
      FATAL("hammer pulse too long: %d", pulse_length);
    }
    INFO("pulse_length %d", pulse_length);
    for (int i = 0; i < pulse_length; i++) {
      float r = (float)i / (float)(pulse_length - 1) - 0.5f;  // -0.5 to + 0.5
      e->pulse_buf[i] = 0.5 + 0.5*cos(2.0 * M_PI * r);
      pulse_sum += e->pulse_buf[i];
      //INFO("pulse_buff %d %f", i, e->pulse_buf[i]);
    }
    INFO("pulse_sum: %f",pulse_sum);
    e->hammer_gain = amplitude/pulse_sum;
    INFO("hammer_gain: %f",e->hammer_gain);
  }
  
  void onReset(const ResetEvent& e) override {
    // Reset all parameters
    Module::onReset(e);   
    for (int ch = 0; ch < POLY_NUM; ch++) {
      // stop any not initiation
      eng[ch].trig_state = 0;
      // clear the delay buffer
      //for (int i = 0; i < BUF_SIZE; i++) eng[ch].buffer[i] = 0.f;
      eng[ch].delay_buffer.clear();
    }
  }
  json_t* dataToJson() override {
    json_t* rootJ = json_object();
    json_object_set_new(rootJ, "loop_model", json_integer(loop_model));
    json_object_set_new(rootJ, "dispersion_enabled", json_integer(dispersion_enabled));
    json_object_set_new(rootJ, "hammer_enabled", json_integer(hammer_enabled));
    json_object_set_new(rootJ, "strike_pos_enabled", json_integer(strike_pos_enabled));
    json_object_set_new(rootJ, "mode_coupling_enabled", json_integer(mode_coupling_enabled));
    return rootJ;
  } 

  void dataFromJson(json_t* rootJ) override {
  
    json_t* modeJ = json_object_get(rootJ, "loop_model");
    if (modeJ)
      loop_model = json_integer_value(modeJ);
      
    modeJ = json_object_get(rootJ, "dispersion_enabled");
    if (modeJ)
      dispersion_enabled = json_integer_value(modeJ);

    modeJ = json_object_get(rootJ, "hammer_enabled");
    if (modeJ)
      hammer_enabled = json_integer_value(modeJ);
 
    modeJ = json_object_get(rootJ, "strike_pos_enabled");
    if (modeJ)
      strike_pos_enabled = json_integer_value(modeJ);
      
    modeJ = json_object_get(rootJ, "mode_coupling_enabled");
    if (modeJ)
      mode_coupling_enabled = json_integer_value(modeJ);
      
  }
  
};


struct InterferometerWidget : ModuleWidget {
  InterferometerWidget(Interferometer* module) {
    setModule(module);
    setPanel(createPanel(asset::plugin(pluginInstance, "res/Interferometer.svg")));

    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(25.4, 53.932)), module, Interferometer::BRIGHTNESS_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(14.753, 73.771)), module, Interferometer::DECAY_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(36.047, 73.771)), module, Interferometer::DELAY_FEEDBACK_FREQ_PARAM));

    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(14.753, 17.686)), module, Interferometer::VOCT_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(36.047, 17.686)), module, Interferometer::TRIG_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(36.047, 38.474)), module, Interferometer::VELOCITY_INPUT));

    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(14.753, 38.474)), module, Interferometer::OUT_OUTPUT));

    addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(44.484, 12.506)), module, Interferometer::ACTIVE_LIGHT));
}
  
  void appendContextMenu(Menu* menu) override {
    Interferometer* module = getModule<Interferometer>();

    // Controls various things
    menu->addChild(createIndexPtrSubmenuItem("Dispersion Enabled",
	    {"False", "True"},
	    &module->dispersion_enabled));
    menu->addChild(createIndexPtrSubmenuItem("Model",
	    {"New", "Old"},
	    &module->loop_model));
    menu->addChild(createIndexPtrSubmenuItem("Hammer Enabled",
	    {"False", "True"},
	    &module->hammer_enabled));
    menu->addChild(createIndexPtrSubmenuItem("Strike Position Enabled",
	    {"False", "True"},
	    &module->strike_pos_enabled));
    menu->addChild(createIndexPtrSubmenuItem("Mode Coupling Enabled",
	    {"False", "True"},
	    &module->mode_coupling_enabled));	
  }
};

using namespace std;

// wav file reading code based upon https://stackoverflow.com/a/32128050

struct RIFFHeader{
    char chunk_id[4];
    uint32_t chunk_size;
    char format[4];
};

struct ChunkInfo{
    char chunk_id[4];
    uint32_t chunk_size;
};

struct FmtChunk{
    uint16_t audio_format;
    uint16_t num_channels;
    uint32_t sample_rate;
    uint32_t byte_rate;
    uint16_t block_align;
    uint16_t bits_per_sample;
};

struct DataChunk
{  
    int nb_of_samples;
    int16_t* data;
    DataChunk(int s): nb_of_samples{s}, data{new int16_t[s]} {}
    ~DataChunk(){delete[] data;}
};

int load(float *extbuff){
    constexpr char riff_id[4] = {'R','I','F','F'};
    constexpr char format[4] = {'W','A','V','E'};
    constexpr char fmt_id[4] = {'f','m','t',' '};
    constexpr char data_id[4] = {'d','a','t','a'};

    // OLD
    //ifstream ifs{asset::plugin(pluginInstance, "res/soundboard.wav").data(), ios_base::binary};
    // NEW
    ifstream ifs{asset::plugin(pluginInstance, "res/piano_impulse.wav").data(), ios_base::binary};
    //INFO("soundboard location %s", asset::plugin(pluginInstance, "res/soundboard.wav").data());
    if (!ifs){
        FATAL("cannot open soundboard file.");
        return -1;
    }

    // first read RIFF header
    RIFFHeader h;
    ifs.read((char*)(&h), sizeof(h));
    if (!ifs || memcmp(h.chunk_id, riff_id, 4) || memcmp(h.format, format, 4)){
        FATAL("bad formatting");
        return -1;
    }

    // read chunk infos iteratively
    ChunkInfo ch;
    bool fmt_read = false;
    bool data_read = false;
    int ebi = 0;
    while(ifs.read((char*)(&ch), sizeof(ch))){

        // if fmt chunk?
        if (memcmp(ch.chunk_id, fmt_id, 4) == 0){
            FmtChunk fmt;
            ifs.read((char*)(&fmt), ch.chunk_size);
            fmt_read = true;
        }
        // is data chunk?
        else if(memcmp(ch.chunk_id, data_id, 4) == 0){
            DataChunk dat_chunk(ch.chunk_size/sizeof(int16_t));
            ifs.read((char*)dat_chunk.data, ch.chunk_size);
            data_read = true;

            //INFO("reading datachunk with length %d", ch.chunk_size); 
            long unsigned int i;
            
            for (i = 0; i < ch.chunk_size/sizeof(int16_t); i++) {
              // samples alternate between channels as the file is stereo.
              // use just one channel.
              if ((i % 2) == 0) {
                //cout << dat_chunk.data[i] << endl;
                // normalize to one volt peak.  divide by 16 (polyphony)
                extbuff[ebi++] = dat_chunk.data[i]/32768.0/16.0;
              }
            }
        }
        // otherwise skip the chunk
        else{
            ifs.seekg(ch.chunk_size, ios_base::cur);
        }
    }
    if (!data_read || !fmt_read){
        FATAL("problem reading data.");
        return -1;
    }
    INFO("Loaded soundboard impulse response file.  %d samples", ebi);
    // return the number of samples in the one channel.
    
    // run the waveform through a low-pass filter
    //rack::dsp::BiquadFilter lowPassFilter;
    //lowPassFilter.setParameters(rack::dsp::BiquadFilter::Type::LOWPASS_1POLE, 1000.0/44100.0, 0.5, 0.0);
    //for (int lcv = 0; lcv < ebi; lcv++) {
    //  extbuff[lcv] = lowPassFilter.process(extbuff[lcv]);
    //}
    //for (int lcv = 0; lcv < 2000; lcv++) {
    //  extbuff[lcv] = (float)lcv/2000.0 * extbuff[lcv];
    //}
    
    INFO("Soundboard filtered.");
    return ebi;
}

Model* modelInterferometer = createModel<Interferometer, InterferometerWidget>("Interferometer");

