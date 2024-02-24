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
    // TODO: Explore that this really is an allpass filter.
    
};
typedef TAllpassFilter<float> AllpassFilter;

template <typename T = float>
struct TSecondOrderFilter : rack::dsp::IIRFilter<3, 3, T> {
    TSecondOrderFilter() {
    }
};
typedef TSecondOrderFilter<float> SecondOrderFilter;

#define ALLPASS_BUF_SIZE (100000)

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
    set_delay_samples(200.0);  // bogus initial value.
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
  
  //void set_max_delay_samples(int max_delay_samples)
  //{
    // TODO: huh?
    //if max_delay_samples >= self.inputs.size:
    //  self.inputs = np.concatenate([self.inputs, np.zeros(max_delay_samples + 1)])
  //}
  
  void set_delay_samples(float new_delay_samples)
  {
    float fractional_out_pointer;
    float alpha;
    
    INFO("new_delay_samples: %f", new_delay_samples);
    
    if ((new_delay_samples < 0.5) || (new_delay_samples > (ALLPASS_BUF_SIZE-2))) {
      FATAL("delay samples %f", delay_samples);
    }

    delay_samples = new_delay_samples;

    fractional_out_pointer = in_pointer - delay_samples + 1.0f; // out_pointer chases in_pointer
    while (fractional_out_pointer < 0.f) {
      fractional_out_pointer += ALLPASS_BUF_SIZE;
    }
    out_pointer = fractional_out_pointer; // cast from float to int.  TODO: test this works.
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
    INFO("allpass_coefficient: %f", allpass_coefficient);
    INFO("in_pointer: %d", in_pointer);
    INFO("out_pointer: %d", out_pointer);
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
  int delay_fractional=0;
  bool dispersion_enabled = true;
  //static const int TRIG_ON = 1;

  // size of the buffer used for the string
  static const int   BUF_SIZE = 100000;
  static const int PULSE_BUF_SIZE = 256;
  // 
#define NOT_A_NOTE (8675309.f);
  const float LOWEST_FREQUENCY = 27.5f;
  const float HIGHEST_FREQUENCY = 4186.0f;
   
  // soundboard storage
  int soundboard_size = 0;
  float soundboard[BUF_SIZE];

  enum ParamId {
		DECAY_PARAM,
		DELAY_FEEDBACK_FREQ_PARAM,
		PARAMS_LEN
  };
  enum InputId {
		VOCT_INPUT,
		TRIG_INPUT,
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
  
    AllpassDelay delay_buffer;
        
    // trigger state
    int trig_state = 0;
    // create a biquad filter to control the feedback through the delay filter.
    rack::dsp::BiquadFilter delayFilter;
    rack::dsp::BiquadFilter dcFilter;
    float last_trig = 0.f;
    
    // dispersion filter
    float Df0 = 0.f;        // dispersion filter delay and fundamental.
    float a1 = 0.0f;
    float curr_f0 = NOT_A_NOTE;  // current note frequency.
    static const int M = 8;
    AllpassFilter dispersionFilter[M];
    
    // hammer pulse
    int pulse_length = 0;
    float pulse_buf[PULSE_BUF_SIZE];
    float hammer_gain = 0.0f;
  };
  Engine eng[POLY_NUM];
  

  Interferometer() {
    config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
    configParam(DECAY_PARAM, 0.f, 0.125f, 0.f, "");
    configParam(DELAY_FEEDBACK_FREQ_PARAM, 0.f, 0.49f, 0.f, "");
    configInput(VOCT_INPUT, "");
    configInput(TRIG_INPUT, "");
    configOutput(OUT_OUTPUT, "");
    
    // DC blocking set to 20.6 Hz
    // See: https://community.vcvrack.com/t/dc-blocker-in-rack-api/8419/6
    
    master_dcFilter.setParameters(rack::dsp::BiquadFilter::Type::HIGHPASS, 20.6f/44000.0, 0.5, 0.0);
    // normalized frequency for filter is cutoff_freq/sample_rate. goes unstable above 0.5.
    // frequency, q, gain_labeled_as_v.  
    for (int ch = 0; ch < POLY_NUM; ch++) {
      eng[ch].delayFilter.setParameters(rack::dsp::BiquadFilter::Type::LOWPASS, 0.3, 0.5, 0.0); 
      // highpass stuff above 4 or 5 Hz
      eng[ch].dcFilter.setParameters(rack::dsp::BiquadFilter::Type::HIGHPASS, 20.6f/44000.0, 0.5, 0.0); 
    }
    // load the soundboard exciter wave file.
    soundboard_size = load(soundboard);
    INFO("soundboard_size %d", soundboard_size);
    if (soundboard_size < 0) {
      FATAL("soundboard size < 0");
    }
    
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
   
    // loop through channels
    for (int ch=0; ch<POLY_NUM; ch++) {
    
      //if (delay_fractional == 1) {
      //  eng[ch].dispersion_enabled = true;
      //}
      
      float co = 0.f; // output for the given channel.
      float ch_decay = decay;
    
      // Q critically damped is 0.5
      eng[ch].delayFilter.setParameters(rack::dsp::BiquadFilter::Type::LOWPASS, feedback_filter_param, 0.5, 0.0);
    
      // a trigger a rising edge.        
      // allow retriggering.  Arbitrarily the threshold is 0.7
      trigger = inputs[TRIG_INPUT].getVoltage(ch);
      if ((eng[ch].last_trig < 0.7) && (trigger > 0.7)) {
        // set the trigger buffer to 1
        eng[ch].trig_state = 1; 
      }
      
      // handle not goes away (real note decay).
      if (trigger < 0.7) {
        ch_decay = 0.06;
      }
      
      // store away current sample.
      eng[ch].last_trig = trigger;
      
      //if (eng[ch].trig_state == 1) {
      //  hammer_pulse_and_gain(1567.98, 0.9, &eng[ch]);
      //}

      // if we are currently triggered and outputting an impulse,
      if (eng[ch].trig_state != TRIG_OFF) {
      
        // sample the frequency at the point where we are triggering
        // sample each time to avoid the race condition between 
        // trigger signal and stabilization of frequency input.
        float pitch = inputs[VOCT_INPUT].getVoltage(ch);

        // The default frequency (0.f volts) is C4 = 261.6256 Hz.
        float freq = dsp::FREQ_C4 * std::pow(2.f, pitch);
        
        // if the note value has changed, update the dispersion filter
        if (abs(freq - eng[ch].curr_f0) > 0.1) {
          float B = b_from_freq(freq);
          //INFO("B = %f",B);
          update_dispersion_values(freq, eng[ch].M, B, (float)args.sampleRate, &eng[ch]);
          
          // update all M allpass filters in the cascade.
          for (int j = 0; j < eng[ch].M; j++) {
            eng[ch].dispersionFilter[j].setParameter(eng[ch].a1);
          }
          
          // set the delay line length accordingly
          //eng[ch].delay_line_len = args.sampleRate/freq;
          //INFO("delay line len: %f", eng[ch].delay_line_len);
          // retune due to dispersion filter delay at the primary frequency.
          if (dispersion_enabled) {
            //eng[ch].delay_line_len -= eng[ch].Df0;
            INFO("dispersion enabled - f = %f", freq);
            eng[ch].delay_buffer.set_delay_samples(args.sampleRate/freq - eng[ch].Df0);
            //INFO("delay line len updated: %f", eng[ch].delay_line_len);
          } else {
            INFO("dispersion disabled - f = %f", freq);
            eng[ch].delay_buffer.set_delay_samples(args.sampleRate/freq);
          }
        
        }

        // random
        //buffer[buf_head] = 5.0f * (random::uniform()-0.5f);
        // ramp
        //buffer[buf_head] = 5.0f * (trig_state/(float)delay_line_len - 0.5f);
        // decaying exponential of noise?
        // TODO: decaying exponential of noise.
        // piano
        // TODO: since trig_state starts at 1, shouldn't this and the
        //       the termination predicate be minus 1?
        // TODO: Look at 
        //       http://lib.tkk.fi/Diss/2007/isbn9789512290666/article3.pdf
        //       for a better excitation model that doesn't involve
        //       loading a waveform.
        co = 8.0f * soundboard[eng[ch].trig_state - 1];
        
        eng[ch].trig_state++;
        if ((eng[ch].trig_state - 1) >= soundboard_size) {
          eng[ch].trig_state = TRIG_OFF;
        }

      }

      delay_fractional = 1;
      if (delay_fractional == 0) {
        // Dead code
      } else if (delay_fractional == 1) {
      
        float feedback_val = eng[ch].delay_buffer.get_next_out();
        co += (1.0f - ch_decay) * eng[ch].delayFilter.process(feedback_val);
        if (dispersion_enabled) {
          for (int j = 0; j < eng[ch].M; j++) {
            co = eng[ch].dispersionFilter[j].process(co);
          }
        }
        //eng[ch].dispersion_enabled = true;
        
      } else if (delay_fractional == 2) {
        // Dead code
      }
      
      // apply that output DC block filter
      co = eng[ch].dcFilter.process(co);
      
      // sum this channel's output into the master output
      y += co;
      
      // update the head before we leave.
      //eng[ch].buf_head = (eng[ch].buf_head+1) % BUF_SIZE;
      eng[ch].delay_buffer.tick(co);
      //eng[ch].dispersion_enabled = false;
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
                                 SecondOrderFilter *loop_filter)
  {
    float g0 = exp(-6.91 / (sustain_seconds * frequency));
    float b0 = g0 * (1 + brightness) / 2.0;
    float b1 = g0 * (1 - brightness) / 4.0;
    //loop_filter = TwoZeroFilter()
    //loop_filter.set_coefficients(b1, b0, b1);
    loop_filter->b[0] = b1;
    loop_filter->b[1] = b0;
    loop_filter->b[2] = b1;
  
    //return loop_filter
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
  void update_dispersion_values(float f0, int M, float B, float fs, struct Engine *e)
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
    float a1 = (1.f - D)/(1.f + D); // D >= 0, so a1 >= 0
    // don't allow a1 above 0.
    if (a1 >= 0.f) a1 = 0.f;
    
    // the fundamental delay is not presented in Ruahala's paper.
    float Df0 = std::atan(std::sin(wT)/(a1      + std::cos(wT))) / wT - 
                std::atan(std::sin(wT)/(1.0f/a1 + std::cos(wT))) / wT;
    Df0 *= M;  // the above delay is for ONE filter, we cascade M of them.
    // the does align with figure 9 in the paper.
    
    //INFO("update dispersion f %f, Df0 %f, a1 %f", f0, Df0, a1);
    e->Df0 = Df0; // store the delay in the engine data.
    e->a1 = a1;   // store the filter coefficient in the engine data.
    e->curr_f0 = f0;  // we're now updated to this.  don't redo if not needed.
  }
  
  // Anders Askenfelt and Erik Janson "From touch to string vibrations"
  // Five Lectures on the Acoustics of the Piano
  // http://www.speech.kth.se/music/5_lectures/askenflt/askenflt.html
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
    for (int i = 0; i < pulse_length; i++) {
      float r = (float)i / (float)(pulse_length - 1) - 0.5f;  // -0.5 to + 0.5
      e->pulse_buf[i] = 0.5 + 0.5*cos(2.0 * M_PI * r);
      pulse_sum += e->pulse_buf[i];
      INFO("pulse_buff %d %f", i, e->pulse_buf[i]);
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
  
};


struct InterferometerWidget : ModuleWidget {
  InterferometerWidget(Interferometer* module) {
    setModule(module);
    setPanel(createPanel(asset::plugin(pluginInstance, "res/Interferometer.svg")));

    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
    addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
    addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(13.05, 78.178)), module, Interferometer::DECAY_PARAM));
    addParam(createParamCentered<RoundBlackKnob>(mm2px(Vec(33.999, 80.013)), module, Interferometer::DELAY_FEEDBACK_FREQ_PARAM));

    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(12.718, 15.651)), module, Interferometer::VOCT_INPUT));
    addInput(createInputCentered<PJ301MPort>(mm2px(Vec(34.012, 17.686)), module, Interferometer::TRIG_INPUT));

    addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(12.848, 43.189)), module, Interferometer::OUT_OUTPUT));

    addChild(createLightCentered<MediumLight<RedLight>>(mm2px(Vec(34.452, 52.137)), module, Interferometer::ACTIVE_LIGHT));
  }
  
  void appendContextMenu(Menu* menu) override {
    Interferometer* module = getModule<Interferometer>();

    // Controls int Module::exciter
    menu->addChild(createIndexPtrSubmenuItem("Dispersion Enabled",
	    {"False", "True"},
	    &module->dispersion_enabled));
    // Controls int Module::fractional delay
    menu->addChild(createIndexPtrSubmenuItem("Fractional Delay",
	    {"Integer", "Fractional", "Sync"},
	    &module->delay_fractional));
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

    ifstream ifs{asset::plugin(pluginInstance, "res/soundboard.wav").data(), ios_base::binary};
    INFO("soundboard location %s", asset::plugin(pluginInstance, "res/soundboard.wav").data());
    if (!ifs){
        INFO("cannot open soundboard file.");
        return -1;
    }

    // first read RIFF header
    RIFFHeader h;
    ifs.read((char*)(&h), sizeof(h));
    if (!ifs || memcmp(h.chunk_id, riff_id, 4) || memcmp(h.format, format, 4)){
        INFO("bad formatting");
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

            INFO("reading datachunk with length %d", ch.chunk_size); 
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
        INFO("problem reading data.");
        return -1;
    }
    // return the number of samples in the one channel.
    return ebi;
}

Model* modelInterferometer = createModel<Interferometer, InterferometerWidget>("Interferometer");

