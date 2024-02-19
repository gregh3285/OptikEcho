#include "plugin.hpp"
#include <iostream>
#include <fstream>
#include <cstring>
#include <stdint.h>
#include <dsp/filter.hpp>

// $Id: Interferometer.cpp,v 1.4 2024/02/10 13:33:44 gregh Exp gregh $

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


struct Interferometer : Module {
  float phase = 0.f;
  float blinkPhase = 0.f;
  static const int TRIG_OFF = 0;
  int exciter=0;
  int delay_fractional=0;
  //static const int TRIG_ON = 1;

  // size of the buffer used for the string
  static const int   BUF_SIZE = 100000;
  // 
  static const int NOT_A_NOTE = 8675309;
   
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
    //float trigger_state = TRIG_OFF;
    float buffer[BUF_SIZE];
    // location in the buffer where the head is.
    int buf_head = 0;
    // trigger state
    int trig_state = 0;
    // create a biquad filter to control the feedback through the delay filter.
    float   delay_line_len = 100;
    rack::dsp::BiquadFilter delayFilter;
    rack::dsp::BiquadFilter dcFilter;
    float last_trig = 0.f;
    
    // dispersion filter
    bool dispersion_enabled = false;
    float Df0 = 0.f;        // dispersion filter delay and fundamental.
    float a1 = 0.0f;
    float curr_f0 = NOT_A_NOTE;  // current note frequency.
    static const int M = 8;
    AllpassFilter dispersionFilter[M];
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
          INFO("B = %f",B);
          // TODO: Real value of B.  Comes from Figure 7.
          update_dispersion_values(freq, eng[ch].M, B, (float)args.sampleRate, &eng[ch]);
          eng[ch].curr_f0 = freq;
          for (int j = 0; j < eng[ch].M; j++) {
            eng[ch].dispersionFilter[j].setParameter(eng[ch].a1);
          }
        }

        // bound it to 60 to 1kHz
        // TODO: Is this required?
        // The top note on a piano is 4186 Hz.  
        // At higher frequencies, the tuning becomes more quantized.
        //if (freq<10.0) freq=10.0;
        //if (freq>2000.0) freq=2000.0;

        // set the delay line length accordingly
        eng[ch].delay_line_len = args.sampleRate/freq;
        // retune due to dispersion filter delay at the primary frequency.
        if (eng[ch].dispersion_enabled) eng[ch].delay_line_len -= eng[ch].Df0;

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
        //y = 4.0f * soundboard[trig_state] + 1.0f;
        
        eng[ch].trig_state++;
        //if (trig_state >= delay_line_len) {
        //  trig_state = 0;
        //}
        // piano
        if ((eng[ch].trig_state - 1) >= soundboard_size) {
          eng[ch].trig_state = TRIG_OFF;
        }

      }
      // TODO: handle when gate goes away!

      if (delay_fractional == 0) {
      
        int tap = eng[ch].buf_head - eng[ch].delay_line_len ;
        if (tap < 0) tap += BUF_SIZE;
        // run the delay filter and decay
        co += (1.0f - ch_decay) * eng[ch].delayFilter.process(eng[ch].buffer[tap]);
        eng[ch].dispersion_enabled = false;
      } else if (delay_fractional == 1) {
      
        // handle non-integer delay line value (naively), without sync()
        int tap1 = floor(eng[ch].delay_line_len);
        int tap2 = tap1 + 1.0f;
        int loc1 = eng[ch].buf_head - tap1;
        int loc2 = eng[ch].buf_head - tap2;
        if (loc1 < 0) loc1 += BUF_SIZE;
        if (loc2 < 0) loc2 += BUF_SIZE;
        float ratio2 = eng[ch].delay_line_len - tap1;
        float ratio1 = 1.0f - ratio2;
        float feedback_val = eng[ch].buffer[loc1] * ratio1 + eng[ch].buffer[loc2] * ratio2;
        co += (1.0f - ch_decay) * eng[ch].delayFilter.process(feedback_val);
        for (int j = 0; j < eng[ch].M; j++) {
          co = eng[ch].dispersionFilter[j].process(co);
        }
        eng[ch].dispersion_enabled = true;
        
      } else if (delay_fractional == 2) {
      
        // TODO:
        // Ruahala's paper DISPERSION MODELING IN WAVEGUIDE PIANO 
        // SYNTHESIS USING TUNABLE ALLPASS FILTERS alludes to 
        // using am allpass filter to get a delay between 1 and 2. 
        // "The tuning filter was implemented 
        // with a first-order Thiran allpass filter, and a delay of 
        // one sample was moved from the delay line to the tuning 
        // filter in order to have the fractional delay parameter 
        // in the range from 1 to 2."
        // So the method described by 
        // https://www.mathworks.com/help/dsp/ug/design-of-
        //         fractional-delay-fir-filters.html
        // might not be the right way to go.
        int tap1 = floor(eng[ch].delay_line_len);
        int tap2 = tap1 + 1;
        int tap3 = tap1 - 1;
        int tap4 = tap1 - 2;
        int tap5 = tap1 + 2;
        int loc1 = eng[ch].buf_head - tap1;
        int loc2 = eng[ch].buf_head - tap2;
        int loc3 = eng[ch].buf_head - tap3;
        int loc4 = eng[ch].buf_head - tap4;
        int loc5 = eng[ch].buf_head - tap5;
        
        // loc1 could be both overflow and underflow
        if (loc1 < 0) loc1 += BUF_SIZE;
        if (loc1 >= BUF_SIZE) loc1 -= BUF_SIZE;
        // loc2 (strictly add) could be overflow
        if (loc2 < 0) loc2 += BUF_SIZE;
        if (loc2 >= BUF_SIZE) loc2 -= BUF_SIZE;
        // loc3 (strictly subract) could (only) be underflow
        if (loc3 < 0) loc3 += BUF_SIZE;
        if (loc3 >= BUF_SIZE) loc3 -= BUF_SIZE;
        if (loc4 < 0) loc4 += BUF_SIZE;
        if (loc4 >= BUF_SIZE) loc4 -= BUF_SIZE;
        if (loc5 < 0) loc5 += BUF_SIZE;
        if (loc5 >= BUF_SIZE) loc5 -= BUF_SIZE;
        
        float ratio1;
        if (tap1-eng[ch].delay_line_len == 0.0)
          ratio1 = 1.0;
        else 
          ratio1 = sin((tap1-eng[ch].delay_line_len)*M_PI) / ((tap1-eng[ch].delay_line_len)*M_PI);
        float ratio2 = sin((tap2-eng[ch].delay_line_len)*M_PI) / ((tap2-eng[ch].delay_line_len)*M_PI);
        float ratio3 = sin((tap3-eng[ch].delay_line_len)*M_PI) / ((tap3-eng[ch].delay_line_len)*M_PI);
        float ratio4 = sin((tap4-eng[ch].delay_line_len)*M_PI) / ((tap4-eng[ch].delay_line_len)*M_PI);
        float ratio5 = sin((tap5-eng[ch].delay_line_len)*M_PI) / ((tap5-eng[ch].delay_line_len)*M_PI);
        
        float sum = ratio1 + ratio2 + ratio3 + ratio4 + ratio5;
        ratio1 = ratio1/sum;
        ratio2 = ratio2/sum;
        ratio3 = ratio3/sum;
        ratio4 = ratio4/sum;
        ratio5 = ratio5/sum;
        
        // all the above would only need to be updated if the frequency is updated.
        
        
        float feedback_val = eng[ch].buffer[loc1] * ratio1 + eng[ch].buffer[loc2] * ratio2 + 
                             eng[ch].buffer[loc3] * ratio3 + eng[ch].buffer[loc4] * ratio4 + 
                             eng[ch].buffer[loc5] * ratio5;
                             
        co += (1.0f - ch_decay) * eng[ch].delayFilter.process(feedback_val);
      }
      
      // apply that output DC block filter
      co = eng[ch].dcFilter.process(co);
      
      // store this channel's output at the head of the delay line buffer
      eng[ch].buffer[eng[ch].buf_head] = co;
      
      // sum this channel's output into the master output
      y += co;
      
      // update the head before we leave.
      eng[ch].buf_head = (eng[ch].buf_head+1) % BUF_SIZE;
      eng[ch].dispersion_enabled = false;
    }
    
    // clamp outputs then output.
    //y = math::clamp(-9.f, y, 9.f);
    //y = master_dcFilter.process(y);
    
    // output the master voltage.
    outputs[OUT_OUTPUT].setVoltage(y);
    
  }
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
    INFO("update dispersion D %f", D);
    float a1 = (1.f - D)/(1.f + D); // D >= 0, so a1 >= 0
    // don't allow a1 above 0.
    if (a1 >= 0.f) a1 = 0.f;
    
    // the fundamental delay is not presented in Ruahala's paper.
    float Df0 = std::atan(std::sin(wT)/(a1      + std::cos(wT))) / wT - 
                std::atan(std::sin(wT)/(1.0f/a1 + std::cos(wT))) / wT;
    Df0 *= M;  // the above delay is for ONE filter, we cascade M of them.
    // the does align with figure 9 in the paper.
    
    INFO("update dispersion f %f, Df0 %f, a1 %f", f0, Df0, a1);
    e->Df0 = Df0; // store the delay in the engine data.
    e->a1 = a1;   // store the filter coefficient in the engine data.
    // TODO: Update the filter coefficients.
  }
  
  void onReset(const ResetEvent& e) override {
    // Reset all parameters
    Module::onReset(e);   
    for (int ch = 0; ch < POLY_NUM; ch++) {
      // stop any not initiation
      eng[ch].trig_state = 0;
      // clear the delay buffer
      for (int i = 0; i < BUF_SIZE; i++) eng[ch].buffer[i] = 0.f;
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
    menu->addChild(createIndexPtrSubmenuItem("Exciter",
	    {"Piano", "Decay Exponential"},
	    &module->exciter));
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

