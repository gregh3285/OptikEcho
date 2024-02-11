#include "plugin.hpp"
#include <iostream>
#include <fstream>
#include <cstring>
#include <stdint.h>
#include <dsp/filter.hpp>

// $Id: Interferometer.cpp,v 1.4 2024/02/10 13:33:44 gregh Exp gregh $

int load(float *extbuff); // load the soundboard file.

struct Interferometer : Module {
  float phase = 0.f;
  float blinkPhase = 0.f;
  const int TRIG_OFF = 0;
  const int TRIG_ON = 1;
  float trigger_state = TRIG_OFF;
  int   delay_line_len = 100;

  // size of the buffer used for the string
  static const int BUF_SIZE = 100000;
  float buffer[BUF_SIZE];
  
  
  // location in the buffer where the head is.
  int buf_head = 0;
  // trigger state
  int trig_state = 0;
  
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
  
  // create a biquad filter to control the feedback through the delay filter.
  rack::dsp::BiquadFilter delayFilter;
  rack::dsp::BiquadFilter dcFilter;

Interferometer() {
    config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
    configParam(DECAY_PARAM, 0.f, 1.f, 0.f, "");
    configParam(DELAY_FEEDBACK_FREQ_PARAM, 0.f, 0.49f, 0.f, "");
    configInput(VOCT_INPUT, "");
    configInput(TRIG_INPUT, "");
    configOutput(OUT_OUTPUT, "");
    
    
    // normalized frequency for filter is cutoff_freq/sample_rate. goes unstable above 0.5.
    // frequency, q, gain_labeled_as_v.  
    delayFilter.setParameters(rack::dsp::BiquadFilter::Type::LOWPASS, 0.3, 0.5, 0.0); 
    // highpass stuff above 4 or 5 Hz
    dcFilter.setParameters(rack::dsp::BiquadFilter::Type::HIGHPASS, 0.00001, 0.5, 0.0); 
    
    // load the soundboard exciter wave file.
    soundboard_size = load(soundboard);
    INFO("soundboard_size %d", soundboard_size);
    if (soundboard_size < 0) {
    
    }
  }

  void process(const ProcessArgs &args) override {

    float decay;
    float trigger;
    float y =0.f;

    // get the Karplus decay parameter from the knob.
    decay = params[DECAY_PARAM].getValue();
    
    // read the value and update the parameters of the biquad feedback filter.
    float feedback_filter_param  = params[DELAY_FEEDBACK_FREQ_PARAM].getValue();
    // Q critically damped is 0.5
    delayFilter.setParameters(rack::dsp::BiquadFilter::Type::LOWPASS, feedback_filter_param, 0.5, 0.0);

    // Blink light at 1Hz
    blinkPhase += args.sampleTime;
    if (blinkPhase >= 1.f) {
      blinkPhase -= 1.f;
    }
    lights[ACTIVE_LIGHT].setBrightness(blinkPhase < 0.5f ? 1.f : 0.f);

    trigger = inputs[TRIG_INPUT].getVoltage();
    if ((trigger_state == TRIG_OFF) && (trigger > 0.5)) {
      trigger_state = TRIG_ON;
      trig_state = 1; 

    } else if ((trigger_state == TRIG_ON) && (trigger < 0.1)) {
      trigger_state = TRIG_OFF;
    }

    if (trig_state != 0) {
      // sample the frequency at the point where we are triggering
      float pitch = inputs[VOCT_INPUT].getVoltage();

      // The default frequency is C4 = 261.6256f
      float freq = dsp::FREQ_C4 * std::pow(2.f, pitch);

      // bound it to 60 to 1kHz
      if (freq<10.0) freq=10.0;
      if (freq>2000.0) freq=2000.0;

      // set the delay line length accordingly
      delay_line_len = args.sampleRate/freq;

      // random
      //buffer[buf_head] = 5.0f * (random::uniform()-0.5f);
      // ramp
      //buffer[buf_head] = 5.0f * (trig_state/(float)delay_line_len - 0.5f);
      // piano
      y = 5.0f * soundboard[trig_state];
      //y = 4.0f * soundboard[trig_state] + 1.0f;
      
      trig_state++;
      //if (trig_state >= delay_line_len) {
      //  trig_state = 0;
      //}
      // piano
      if (trig_state >= soundboard_size) {
        trig_state = 0;
      }

    }

    int tap1 = buf_head - delay_line_len ;
    if (tap1 < 0) tap1 += BUF_SIZE;

    // run the delay filter and decay
    y += (1-decay) * delayFilter.process(buffer[tap1]);
    // apply that output DC block filter
    y = dcFilter.process(y);

    y = math::clamp(-9.f, y, 9.f);
    buffer[buf_head] = y;
    
    outputs[OUT_OUTPUT].setVoltage(y);

    buf_head = (buf_head+1) % BUF_SIZE;

  }
  
  void onReset(const ResetEvent& e) override {
    // Reset all parameters
    Module::onReset(e);   
    // stop any not initiation
    trig_state = 0;
    // clear the delay buffer
    for (int i = 0; i < BUF_SIZE; i++) buffer[i] = 0.f;
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
                extbuff[ebi++] = dat_chunk.data[i]*1.0/32768.0;
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

