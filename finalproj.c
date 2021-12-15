
#include "config_1_3_2.h"
#include "pt_cornell_1_3_2_python.h"
#include "port_expander_brl4.h"
#include "tft_master.h"
#include "tft_gfx.h"
#include <stdlib.h>
#include <math.h>
#include <stdfix.h>


#define start_spi2_critical_section INTEnable(INT_T2, 0);
#define end_spi2_critical_section INTEnable(INT_T2, 1);


//Can also make stereo outputs!! by splitting signal across DAC A&B
#define DAC_config_chan_A 0b0011000000000000
#define DAC_config_chan_B 0b1011000000000000

// Audio sampling frequency
#define Fs 44100.0

enum effect_t {
    bypass_effect = 0,
    delay_effect,
    distortion_effect,
    flanger_effect
};

enum effect_t mode = bypass_effect;
volatile short out = 0; // The outputted audio value

volatile float mix_param = 1.0;

// ADC 
volatile int adc_0, adc_1, adc_5, adc_11;
const float MAX_POT_VAL = 1024 >> 2;

// Delay array variables
volatile int delay_samples = 11000;
volatile int averaged_delay_samples = 11000;
const int min_delay_samples = 1000;

// Delay array
#define array_size 11000
volatile short adc_arr[array_size];
volatile int curr_adc_array_idx = 0;
volatile int adc_arr_filled_to = 0;

#define delay_averager_size 8
volatile int delay_averager[delay_averager_size];
volatile int curr_delay_idx = 0;
volatile int delay_averager_filled_to = 0;

// Delay parameters
volatile float delay_feedback_param = 0.0;

// Distortion parameters
volatile int distortion_limit = 512;
volatile int distortion_cutoff_freq = 1000;

// Flanger parameters
volatile float flanger_feedback_param = 0.0;
volatile float flanger_rate = 1.0; 
volatile float flanger_depth = 0.95;

const float flanger_max_rate = 5.0; // Max rate in seconds
const float flanger_max_depth = 0.9;  // Depth can go + or - this coeff times base rate

#define flanger_base_delay_time 15 // in ms
const int flanger_base_delay_diff = Fs / (1000 / flanger_base_delay_time); // how many samples back to look for delay

// Sine table for flanger
#define sine_table_size 256
volatile _Accum sine_table[sine_table_size];
volatile int sine_idx = 0;
volatile int sample_num = 0;

volatile _Accum running_avg = 0;

// For distortion low pass filter
float outcontd = 0;
float wt, a;

float sampling_time = 0.00002272727; // (1/44000)

// Buttons
volatile int delay_button_down = 0;
volatile int distortion_button_down = 0;
volatile int flanger_button_down = 0;

#define DELAY_LED_PIN BIT_14
#define DISTORTION_LED_PIN BIT_2
#define FLANGER_LED_PIN BIT_10

#define DELAY_BUTTON_PIN BIT_7
#define DISTORTION_BUTTON_PIN BIT_8
#define FLANGER_BUTTON_PIN BIT_11

int true_avg = 0;

//Do audio processing here
void __ISR(_TIMER_2_VECTOR, ipl2) Timer2Handler(void)
{
    mT2ClearIntFlag();
    //sample ADC
    adc_0 = ReadADC10(0);   // read the result of channel 9 conversion from the idle buffer
    
    int i;
    out = adc_0;
	
    // Increment the sample we are at, for indexing into the flanger sine table
    sample_num++;
    
    int delay_idx;
    switch (mode){
    case bypass_effect: // Clean signal
	// out value is already set to be equal to the input adc signal above
        break;
            
    case delay_effect: // Delay effect
        // Ensure the idx is valid and within the array. Note that this is the averaged value,
        // so the previous values of the delay time are averaged together to reduce noise from
        // the knob
        delay_idx = curr_adc_array_idx - averaged_delay_samples;
        while (delay_idx < 0) {
            delay_idx += array_size;
        }
        
        // Add to the new output the previous sample, weighted based on the mix param
        out = (adc_0 + adc_arr[delay_idx] * mix_param);       
        
        // If feedback knob is up, also have the delayed sound be fed back into the effect
        adc_arr[curr_adc_array_idx] =
                (adc_0 + (delay_feedback_param * adc_arr[delay_idx])) / (1.f + delay_feedback_param);  

        // So that the output (even with full mix) will never be too loud and cause the
        // DAC to distort
        out /= 2.0;
        break;
            
    case distortion_effect: { // Distortion effect
        // Found by hand tuning with an oscilloscope to find the approximate value of the
        // center of the waveform. Signal will be manipulated around a midpoint of 0
        float average = 373; 
        out -= average;

        // gain can be between 1x and 500x
        float distortion_gain = ((float)adc_5 * 500.0 / 1023) + 1.0;

        // The threshold at which the signal is capped		
        int limit = 256;

        out = distortion_gain * out;
        if (out > limit) {
            out = limit;
        } else if (out < -limit) {
            out = -limit;
        }

        // Return the new signal back to the original midpoint
        out += average;

        // Add low pass tone, knob controls adjustable cutoff frequency
        wt = distortion_cutoff_freq * sampling_time;
        a = wt / (1 + wt);
        outcontd = outcontd*(1 - a) + a*out;
        out = outcontd;

        // Within DAC range
        if (out > 1023) {
            out = 1023;
        } else if (out < 0) {
            out = 0;
        }

        // How much distortion signal vs how much clean signal
        out = (1.0 - mix_param) * adc_0 + (out * mix_param);
            
        break;
            
    } case flanger_effect: { // Flanger
        // How many samples back to look, modulated by sine, with amplitude of the depth,
        // and centered around the base delay value
        int delay_diff = flanger_base_delay_diff;
        delay_diff += flanger_depth * delay_diff * sine_table[sine_idx];
            
        // Iterate the index in the sine table so there is one period per flanger_rate seconds
        if (sample_num >= flanger_rate * Fs / sine_table_size) {
            sample_num = 0;
            sine_idx++;
            if (sine_idx >= sine_table_size) {
                sine_idx = 0;
            }
        }

        // Now that we have modulated how far back we look, the rest of the effect functions
        // similar to the delay effect

        delay_idx = curr_adc_array_idx - delay_diff;
        while (delay_idx < 0) {
            delay_idx += array_size;
        }

        out = (adc_0 + adc_arr[delay_idx] * mix_param);
        adc_arr[curr_adc_array_idx] = adc_0;  
        out /= 2.0;
        
        // Adjust mix
        out = (1.0 - mix_param) * adc_0 + (out * mix_param);
        
        break;
            
    }
    }

    // Increment where in the adc array we look
    curr_adc_array_idx = (curr_adc_array_idx + 1);
    if (curr_adc_array_idx == array_size) {
        curr_adc_array_idx = 0;
    }
    
    // Write the output to the DAC
    mPORTBClearBits(BIT_4); // start transaction
    WriteSPI2( DAC_config_chan_A | (((int)out << 2))); //idk if the 0xfff is necessary tbh
    while (SPI2STATbits.SPIBUSY) {}; // wait for end of transaction
    mPORTBSetBits(BIT_4) ; // end transaction
}

// A function to update all of the LEDs based on the button that was pressed.
// All lights of other effects are set to off. The LED of thespecified effect
// is only turned on if it was inactive (and is now active); otherwise, it too
// is turned off, for the pedal will by in bypass mode.
//
// @param effect The effect corresponding with a button that was just pressed
void setLEDs(enum effect_t effect) {
    if (effect == delay_effect) {
        mPORTBSetBits(DELAY_LED_PIN);
    } else {
        mPORTBClearBits(DELAY_LED_PIN); 
    }
    
    if (effect == distortion_effect) {
        mPORTBSetBits(DISTORTION_LED_PIN);
    } else {
        mPORTBClearBits(DISTORTION_LED_PIN);
    }
    
    if (effect == flanger_effect) {
        mPORTBSetBits(FLANGER_LED_PIN);
    } else {
        mPORTBClearBits(FLANGER_LED_PIN);
    }
}

// Process the button inputs to toggle each effect. This also ensures that only one
// will be active at a time. Toggling occurs on button release
static PT_THREAD (protothread_buttons(struct pt *pt)){
    PT_BEGIN(pt);
    static int val = 0;
    while(1){
        PT_YIELD_TIME_msec(100);
        
        unsigned int bits = mPORTBReadBits(DELAY_BUTTON_PIN | DISTORTION_BUTTON_PIN | FLANGER_BUTTON_PIN);
        
        unsigned int delay_down_now = (bits & DELAY_BUTTON_PIN);
        unsigned int distortion_down_now = (bits & DISTORTION_BUTTON_PIN);
        unsigned int flanger_down_now = (bits & FLANGER_BUTTON_PIN);
        
        if (delay_button_down && !delay_down_now) { // Delay button released
            if (mode == delay_effect) {
                mode = bypass_effect;
            } else {
                mode = delay_effect;
            }
        } else if (distortion_button_down && !distortion_down_now) { // Distortion button released
            if (mode == distortion_effect) {
                mode = bypass_effect;
            } else {
                mode = distortion_effect;
            }
        } else if (flanger_button_down && !flanger_down_now) { // Distortion button released
            if (mode == flanger_effect) {
                mode = bypass_effect;
            } else {
                mode = flanger_effect;
            }
        }
        
        // Update LEDs based on the released button
        setLEDs(mode);
        
        // Save the button pressed
        delay_button_down = delay_down_now;
        distortion_button_down = distortion_down_now;
        flanger_button_down = flanger_down_now;
    }
    PT_END(pt);
}

// Read in non-audio values from the adc, for reading potentiometer (knob) values
static PT_THREAD (protothread_adc(struct pt *pt)){
    PT_BEGIN(pt);
    static int val = 0;
    while(1) {
        PT_YIELD_TIME_msec(100);
        
        // Low pass the adc values by rightshifting by 2. This removes noise which would otherwise
        // randomly alter existing effects. Note that the values are also inverted, so that a higher
        // value comes when the knob is turned to the right, as is common with most pedals
        adc_1 = MAX_POT_VAL - (ReadADC10(1) >> 2);
        adc_5 = MAX_POT_VAL - (ReadADC10(2) >> 2);
        adc_11 = MAX_POT_VAL - (ReadADC10(3) >> 2);
        
        mix_param = (float)adc_1 / MAX_POT_VAL;
        
        // For delay
        delay_feedback_param = (float)adc_5 / MAX_POT_VAL;
        delay_samples = (adc_11 * (array_size - min_delay_samples) / MAX_POT_VAL) + min_delay_samples;
        
        // For distortion
        distortion_limit = (adc_5 < 1 ? 1 : adc_5);
        distortion_cutoff_freq = (adc_11 * 100) + 2048;
        
        // For flanger
        flanger_rate = flanger_max_rate * adc_5 / 1023.0;
        flanger_depth = flanger_max_depth * adc_11 / 1023.0;
        delay_averager[curr_delay_idx] = delay_samples;
        
        // Increment the index in the averager, returning to 0 if needed
        curr_delay_idx++;
        if (curr_delay_idx >= delay_averager_size) {
            curr_delay_idx = 0;
        }
        
        // Ensure we record how filled the array is, so that we don't use
        // uninitialized elements before they have first been written to the
        // array
        if (delay_averager_filled_to < curr_delay_idx) {
            delay_averager_filled_to++;
        }
        
        // Average the previous delay time values together. This ensures that if the noise of the
        // knob causes the value to change randomly by small amounts, there is less likely to be
        // noticeable audio artifacts from the delay time changing.
        if (delay_averager_filled_to > 0) {
            averaged_delay_samples = 0;
            int i;
            for (i = 0; i < delay_averager_filled_to; i++) {
                averaged_delay_samples += delay_averager[i];
            }
            averaged_delay_samples /= delay_averager_filled_to;
        } else {
            averaged_delay_samples = delay_samples;
        }
    }
    PT_END(pt);
}

void main(void) {
    ANSELA = 0; ANSELB = 0; 
    int generate_period = (int)sys_clock/Fs;
    generate_period = 2000; //change this just so that the program doesn't crash
    OpenTimer2(T2_ON | T2_SOURCE_INT | T2_PS_1_1, generate_period);
    ConfigIntTimer2(T2_INT_ON | T2_INT_PRIOR_2);
    mT2ClearIntFlag(); // and clear the interrupt flag
    
    PPSOutput(2, RPB5, SDO2);
    
// Initialize the sine table for the flanger modulation effect
    int i;
    for (i = 0; i < sine_table_size; i++) {
        sine_table[i] = (_Accum)(sin((float)i*6.283/(float)sine_table_size));
    }
    
// Setup the ADC
    // configure and enable the ADC
    CloseADC10();    // ensure the ADC is off before setting the configuration

    // define setup parameters for OpenADC10
    // Turn module on | ouput in integer | trigger mode auto | enable autosample
    // ADC_CLK_AUTO -- Internal counter ends sampling and starts conversion (Auto convert)
    // ADC_AUTO_SAMPLING_ON -- Sampling begins immediately after last conversion completes; SAMP bit is automatically set
    // ADC_AUTO_SAMPLING_OFF -- Sampling begins with AcquireADC10();
    #define PARAM1  ADC_FORMAT_INTG16 | ADC_CLK_AUTO | ADC_AUTO_SAMPLING_ON //

    // define setup parameters for OpenADC10
    // ADC ref external  | disable offset test | disable scan mode | do 1 sample | use single buf | alternate mode off
    #define PARAM2  ADC_VREF_AVDD_AVSS | ADC_OFFSET_CAL_DISABLE | ADC_SCAN_ON | ADC_SAMPLES_PER_INT_4 | ADC_ALT_BUF_OFF | ADC_ALT_INPUT_OFF
    //
    // Define setup parameters for OpenADC10
    // use peripherial bus clock | set sample time | set ADC clock divider
    // ADC_CONV_CLK_Tcy2 means divide CLK_PB by 2 (max speed)
    // ADC_SAMPLE_TIME_5 seems to work with a source resistance < 1kohm
    #define PARAM3 ADC_CONV_CLK_PB | ADC_SAMPLE_TIME_15 | ADC_CONV_CLK_Tcy //ADC_SAMPLE_TIME_15| ADC_CONV_CLK_Tcy2

    // define setup parameters for OpenADC10
    // set AN11 and  as analog inputs
    #define PARAM4 ENABLE_AN0_ANA | ENABLE_AN1_ANA | ENABLE_AN5_ANA | ENABLE_AN11_ANA // pin 24, 25 (RB13/14)

    // define setup parameters for OpenADC10
    #define PARAM5 SKIP_SCAN_AN2 | SKIP_SCAN_AN3 | SKIP_SCAN_AN4 | SKIP_SCAN_AN6 | SKIP_SCAN_AN7 | SKIP_SCAN_AN8 | SKIP_SCAN_AN9 | SKIP_SCAN_AN10 | SKIP_SCAN_AN12 | SKIP_SCAN_AN13 | SKIP_SCAN_AN14 | SKIP_SCAN_AN15

    // use ground as neg ref for A
    SetChanADC10( ADC_CH0_NEG_SAMPLEA_NVREF); 
    OpenADC10( PARAM1, PARAM2, PARAM3, PARAM4, PARAM5 ); // configure ADC using the parameters defined above

    EnableADC10(); // Enable the ADC

    
// Seup the DAC
    PPSOutput(2, RPB5, SDO2);
    SpiChnOpen(SPI_CHANNEL2, SPI_OPEN_ON | SPI_OPEN_MODE16 | SPI_OPEN_MSTEN | SPI_OPEN_CKE_REV , 4);
    mPORTBSetPinsDigitalOut(BIT_4);
    mPORTBSetBits(BIT_4);
    

// Setup the LEDs
    mPORTBSetPinsDigitalOut(DELAY_LED_PIN | DISTORTION_LED_PIN | FLANGER_LED_PIN);
    

// Setup the buttons
    mPORTBSetPinsDigitalIn(DELAY_BUTTON_PIN | DISTORTION_BUTTON_PIN | FLANGER_BUTTON_PIN);
    

// Configure the threads
    PT_setup();

    // === identify the threads to the scheduler =====
    // add the thread function pointers to be scheduled
    // --- Two parameters: function_name and rate. ---
    // rate=0 fastest, rate=1 half, rate=2 quarter, rate=3 eighth, rate=4 sixteenth,
    // rate=5 or greater DISABLE thread!

    pt_add(protothread_adc, 0);
    pt_add(protothread_buttons, 0);

    // Initialize the scheduler
    PT_INIT(&pt_sched) ;

    pt_sched_method = SCHED_ROUND_ROBIN ;

    // Schedule threads: (scheduler never exits)
    PT_SCHEDULE(protothread_sched(&pt_sched));

} // main
