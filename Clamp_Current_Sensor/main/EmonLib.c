/*
  Emon.cpp - Library for openenergymonitor
  Created by Trystan Lea, April 27 2010
  GNU GPL
  modified to use up to 12 bits ADC resolution (ex. Arduino Due)
  by boredman@boredomprojects.net 26.12.2013
  Low Pass filter for offset removal replaces HP filter 1/1/2015 - RW
*/

// Proboscide99 10/08/2016 - Added ADMUX settings for ATmega1284 e 1284P (644 / 644P also, but not tested) in readVcc function

#include <math.h>

#include "EmonLib.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"

static const char TAG[] = "EmonLib";

//ADC Attenuation
#define ADC_EXAMPLE_ATTEN           ADC_ATTEN_DB_12

//ADC Calibration
#if CONFIG_IDF_TARGET_ESP32
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_VREF
#elif CONFIG_IDF_TARGET_ESP32S2
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32C3
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP
#elif CONFIG_IDF_TARGET_ESP32S3
#define ADC_EXAMPLE_CALI_SCHEME     ESP_ADC_CAL_VAL_EFUSE_TP_FIT
#endif

static esp_adc_cal_characteristics_t adc1_chars;

double realPower, apparentPower, powerFactor, Vrms, Irms;

//Calibration coefficients
//These need to be set in order to obtain accurate results
double VCAL;
double ICAL;
double PHASECAL;

//--------------------------------------------------------------------------------------
// Variable declaration for emon_calc procedure
//--------------------------------------------------------------------------------------
double lastFilteredV, filteredV;         //Filtered_ is the raw analog value minus the DC offset
double filteredI;
double offsetV;                          //Low-pass filter output
double offsetI;                          //Low-pass filter output

double phaseShiftedV;                     //Holds the calibrated phase shifted voltage.

double sqV, sumV, sqI, sumI, instP, sumP; //sq = squared, sum = Sum, inst = instantaneous

int startV;                               //Instantaneous voltage at start of sample window.

bool lastVCross, checkVCross;             //Used to measure number of times threshold is crossed.

static bool adc_calibration_init(void)
{
    esp_err_t ret;
    bool cali_enable = false;

    ret = esp_adc_cal_check_efuse(ADC_EXAMPLE_CALI_SCHEME);
    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGE(TAG, "Calibration scheme not supported, skip software calibration");
    } else if (ret == ESP_ERR_INVALID_VERSION) {
        ESP_LOGE(TAG, "eFuse not burnt, skip software calibration");
    } else if (ret == ESP_OK) {
        cali_enable = true;
        esp_adc_cal_characterize(ADC_UNIT_1, ADC_EXAMPLE_ATTEN, ADC_WIDTH_BIT_DEFAULT, 0, &adc1_chars);
    } else {
        ESP_LOGE(TAG, "Invalid arg");
    }

    return cali_enable;
}

void EnergyMonitor_init(void) {
  // Init ADC
  bool cali_enable = adc_calibration_init();

  //ADC1 config
  ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_DEFAULT));
  ESP_ERROR_CHECK(adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_EXAMPLE_ATTEN));

  ESP_LOGE(TAG, "Calib status: %d", cali_enable);
}

//--------------------------------------------------------------------------------------
// Sets the pins to be used for voltage and current sensors
//--------------------------------------------------------------------------------------
void EnergyMonitor_voltageCalib(double _VCAL, double _PHASECAL)
{
  VCAL = _VCAL;
  PHASECAL = _PHASECAL;
  offsetV = (ADC_COUNTS >> 1);
}

void EnergyMonitor_currentCalib(double _ICAL)
{
  ICAL = _ICAL;
  offsetI = (ADC_COUNTS >> 1);
  int adjust = 0;

  while (adjust < 250)
  {
    EnergyMonitor_calcIrms(1480);
    adjust++;
  }

  ESP_LOGE(TAG, "EnergyMonitor_currentCalib: adjust = %d", adjust);
}

uint32_t millis(void) {
  return 1;
}

//--------------------------------------------------------------------------------------
// emon_calc procedure
// Calculates realPower,apparentPower,powerFactor,Vrms,Irms,kWh increment
// From a sample window of the mains AC voltage and current.
// The Sample window length is defined by the number of half wavelengths or crossings we choose to measure.
//--------------------------------------------------------------------------------------
void EnergyMonitor_calcVI(unsigned int crossings, unsigned int timeout)
{
  int sampleV; //sample_ holds the raw analog read value
  int sampleI;
  int filteredI;
  int SupplyVoltage = 3300; // mV

  unsigned int crossCount = 0;                             //Used to measure number of times threshold is crossed.
  unsigned int numberOfSamples = 0;                        //This is now incremented

  //-------------------------------------------------------------------------------------------------------------------------
  // 1) Waits for the waveform to be close to 'zero' (mid-scale adc) part in sin curve.
  //-------------------------------------------------------------------------------------------------------------------------
  unsigned long start = millis();    //millis()-start makes sure it doesnt get stuck in the loop if there is an error.

  while(1)                                   //the while loop...
  {
    startV = adc1_get_raw(ADC1_CHANNEL_2);  //using the voltage waveform
    if ((startV < (ADC_COUNTS * 0.55)) && (startV > (ADC_COUNTS * 0.45))) break;  //check its within range
    if ((millis() - start) > timeout) break;
  }

  //-------------------------------------------------------------------------------------------------------------------------
  // 2) Main measurement loop
  //-------------------------------------------------------------------------------------------------------------------------
  start = millis();

  while ((crossCount < crossings) && ((millis() - start) < timeout))
  {
    numberOfSamples++;                       //Count number of times looped.
    lastFilteredV = filteredV;               //Used for delay/phase compensation

    //-----------------------------------------------------------------------------
    // A) Read in raw voltage and current samples
    //-----------------------------------------------------------------------------
    sampleV = adc1_get_raw(ADC1_CHANNEL_2);                 //Read in raw voltage signal
    sampleI = adc1_get_raw(ADC1_CHANNEL_2);                 //Read in raw current signal

    //-----------------------------------------------------------------------------
    // B) Apply digital low pass filters to extract the 2.5 V or 1.65 V dc offset,
    //     then subtract this - signal is now centred on 0 counts.
    //-----------------------------------------------------------------------------
    offsetV = offsetV + ((sampleV - offsetV) / 1024);
    filteredV = sampleV - offsetV;
    offsetI = offsetI + ((sampleI - offsetI) / 1024);
    filteredI = sampleI - offsetI;

    //-----------------------------------------------------------------------------
    // C) Root-mean-square method voltage
    //-----------------------------------------------------------------------------
    sqV= filteredV * filteredV;                 //1) square voltage values
    sumV += sqV;                                //2) sum

    //-----------------------------------------------------------------------------
    // D) Root-mean-square method current
    //-----------------------------------------------------------------------------
    sqI = filteredI * filteredI;                //1) square current values
    sumI += sqI;                                //2) sum

    //-----------------------------------------------------------------------------
    // E) Phase calibration
    //-----------------------------------------------------------------------------
    phaseShiftedV = lastFilteredV + PHASECAL * (filteredV - lastFilteredV);

    //-----------------------------------------------------------------------------
    // F) Instantaneous power calc
    //-----------------------------------------------------------------------------
    instP = phaseShiftedV * filteredI;          //Instantaneous Power
    sumP += instP;                               //Sum

    //-----------------------------------------------------------------------------
    // G) Find the number of times the voltage has crossed the initial voltage
    //    - every 2 crosses we will have sampled 1 wavelength
    //    - so this method allows us to sample an integer number of half wavelengths which increases accuracy
    //-----------------------------------------------------------------------------
    lastVCross = checkVCross;
    if (sampleV > startV) checkVCross = true;
                     else checkVCross = false;
    if (numberOfSamples==1) lastVCross = checkVCross;

    if (lastVCross != checkVCross) crossCount++;
  }

  //-------------------------------------------------------------------------------------------------------------------------
  // 3) Post loop calculations
  //-------------------------------------------------------------------------------------------------------------------------
  //Calculation of the root of the mean of the voltage and current squared (rms)
  //Calibration coefficients applied.

  double V_RATIO = VCAL * ((SupplyVoltage / 1000.0) / (ADC_COUNTS));
  Vrms = V_RATIO * sqrt(sumV / numberOfSamples);

  double I_RATIO = ICAL * ((SupplyVoltage / 1000.0) / (ADC_COUNTS));
  Irms = I_RATIO * sqrt(sumI / numberOfSamples);

  //Calculation power values
  realPower = V_RATIO * I_RATIO * sumP / numberOfSamples;
  apparentPower = Vrms * Irms;
  powerFactor=realPower / apparentPower;

  //Reset accumulators
  sumV = 0;
  sumI = 0;
  sumP = 0;
//--------------------------------------------------------------------------------------
}

//--------------------------------------------------------------------------------------
double EnergyMonitor_calcIrms(unsigned int Number_of_Samples)
{
  int sampleI = 0; //sample_ holds the raw analog read value
  int SupplyVoltage = 3300; //mV

  for (unsigned int n = 0; n < Number_of_Samples; n++)
  {
    sampleI = adc1_get_raw(ADC1_CHANNEL_2);

    // Digital low pass filter extracts the 2.5 V or 1.65 V dc offset,
    // then subtract this - signal is now centered on 0 counts.
    offsetI = (offsetI + (sampleI - offsetI) / ADC_COUNTS);
    filteredI = sampleI - offsetI;

    // Root-mean-square method current
    // 1) square current values
    sqI = filteredI * filteredI;

    // 2) sum
    sumI += sqI;
  }

  ESP_LOGE(TAG, "adc1_get_raw: sampleI = %d", sampleI);

  double I_RATIO = ICAL * ((SupplyVoltage / 1000.0) / (ADC_COUNTS));

  Irms = I_RATIO * sqrt(sumI / Number_of_Samples);

  if (Irms < 1.0) {
    Irms = 0;
  }

  // Reset accumulators
  sumI = 0;
  
  return Irms;
}

void EnergyMonitor_serialprint(void)
{
  ESP_LOGE(TAG, "Current value: %d", (uint32_t)(Irms * 1000));
}

