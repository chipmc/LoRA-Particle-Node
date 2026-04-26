/*
 * @file take_measurements.h
 * @author Chip McClelland (chip@seeinisghts.com)
 * @brief This function will take measurements at intervals defined int he sleep_helper_config file.  
 * The libraries and functions needed will depend the spectifics of the device and sensors
 * 
 * @version 0.2
 * @date 2022-12-23
 * 
 */

// Particle functions
#ifndef TAKE_MEASUREMENTS_H
#define TAKE_MEASUREMENTS_H

#include "Particle.h"

extern char internalTempStr[16];                       // External as this can be called as a Particle variable
extern char signalStr[64];

/**
 * @brief This code collects basic data from the default sensors - TMP-36 (inside temp), battery charge level and signal strength
 * 
 * @details Uses an analog input and the appropriate scaling
 * 
 * @returns Returns true if succesful and puts the data into the current object
 * 
 */
bool takeMeasurements();                               // Function that calls the needed functions in turn

/**
 * @brief tmp36TemperatureC
 * 
 * @details generic code that convers the analog value of the TMP-36 sensors to degrees c - Assume 3.3V Power
 * 
 * @returns Returns the temperature in C as a float value
 * 
 */
float tmp36TemperatureC (int adcValue);                // Temperature from the tmp36 - inside the enclosure


/**
 * @brief Get the Signal Strength values and make up a string for use in the console
 * 
 * @details Provides data on the signal strength and quality
 * 
 */
void getSignalStrength();

/**
 * @brief This function is called once a hardware interrupt is triggered by the device's sensor
 * 
 * @details The sensor may change based on the settings in sysSettings but the overall concept of operations
 * is the same regardless.  The sensor will trigger an interrupt, which will set a flag. In the main loop
 * that flag will call this function which will determine if this event should "count" as a visitor.
 * 
 * @returns true so we can reset the interrupt flag
 * 
 */
bool recordCount();

/**
 * @brief soft delay let's us process Particle functions and service the sensor interrupts while pausing
 * 
 * @details takes a single unsigned long input in millis
 * 
 */
void softDelay(uint32_t t);                 		// Soft delay is safer than delay

#endif