# Parkinsonian Tremor Detector  
*Semester Embedded Systems Challenge*  

## Overview  

Over **1 million people in the USA** and more than **10 million people worldwide** suffer from **Parkinson’s disease**. A critical challenge in managing Parkinson’s is the **clinically accurate detection of symptoms** to enable therapy optimization.  

One of the most common symptoms is the **resting tremor**, which affects over **70% of patients**. Resting tremors occur when a body part (typically the hand or wrist) is completely supported and at rest, and they are **minimal or absent during voluntary activity**.  
The **classical Parkinsonian tremor** typically occurs at **3 to 6 Hz** (cycles per second).  

This project focuses on **developing a wearable Parkinsonian tremor detector** using only the **STM32F429 Discovery board** with its **embedded gyroscope (L3GD20)**.  

## Objective  

The goal of this challenge is to:  
✅ **Capture real-time rotation data** using the onboard gyroscope by measuring angular velocities.  
✅ **Analyze time-segmented data** to detect tremor patterns within the target frequency range (3-6 Hz).  
✅ **Provide a visual indication** of the presence and intensity of resting tremors using available board resources (e.g., LEDs, LCD screen).  

### Constraints  
- **No additional hardware** is allowed—use only the STM32F429 Discovery board and its built-in sensors and display resources.  

## Key Features  
- **Tremor Detection**: Identify the presence of resting tremors based on frequency analysis.  
- **Intensity Estimation**: Indicate the severity or strength of the tremor.  
- **Real-time Feedback**: Display visual cues for tremor status using the board's LEDs or LCD.  

## Tools & Resources  
- **STM32F429 Discovery Board**  
- **Onboard Gyroscope (L3GD20)**  
- **C programming & embedded systems knowledge**  
- **Signal processing techniques (e.g., FFT, filtering)**  

## Notes  
- This challenge emphasizes **embedded programming, signal analysis, and real-time processing**.  
- Explore creative ways to visualize tremor data using the limited resources of the board.  
- Consider using filtering techniques to isolate the target frequency range (3-6 Hz) from noise.  

Let’s build a meaningful solution that could make a real difference in Parkinson’s disease management!
