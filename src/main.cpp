/*
Group Details to go here

Memeber 1: Saksham Madan (sm11875)
Memeber 2: 
Memeber 3:
Memeber 4:

*/

#include <mbed.h>
#include <arm_math.h>
#include <stdio.h>
#include <math.h> 

// Define control register addresses and their configurations
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
#define SPI_FLAG 1
#define OUT_X_L 0x28


EventFlags flags;

void spi_cb(int event)
{
    flags.set(SPI_FLAG);
}

#define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)
#define TREMOR_THRESHOLD 0.5f
#define ALPHA 0.5f  // The weight of the new value in the filtered value. Should be between 0 and 1.

#define SAMPLES 256  // Number of samples to collect before performing FFT
#define SAMPLING_FREQUENCY 100  // Sampling frequency in Hz
#define PI 3.14159265358979323846
#define N 256
#define SAMPLING_FREQ 100.0


float gx_samples[SAMPLES];
uint16_t sample_index = 0;


float filtered_gx = 0.0f;
float previous_gx = 0.0f;




struct cmpx                       //complex data structure used by FFT
    {
    float real;
    float imag;
    };
typedef struct cmpx COMPLEX;

void fft(COMPLEX *Y, int M, COMPLEX *w)       //input sample array, number of points
{
  COMPLEX temp1,temp2;            //temporary storage variables
  int i,j,k;                      //loop counter variables
  int upper_leg, lower_leg;       //index of upper/lower butterfly leg
  int leg_diff;                   //difference between upper/lower leg
  int num_stages=0;               //number of FFT stages, or iterations
  int index, step;                //index and step between twiddle factor
  i=1;                            //log(base 2) of # of points = # of stages
  do
  {
    num_stages+=1;
    i=i*2;
  } while (i!=M);

  leg_diff=M/2;                 //starting difference between upper & lower legs
  step=2;                     //step between values in twiddle.h              
  for (i=0;i<num_stages;i++)      //for M-point FFT                 
  {
    index=0;
    for (j=0;j<leg_diff;j++)
    {
      for (upper_leg=j;upper_leg<M;upper_leg+=(2*leg_diff))
      {
        lower_leg=upper_leg+leg_diff;
        temp1.real=(Y[upper_leg]).real + (Y[lower_leg]).real;
        temp1.imag=(Y[upper_leg]).imag + (Y[lower_leg]).imag;
        temp2.real=(Y[upper_leg]).real - (Y[lower_leg]).real;
        temp2.imag=(Y[upper_leg]).imag - (Y[lower_leg]).imag;
        (Y[lower_leg]).real=temp2.real*(w[index]).real-temp2.imag*(w[index]).imag;
        (Y[lower_leg]).imag=temp2.real*(w[index]).imag+temp2.imag*(w[index]).real;
        (Y[upper_leg]).real=temp1.real;
        (Y[upper_leg]).imag=temp1.imag;
      }
      index+=step;
    }
    leg_diff=leg_diff/2;
    step*=2;
  }
  j=0;
  for (i=1;i<(M-1);i++)           //bit reversal for resequencing data*/
  {
    k=M/2;
    while (k<=j)
    {
      j=j-k;
      k=k/2;
    }
    j=j+k;
    if (i<j)
    {
      temp1.real=(Y[j]).real;
      temp1.imag=(Y[j]).imag;
      (Y[j]).real=(Y[i]).real;
      (Y[j]).imag=(Y[i]).imag;
      (Y[i]).real=temp1.real;
      (Y[i]).imag=temp1.imag;
    }
  }
  return;
}





void hamming_window(float* window, int nn) {
    for (int n = 0; n < nn; n++) {
        window[n] = 0.54 - 0.46 * cos(2.0 * PI * n / (nn - 1));
    }
}

void find_frequency(float gx_samples[SAMPLES]) {

    COMPLEX samples[N];
    COMPLEX twiddle[N];

    float window[SAMPLES];

    // Apply a window function to the data
    hamming_window(window, SAMPLES);



    for (int i = 0; i < SAMPLES; i++) {
        gx_samples[i] *= window[i];
    }
    //

    for (int n=0 ; n<N ; n++)         //set up DFT twiddle factors
    {
        twiddle[n].real = cos(PI*n/N);
        twiddle[n].imag = -sin(PI*n/N);
    }
    // Step 3: Perform FFTs
    for (int i = 0; i < SAMPLES; i++) {
        samples[i].real = gx_samples[i];
        samples[i].imag = 0;
    }

    fft(samples, N, twiddle);

    // Step 4: Calculate magnitudes
    float magnitudes[SAMPLES];
    for (int i = 0; i < SAMPLES; i++) {
        magnitudes[i] = sqrt(samples[i].real * samples[i].real + samples[i].imag * samples[i].imag);
    }

    // Step 5: Find index of max magnitude
    int max_index = 0;
    for (int i = 1; i < SAMPLES; i++) {
        if (magnitudes[i] > magnitudes[max_index]) {
            max_index = i;
        }
    }

    // Step 6: Convert index to frequency
    float frequency = (float)max_index * SAMPLING_FREQUENCY / SAMPLES;

    printf("Dominant frequency: %f Hz\n", frequency);
}


int main()
{
    // Initialize the SPI object with specific pins.
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);

    // Buffers for sending and receiving data over SPI.
    uint8_t write_buf[32], read_buf[32];

    // Configure SPI format and frequency.
    spi.format(8, 3);
    spi.frequency(1'000'000);

    // Configure CTRL_REG1 register.
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);

    // Configure CTRL_REG4 register.
    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, spi_cb);
    flags.wait_all(SPI_FLAG);


    


    while(1){

        uint16_t raw_gx, raw_gy, raw_gz;
        float gx, gy, gz;

        // Prepare to read the gyroscope values starting from OUT_X_L
        write_buf[0] = OUT_X_L | 0x80 | 0x40;

        // Perform the SPI transfer to read 6 bytes of data (for x, y, and z axes)
        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);

        // Convert the received data into 16-bit integers for each axis
        raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t) read_buf[1]);
        raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t) read_buf[3]);
        raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t) read_buf[5]);

        // Print the raw values for debugging 
        //printf("RAW -> \t\tgx: %d \t gy: %d \t gz: %d \t\n", raw_gx, raw_gy, raw_gz);

        //    printf(">x_axis: %d|g \n", raw_gx);
         //   printf(">y_axis: %d|g \n", raw_gy);
          //  printf(">z_axis: %d|g \n", raw_gz);

        // Convert raw data to actual values using a scaling factor
        gx = ((float) raw_gx) * SCALING_FACTOR;
        gy = ((float) raw_gy) * SCALING_FACTOR;
        gz = ((float) raw_gz) * SCALING_FACTOR;

        // Print the actual values
        //printf("Actual -> \t\tgx: %4.5f \t gy: %4.5f \t gz: %4.5f \t\n", gx, gy, gz);
        filtered_gx = ALPHA * gx + (1.0f - ALPHA) * previous_gx;
        
        gx_samples[sample_index++] = filtered_gx;

        if (sample_index >= SAMPLES) {
            sample_index = 0;

            // Find the dominant frequency
            find_frequency(gx_samples);
            printf("Sending data to the function\n");
        }

        
    }

        
    thread_sleep_for(1000);

    
}