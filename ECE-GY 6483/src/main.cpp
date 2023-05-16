/*
 *  ECE-GY 6483 Group 39 Project
 */

#include <mbed.h>
#include <math.h>
#include <stdlib.h>
#include "drivers/LCD_DISCO_F429ZI.h"

#define OUT_X_L 0x28
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1
#define CTRL_REG3 0x22              
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0
#define SCALING_FACTOR (17.5f*0.017453292519943295769236907684886f / 1000.0f)
#define SPI_FLAG 1
#define DATA_READY_FLAG 2
#define DATA_SAMPLE_LEN 625
#define DATA_SAMPLE_DELAY 8
#define WINDOW_SIZE 35
#define RETRY_LIMIT 5
#define N 1024

typedef struct{
  double real;
  double imag;
}complex;

EventFlags flags;
SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel); // mosi, miso, sclk, cs
InterruptIn int2(PA_2,PullDown);
DigitalOut led1(LED1);
DigitalOut led2(LED2);
uint8_t write_buf[32], read_buf[32];
InterruptIn btn(PA_0, PullDown);
Timer t;
LCD_DISCO_F429ZI lcd;

float tolerance = 0.55;

float lock_recording_x[DATA_SAMPLE_LEN];
float lock_recording_y[DATA_SAMPLE_LEN];
float lock_recording_z[DATA_SAMPLE_LEN];
float unlock_recording_x[DATA_SAMPLE_LEN];
float unlock_recording_y[DATA_SAMPLE_LEN];
float unlock_recording_z[DATA_SAMPLE_LEN];

volatile int state_id = 0;        // lcd state control
volatile int program_state = 0;   // main function state control
volatile int is_first_smooth = 0;
volatile int fail_count = 0;
volatile int data_count_lock = 0;
volatile int data_count_unlock = 0;

complex fft[N], *W;
double PI = 4.0*atan(1);

/* ttf help function 1 */
void fft_change(){
  complex temp;
  unsigned short i = 0,j = 0,k = 0;
  double t;
  for(i=0;i<DATA_SAMPLE_LEN;i++){
    k = i;
    j = 0;
    t = (log(DATA_SAMPLE_LEN)/log(2));
    while((t--)>0){
      j = j << 1;
      j |= (k & 1);
      k = k >> 1;
    }
    if(j > i){
      temp = fft[i];
      fft[i] = fft[j];
      fft[j] = temp;
    }
  }
}

/* ttf help function 2 */
void fft_transform(){
  W = (complex *)malloc(sizeof(complex) * DATA_SAMPLE_LEN);
  for(int i = 0; i < DATA_SAMPLE_LEN; i++){
    W[i].real = cos(2*PI/DATA_SAMPLE_LEN*i);
    W[i].imag = -1*sin(2*PI/DATA_SAMPLE_LEN*i);
  }
}

/* ttf help function for calculation */
void add(complex src, complex summand, complex *target){
  target->real = src.real+summand.real;
  target->imag = src.imag+summand.imag;
}

void sub(complex src, complex minuend, complex *target){
  target->real = src.real-minuend.real;
  target->imag = src.imag-minuend.imag;
}

void mul(complex src, complex multiplcand, complex *target){
  target->real = src.real*multiplcand.real - src.imag*multiplcand.imag;
  target->imag = src.real*multiplcand.imag - src.imag*multiplcand.real;
}

void fft_main(){
  int m = 0;
  complex q,y,z;
  fft_change();
  for(int i = 0; i<log(DATA_SAMPLE_LEN)/log(2);i++){
    m = 1<<i;
    for(int j = 0; j<DATA_SAMPLE_LEN;j+=2*m){
      for(int k = 0;k<m;k++){
        mul(fft[k+j+m],W[DATA_SAMPLE_LEN*k/2],&q);
        add(fft[j+k],q,&y);
        add(fft[j+k],q,&z);
        fft[j+k] = y;
        fft[j+k+m] = z;
      }
    }
  }
}

/* calculate the difference between the lock and unlock recording */
int test_data(float *lockList, float *unlockList, int size){
  float temp = 0;
  for(int i = 0; i < size; i++){
    temp += (lockList[i] - unlockList[i]) * (lockList[i] - unlockList[i]); 
  }
  temp = temp / (size * 1.0);
  if(temp > tolerance){
    return 0;
  }else{
    return 1;
  }
}

/* help function for data filter */
float window_mean_sum(int start, int end, float *target){
  float result = 0.0;
  for(int i = start; i < end; i++){
    result += target[i];
  }
  return result / (end - start);
}

/* help function for edit list */
void edit_data(float *prevList, float *newList, int size){
  for(int i = 0;i<size;i++){
    prevList[i] = newList[i];
  }
}

/* data filter, moving average */
void smooth_data(float *target, int size, int window){
  float temp_data[DATA_SAMPLE_LEN];
  for(int i = 0;i<size;i++){
    int start = ((i - window/2) < 0) ? 0 : (i - window/2);
    int end = ((i + window/2) > size) ? size : (i + window/2);
    float mean_sum = window_mean_sum(start, end, target);
    temp_data[i] = mean_sum;
  }
  edit_data(target, temp_data, size);
}

/* find mean value of the list */
float mean(float data[]) {
    int length = sizeof(data) / sizeof(data[0]);
    float sum = 0.0f;
    for (int i = 0; i < length; i++) {
        sum += data[i];
    }
    return sum / length;
}

/* find standard deivation of the list */
float std(float data[], float mean) {
    int length = sizeof(data) / sizeof(data[0]);
    float sum = 0.0f;
    for (int i = 0; i < length; i++) {
        float diff = data[i] - mean;
        sum += diff * diff;
    }
    float variance = sum / length;
    return sqrt(variance);
}

float coV(float data1[], float data2[], float mean1, float mean2) {
    int length = sizeof(data1) / sizeof(data1[0]);
    float sum = 0.0f;
    for (int i = 0; i < length; i++) {
        float diffA = data1[i] - mean1;
        float diffB = data2[i] - mean2;
        sum += diffA * diffB;
    }
    return sum / length;
}

/* algorithm for calculate the difference */
float pearsonCo(float f1[], float f2[]) {
    int length = sizeof(f1) / sizeof(f1[0]);
    float mf1 = mean(f1);
    float mf2 = mean(f2);

    float stdf1 = std(f1, mf1);
    float stdf2 = std(f2, mf2);

    float cov = coV(f1, f2, mf1, mf2);

    return cov / (stdf1 * stdf2);
}

/* help function for check difference */
int passCheck(float* pw, float* unlock) {
    float coSum = pearsonCo(pw, unlock);
    return fabs(coSum);
}


//The spi.transfer function requires that the callback
//provided to it takes an int parameter
void spi_cb(int event){
  flags.set(SPI_FLAG);
}

void data_cb(){
  flags.set(DATA_READY_FLAG);
}


/* flow control function for button press action */
void toggle(){
  if((t.elapsed_time()).count() >= 300){
    if(state_id == 0){
      lcd.DisplayStringAtLine(0,(uint8_t *)"READY FOR LOCK");
    }else if(state_id == 1){
      lcd.ClearStringLine(LINE(0));
      lcd.DisplayStringAtLine(0,(uint8_t *)"LOCKING START");
      program_state = 1;
    }else if(state_id == 2){
      lcd.ClearStringLine(LINE(0));
      lcd.DisplayStringAtLine(0,(uint8_t *)"READY FOR UNLOCK");
    }else if(state_id == 3){
      lcd.ClearStringLine(LINE(0));
      lcd.DisplayStringAtLine(0,(uint8_t *)"UNLOCKING START");
      program_state = 2;
    }else if(state_id == 4){
      lcd.ClearStringLine(LINE(0));
      lcd.DisplayStringAtLine(0,(uint8_t *)"DEALING DATA");
      program_state = 3;
    }else if(state_id == 5){
      lcd.ClearStringLine(LINE(0));
      lcd.DisplayStringAtLine(0,(uint8_t *)"RETRY");   
      data_count_unlock = 0;
      state_id = 1;   
    }
    state_id++;
    t.reset();
  }
}
 
int main() {
  // set leds off
  led1 = 0;
  led2 = 0;
  // Setup the spi for 8 bit data, high steady state clock,
  // second edge capture, with a 1MHz clock rate
  spi.format(8,3);
  spi.frequency(1'000'000);
  //configure the interrupt to call our function
  //when the pin becomes high
  int2.rise(&data_cb);
  btn.rise(&toggle);

  // init communication with the board, Set up general behaviors
  write_buf[0]=CTRL_REG1;
  write_buf[1]=CTRL_REG1_CONFIG;
  spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
  flags.wait_all(SPI_FLAG);

  write_buf[0]=CTRL_REG4;
  write_buf[1]=CTRL_REG4_CONFIG;
  spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
  flags.wait_all(SPI_FLAG);

  write_buf[0]=CTRL_REG3;
  write_buf[1]=CTRL_REG3_CONFIG;
  spi.transfer(write_buf,2,read_buf,2,spi_cb,SPI_EVENT_COMPLETE );
  flags.wait_all(SPI_FLAG);

  write_buf[1] = 0xFF;	//sanity check

  if(!(flags.get()&DATA_READY_FLAG)&&(int2.read()==1)){
    flags.set(DATA_READY_FLAG);
  }

  t.start();

  while(1){
    if (program_state == 0){
      //do nothing
    }else if(program_state == 1){
      //lock recording
      int16_t raw_gx, raw_gy, raw_gz;
      float gx, gy, gz;

      //wait until new sample is ready
      flags.wait_all(DATA_READY_FLAG);
      //prepare the write buffer to trigger a sequential read
      write_buf[0]=OUT_X_L|0xC0;

      //start sequential sample reading
      spi.transfer(write_buf,7,read_buf,8,spi_cb,SPI_EVENT_COMPLETE);
      flags.wait_all(SPI_FLAG);

      //read_buf after transfer: garbage byte, gx_low,gx_high,gy_low,gy_high,gz_low,gz_high
      //Put the high and low bytes in the correct order lowB,Highb -> HighB,LowB
      raw_gx=( ( (uint16_t)read_buf[2] ) <<8 ) | ( (uint16_t)read_buf[1] );
      raw_gy=( ( (uint16_t)read_buf[4] ) <<8 ) | ( (uint16_t)read_buf[3] );
      raw_gz=( ( (uint16_t)read_buf[6] ) <<8 ) | ( (uint16_t)read_buf[5] );

      //printf("RAW|\tgx: %d \t gy: %d \t gz: %d\n",raw_gx,raw_gy,raw_gz);
      gx=((float)raw_gx)* SCALING_FACTOR;
      gy=((float)raw_gy)* SCALING_FACTOR;
      gz=((float)raw_gz)* SCALING_FACTOR;

      lock_recording_x[data_count_lock] = gx;
      lock_recording_y[data_count_lock] = gy;
      lock_recording_z[data_count_lock] = gz;
      data_count_lock++;
      // manual control the data collection rate
      thread_sleep_for(DATA_SAMPLE_DELAY);
      if(data_count_lock > DATA_SAMPLE_LEN-1){
          // end check for lock recording
          lcd.ClearStringLine(LINE(0));
          lcd.DisplayStringAtLine(0,(uint8_t *)"LOCKING END");
          program_state = 0;
          t.reset();
          // blink the led light
          led1 = 1;
          thread_sleep_for(250);
          led1 = 0;
          thread_sleep_for(250);
          led1 = 1;
          thread_sleep_for(250);
          led1 = 0;
      }
    }else if(program_state == 2){
      // unlock recording
      int16_t raw_gx, raw_gy, raw_gz;
      float gx, gy, gz;

      //wait until new sample is ready
      flags.wait_all(DATA_READY_FLAG);
      //prepare the write buffer to trigger a sequential read
      write_buf[0]=OUT_X_L|0xC0;

      //start sequential sample reading
      spi.transfer(write_buf,7,read_buf,8,spi_cb,SPI_EVENT_COMPLETE );
      flags.wait_all(SPI_FLAG);

      //read_buf after transfer: garbage byte, gx_low,gx_high,gy_low,gy_high,gz_low,gz_high
      //Put the high and low bytes in the correct order lowB,Highb -> HighB,LowB
      raw_gx=( ( (uint16_t)read_buf[2] ) <<8 ) | ( (uint16_t)read_buf[1] );
      raw_gy=( ( (uint16_t)read_buf[4] ) <<8 ) | ( (uint16_t)read_buf[3] );
      raw_gz=( ( (uint16_t)read_buf[6] ) <<8 ) | ( (uint16_t)read_buf[5] );

      //printf("RAW|\tgx: %d \t gy: %d \t gz: %d\n",raw_gx,raw_gy,raw_gz);
      gx=((float)raw_gx)* SCALING_FACTOR;
      gy=((float)raw_gy)* SCALING_FACTOR;
      gz=((float)raw_gz)* SCALING_FACTOR;

      unlock_recording_x[data_count_unlock] = gx;
      unlock_recording_y[data_count_unlock] = gy;
      unlock_recording_z[data_count_unlock] = gz;
      data_count_unlock++;
      // DATA_SAMPLE_DELAY*DATA_SAMPLE_LEN/1000 = total time
      thread_sleep_for(DATA_SAMPLE_DELAY);
      if(data_count_unlock > DATA_SAMPLE_LEN-1){
          // end check for unlock recording
          lcd.ClearStringLine(LINE(0));
          lcd.DisplayStringAtLine(0,(uint8_t *)"UNLOCKING END");
          program_state = 0;
          t.reset();
      }
    }else if(program_state == 3){
      // data processing and comparation
      if(is_first_smooth == 0){
        // deal with lock recording for only once
        smooth_data(lock_recording_x, DATA_SAMPLE_LEN, WINDOW_SIZE);
        smooth_data(lock_recording_y, DATA_SAMPLE_LEN, WINDOW_SIZE);
        smooth_data(lock_recording_z, DATA_SAMPLE_LEN, WINDOW_SIZE);
        is_first_smooth++;
      }
      // deal with unlock recording
      smooth_data(unlock_recording_x, DATA_SAMPLE_LEN, WINDOW_SIZE);
      smooth_data(unlock_recording_y, DATA_SAMPLE_LEN, WINDOW_SIZE);
      smooth_data(unlock_recording_z, DATA_SAMPLE_LEN, WINDOW_SIZE);
      // validation check method 1
      /*
        float vlaid = 0.0f;
        vlaid += passCheck(lock_recording_x, unlock_recording_x);
        vlaid += passCheck(lock_recording_y, unlock_recording_y);
        vlaid += passCheck(lock_recording_z, unlock_recording_z);
        if(valid/3 >= tolerance){
          //successful
        }
      */

      // validation check method 2
      int valid = 0;
      valid += test_data(lock_recording_x, unlock_recording_x, DATA_SAMPLE_LEN);
      valid += test_data(lock_recording_y, unlock_recording_y, DATA_SAMPLE_LEN);
      valid += test_data(lock_recording_z, unlock_recording_z, DATA_SAMPLE_LEN);
      if(valid == 3){
        lcd.ClearStringLine(LINE(0));
        lcd.DisplayStringAtLine(0,(uint8_t *)"SUCCESSFUL");
        led2 = 0;
        led1 = 1;
        state_id = 10;
        break;
      }else{
        lcd.ClearStringLine(LINE(0));
        lcd.DisplayStringAtLine(0,(uint8_t *)"FAIL");
        program_state = 0;
        fail_count++;
        led2 = 1;
        thread_sleep_for(200);
        led2 = 0;
        thread_sleep_for(200);
        led2 = 1;
        thread_sleep_for(200);
        led2 = 0;
        t.reset();
        if(fail_count > RETRY_LIMIT){
          // chance for retry
          state_id = 10;
          led1 = 0;
          led2 = 1;
          break;
        }
      }
    }else{
      state_id = 10;
      break;
    }
  }
}