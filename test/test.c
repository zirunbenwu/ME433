#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "heartbeat_v2.h"

// Force sensor GPIO pins
#define GPIO_PIN_SCK 0  // output
#define GPIO_PIN_DT 1   // input

// Sampling period [ms]
#define SAMPLING_PERIOD_MS 1.


uint32_t readForceData(){
    uint32_t dataReadingRAW=0;
    static int sckUsDelay=1;
    int bit;
    
    sleep_us(sckUsDelay);
    for(int i=1; i<=24; i++){
        gpio_put(GPIO_PIN_SCK,1);
        sleep_us(sckUsDelay);
        if(gpio_get(GPIO_PIN_DT)==0){
            bit=0;
        } else if(gpio_get(GPIO_PIN_DT)>0){
            bit=1;
        }
        
        dataReadingRAW|=bit;
        //printf("Raw %.24b, bit %i\n",dataReadingRAW,bit);
        dataReadingRAW<<=1;
        gpio_put(GPIO_PIN_SCK,0);
        sleep_us(sckUsDelay);
    }
    gpio_put(GPIO_PIN_SCK,1);
    sleep_us(sckUsDelay);
    gpio_put(GPIO_PIN_SCK,0);
    sleep_us(sckUsDelay);
    

    return (int32_t) dataReadingRAW;

}

int main()
{
    stdio_init_all();

    gpio_init(GPIO_PIN_SCK); 
    gpio_set_dir(GPIO_PIN_SCK,GPIO_OUT);
    gpio_put(GPIO_PIN_SCK,0); //init SCK low to NOT read data
    
    gpio_init(GPIO_PIN_DT);
    gpio_set_dir(GPIO_PIN_DT, GPIO_IN);
    

    int numDataPts;
    float secondsOfData;
    uint32_t dataReadingRAW;


    sleep_ms(500); // sleep delay to allow time for initial printf to print in terminal

    while (true) {
        printf("\n\n----------------------------------------------\n\n");
        printf("How many seconds of data are desired?\n");
        scanf("%f", &secondsOfData);
        numDataPts=(1000.*secondsOfData)/SAMPLING_PERIOD_MS;
        //printf("%i",numDataPts);
        int timeArr[numDataPts];
        uint dataRAWarr[numDataPts];
        uint dataIIRarr[numDataPts];
        //dataIIRarr[numDataPts-1]=3;
        if(numDataPts<=0||numDataPts>99999999){
            printf("\n\n<!> NUMBER OF DATA POINTS OUT OF RANGE <!>\n\n");
            for(int e=0;e<10;e++){
                heartbeat();
            }
            goto error_exit;
        }

        printf("Collecting %i data points\n", numDataPts);
        
        heartbeat();

        //printf("__Time [ms]__ , __Data (RAW)__ , __Data (IIR)__\n");


        int dataSum = 0; // for avg calc
        uint calcAvg=2530000;
        uint dataReadingIIR=calcAvg; //init of IIR, user input from avg calc code
        
        if(gpio_get(GPIO_PIN_DT)==0){

            for(int i=1; i<=numDataPts; i++){
                //===TIME===
                uint time=(uint) to_ms_since_boot(get_absolute_time());
                //printf("\ntime: %i\n",time);
                timeArr[i]=time;

                //===RAW DATA===
                dataReadingRAW=readForceData();
                if(dataReadingRAW>5000000||dataReadingRAW<2000000){ //electrical noise/saturation cut-off
                    dataReadingRAW=dataRAWarr[i-1];
                }
                //printf("\nDataReceivedRAW: 0b%.24b, 0x%.6X\n",dataReadingRAW,dataReadingRAW);
                dataRAWarr[i]=dataReadingRAW;
                
                //=== IIR DATA ===
                //dataSum += dataReadingRAW; // for avg calc
                
                float signalWeight=0.1;
                dataIIRarr[i]=((1.-signalWeight)*dataReadingIIR) + (signalWeight*dataReadingRAW);
                dataReadingIIR=dataIIRarr[i];

                /*
                //=== PRINT & PLOT ===
                printf("ForceSensorData:%.8u,%.8u,%.8u", time, dataReadingRAW, dataReadingIIR);
                // Graph data on CLI
                char graphBar[]=" |                                                                                                    ";      
                uint16_t markPosition =100.*(dataReadingRAW/(2.*3000000));
                graphBar[markPosition+1]='X';        
                printf("%s\n", graphBar);
                */
                
                sleep_ms(SAMPLING_PERIOD_MS);
            }

        printf("\n\n[Data Collection Finished]\n\n\n");
        }
             
        
        //===PRINT DATA==
        for(int i=1; i<=numDataPts; i++){
            //sleep_ms(10);
            //heartbeat();
            printf("ForceSensorData:%.8i,%.10u,%.10u\n", timeArr[i], dataRAWarr[i], dataIIRarr[i]);
        }    

        //printf("\nData Average: %i\n\n", (dataSum/numDataPts)); //avg calc
        

        error_exit: // jump-to location post error
        printf("\n[END]\n");
        
        //sleep_ms(1000);
    }
}
