#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_mac.h"
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/gpio.h"


#define GPS_STATUS_LED GPIO_NUM_6
#define BUF_SIZE (1024*8)
#define EXAMPLE_MAX_CHAR_SIZE    2048
#define MAX_PATH 32

char GPS_data[2048];
char data[EXAMPLE_MAX_CHAR_SIZE];
char GPS_output[EXAMPLE_MAX_CHAR_SIZE];
char NMEA_data [6][15][15];
char baud_rates[7][6] = {"4800", "9600", "14400", "19200", "38400", "57600", "115200"};

int baud_rate = 6;

uart_config_t uart_config = 
{
    .baud_rate = 9600,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_APB,
};

static const char *TAG = "example";

#define MOUNT_POINT "/sdcard"

#define PIN_NUM_MISO  9
#define PIN_NUM_MOSI  10
#define PIN_NUM_CLK   8
#define PIN_NUM_CS    4


void generate_gpx_file(char* filename) {
    
    FILE *f_gpx = fopen(filename, "a+");
    if (f_gpx == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
    }
    else
    {
        fprintf(f_gpx, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        fprintf(f_gpx, "<gpx creator=\"ESP32_GPS_Tracker\">\n");
        fprintf(f_gpx, "  <metadata>\n");
        fprintf(f_gpx, "    <name>GPS Data</name>\n");
        fprintf(f_gpx, "    <desc>A GPX file generated from EPS32 GPS tracker</desc>\n");
        fprintf(f_gpx, "  </metadata>\n");
        fprintf(f_gpx, "<trk>\n");
        fprintf(f_gpx, "     <name>Test Track</name>\n");
        fprintf(f_gpx, "     <type>Driving</type>\n");
        fprintf(f_gpx, "<trkseg>\n");
        fprintf(f_gpx, "</trkseg>\n");
        fprintf(f_gpx, "</trk>\n");
        fprintf(f_gpx, "</gpx>\n");

        

        fclose(f_gpx);
    }
}

void write_waypoint(FILE *file, float lat, float lon, float ele, const char* time) {
    fprintf(file, "  <wpt lat=\"%.6f\" lon=\"%.6f\">\n", lat, lon);
    fprintf(file, "    <ele>%f</ele>\n", ele);
    fprintf(file, "    <time>%s</time>\n", time);
    fprintf(file, "  </wpt>\n");
}

void begin_track(FILE *file) {
    fprintf(file, "<trk>\n");
    fprintf(file, "     <name>Test Track</name>\n");
    fprintf(file, "     <type>Driving</type>\n");
    fprintf(file, "<trkseg>\n");
    fprintf(file, "</trkseg>\n");
    fprintf(file, "</trk>\n");
}

void write_track_point(FILE *file, float lat, float lon, float ele, const char* time) {
    fprintf(file, "<trkpt lat=\"%.6f\" lon=\"%.6f\">\n", lat, lon);
    fprintf(file, "<ele>%f</ele>\n", ele);
    fprintf(file, "<time>%s</time></trkpt>\n", time);
}

int nmea0183_checksum(char *nmea_data)
{
    int crc = 0;
    int i;
    
    for (i = 1; i < strlen(nmea_data)-5; i ++) {
        crc ^= nmea_data[i];
    }

    return crc;
}

void intToHexString(int num, char *hexStr) 
{
    sprintf(hexStr, "%x", num);  // Convert integer to hex string
}

float ConvertLatToDecimalDegrees(const char* LatStr, const char* LatDirection) {
    int degrees, minutes;
    float decimalMinutes;
    float decimalLatitude;

    sscanf(LatStr, "%2d%2d.%4f", &degrees, &minutes, &decimalMinutes);

    if (LatDirection[0] == 'S')
    {
        decimalLatitude = -1 * (degrees + (minutes + decimalMinutes / 10000) / 60.0);
    }
    else
    {
        decimalLatitude = degrees + (minutes + decimalMinutes / 10000) / 60.0;
    }

    return decimalLatitude;
}

float ConvertLongToDecimalDegrees(const char* LongStr, const char* LongDirection) {
    int degrees, minutes;
    float decimalMinutes;
    float decimalLongitude;

    sscanf(LongStr, "%3d%2d.%4f", &degrees, &minutes, &decimalMinutes);


    if (LongDirection[0] == 'W')
    {
        decimalLongitude = -1 * (degrees + (minutes + decimalMinutes / 10000) / 60.0);
    }
    else
    {
        decimalLongitude = degrees + (minutes + decimalMinutes / 10000) / 60.0;
    }

    return decimalLongitude;
}

void ConvertDateandTimeFormat(const char* DateStr, const char* TimeStr, char* result) {
    int year, month, day, hour, minute; 
    float second;

    sscanf(DateStr, "%2d%2d%2d", &day, &month, &year);
    sscanf(TimeStr, "%2d%2d%6f", &hour, &minute, &second);

    sprintf(result, "20%02d-%02d-%02dT%02d:%02d:%06.3fZ", year, month, day, hour, minute, second);
}

static void uart_task()
{
    while (1)
    {   
        int len = uart_read_bytes(UART_NUM_1, GPS_data, BUF_SIZE, 10);
        
        
        if (len>0)
        { 
            GPS_data[len] = '\0';
            
            int j = 0;
            int k = 0;
            int l = 0;

            char Sentance_Data[6];

            Sentance_Data[5] = '\0';

            for (size_t i = 0; i < len; i++)
            {

                if(GPS_data[i] == '$' && GPS_data[i+1] == 'G')
                {
                    strncpy(Sentance_Data, GPS_data+i+1, 5);
                    

                    l = 0;
    
                    if (strcmp("GPGLL",Sentance_Data) == 0)
                    {
                        j = 0;
                        i+=5;
                        l = 6;
                        strncpy(NMEA_data[j][0], Sentance_Data, 6);  
                    }
                    else if (strcmp("GPRMC",Sentance_Data) == 0)
                    {
                        j = 1;
                        i+=5;
                        l = 6;
                        strncpy(NMEA_data[j][0], Sentance_Data, 6);
                    }
                    else if (strcmp("GPVTG",Sentance_Data) == 0)
                    {
                        j = 2;
                        i+=5;
                        l = 6;
                        strncpy(NMEA_data[j][0], Sentance_Data, 6);
                    }
                    else if (strcmp("GPGGA",Sentance_Data) == 0)
                    {
                        j = 3;
                        i+=5;
                        l = 6;
                        strncpy(NMEA_data[j][0], Sentance_Data, 6);
                    }
                    else if (strcmp("GPGSA",Sentance_Data) == 0)
                    {
                        j = 4;
                        i+=5;
                        l = 6;
                        strncpy(NMEA_data[j][0], Sentance_Data, 6);
                    }
                    else if (strcmp("GPGSV",Sentance_Data) == 0)
                    {
                        j = 5;
                        i+=5;
                        l = 6;
                        strncpy(NMEA_data[j][0], Sentance_Data, 6);
                    }
                }
                else if(GPS_data[i] == '$' && GPS_data[i+1] != 'G') 
                {
                    i++;
                    while(GPS_data[i] != '$')
                    {
                        i++;
                    }
                }
                else if (GPS_data[i] == ',')
                {
                    NMEA_data[j][k][l] = '\0';
                    k+=1;
                    l = 0;
                }
                else if (GPS_data[i] == '\r')
                {
                    ;
                }
                else if (GPS_data[i] == '\n')
                {
                    NMEA_data[j][k][l] = '\0';
                    k = 0;
                }
                else if (GPS_data[i] != ',')
                {
                    NMEA_data[j][k][l] = GPS_data[i];
                    l+=1;
                }

                
            }
            

            // If data is valid
            if (NMEA_data[1][2][0] == 'A')
            {
                // Valid GPS status LED is turned on
                gpio_set_level(GPS_STATUS_LED, 1);

                float latitude;
                float longitude;
                char GPX_Time[30];
                float MSL = atof(NMEA_data[3][9]);
                float geoid_sep = atof(NMEA_data[3][11]);
                float elevation = MSL;

                latitude = 0;
                longitude = 0;


                // Latitude, Longitude, and Time are all converted to a different format. See functions for details
                latitude = ConvertLatToDecimalDegrees(NMEA_data[1][3], NMEA_data[1][4]);
                longitude = ConvertLongToDecimalDegrees(NMEA_data[1][5], NMEA_data[1][6]);
                ConvertDateandTimeFormat(NMEA_data[1][9], NMEA_data[1][1], GPX_Time);

                
                

                

                
                char gpx_file_path[26]; 
                strcpy(gpx_file_path, "\0");

                strcat(gpx_file_path, "/sdcard/DATA_");
                strcat(gpx_file_path, NMEA_data[1][9]);
                strcat(gpx_file_path, ".gpx");

                // Checks to see if GPX file is accessible. If not, it is generated
                if (access(gpx_file_path, F_OK) == 0)
                {
                    
                }
                else
                {
                    printf(gpx_file_path);
                    generate_gpx_file(gpx_file_path);
                }

                FILE *f_gpx = fopen(gpx_file_path, "r+");
                if (f_gpx == NULL) 
                {
                    ESP_LOGE(TAG, "Failed to open file for writing");
                }
                else //removes the footer. Need to rework this later to just move the cursor
                {
                    fseek(f_gpx, -24, SEEK_END);
                    write_track_point(f_gpx, latitude, longitude, elevation, GPX_Time);

                    //printf("lat=%f, long=%f, ele=%f, time=%s\n", latitude, longitude, elevation, GPX_Time);

                    fprintf(f_gpx, "</trkseg>\n");
                    fprintf(f_gpx, "</trk>\n");
                    fprintf(f_gpx, "</gpx>\n");
                }

                
                
                



                strcpy(gpx_file_path, "\0");
                strcpy(GPS_output, "\0");
                fclose(f_gpx);
            
            }
            // Data is not valid
            else if (NMEA_data[1][2][0] == 'V')
            {
                gpio_set_level(GPS_STATUS_LED, 0);

                float latitude;
                float longitude;
                char GPX_Time[30];

                // Latitude, Longitude, and Time are all converted to a different format. See functions for details
                latitude = ConvertLatToDecimalDegrees(NMEA_data[1][3], NMEA_data[1][4]);
                longitude = ConvertLongToDecimalDegrees(NMEA_data[1][5], NMEA_data[1][6]);
                ConvertDateandTimeFormat(NMEA_data[1][9], NMEA_data[1][1], GPX_Time);

                printf("lat=%f, long=%f\n", latitude, longitude);

                
            }
            
            
            
            
            uart_flush(UART_NUM_1);   
        }     
        
    }

    

}

void SD_Setup(void)
{
    esp_err_t ret;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
    #ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
    #else
        .format_if_mount_failed = false,
    #endif
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };


    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = 4;
    slot_config.host_id = host.slot;
    

    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }
    ESP_LOGI(TAG, "Filesystem mounted");

    sdmmc_card_print_info(stdout, card);

}

void GPIO_Setup(void)
{
    // Sets up status LED's gpio pin
    gpio_reset_pin(GPS_STATUS_LED);
    gpio_set_direction(GPS_STATUS_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(GPS_STATUS_LED, 0);
}

void UART_Setup()
{
    uart_driver_install(UART_NUM_1, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, 2, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    
    char GPS_command_baud_rate[21];

    strcat(GPS_command_baud_rate, "$PMTK251,");
    strcat(GPS_command_baud_rate, baud_rates[baud_rate]);
    strcat(GPS_command_baud_rate, "*00\r\n");


    char nmea_sentence_checksum;

    nmea_sentence_checksum = nmea0183_checksum(GPS_command_baud_rate);

    //printf(nmea_sentence_checksum);

    char hexString[2];

    intToHexString(nmea_sentence_checksum, hexString);

    GPS_command_baud_rate[strlen(GPS_command_baud_rate)-4] = hexString[0];
    GPS_command_baud_rate[strlen(GPS_command_baud_rate)-3] = hexString[1];

    printf("%s\n", GPS_command_baud_rate);

    
    for (size_t i = 0; i < sizeof(baud_rates); i++)
    {
        uart_set_baudrate(UART_NUM_1, atoi(baud_rates[i]));

        uart_write_bytes(UART_NUM_1, (const char*)GPS_command_baud_rate, 21);

        uart_wait_tx_done(UART_NUM_1, 1000);
    }

    uart_set_baudrate(UART_NUM_1, atoi(baud_rates[baud_rate]));



    

    char* GPS_rate_command_fast = "$PMTK220,100*2F\r\n";

    char* GPS_rate_command_slow = "$PMTK220,1000*1F\r\n";

    char* GPS_erase_flash = "$PMTK184,1*22\r\n";

    

    char* GPS_sys_msg = "$PMTK011,MTKGPS*08\r\n";

    char GPS_NMEA_sentence_command[] = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*00\r\n"; 

    
    nmea_sentence_checksum = nmea0183_checksum(GPS_NMEA_sentence_command);

    

    intToHexString(nmea_sentence_checksum, hexString);

    GPS_NMEA_sentence_command[47] = hexString[0];
    GPS_NMEA_sentence_command[48] = hexString[1];

    uart_flush(UART_NUM_1);
    
    uart_write_bytes(UART_NUM_1, (const char*)GPS_NMEA_sentence_command, 52);
    uart_wait_tx_done(UART_NUM_1, 1000);


    uart_write_bytes(UART_NUM_1, (const char*)GPS_rate_command_slow, 20);
    uart_wait_tx_done(UART_NUM_1, 1000);


}

void app_main(void)
{
    GPIO_Setup();
    
    SD_Setup();

    UART_Setup();

    
    
    uart_task();
}
