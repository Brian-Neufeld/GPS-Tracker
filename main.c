// https://cdn.sparkfun.com/datasheets/Sensors/GPS/NMEA%20Reference%20Manual1.pdf
// https://cdn-shop.adafruit.com/datasheets/PMTK%20command%20packet-Complete-C39-A01.pdf
// https://cdn.sparkfun.com/assets/parts/1/2/2/8/0/PMTK_Packet_User_Manual.pdf



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "esp_mac.h"
#include "esp_system.h"
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
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
int CD_status = 0;
int CD_status_old = 0;
int counter = 0;
int SDMMC_TIMEOUT_MS = 500;

// Method to determine when to start a new tracking
int max_time_difference = 2; // Maximum time between points in seconds

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
#define PIN_NUM_CD    7


// If SD card fails to format, it will be formated 
esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
    .format_if_mount_failed = true,
#else
    .format_if_mount_failed = false,
#endif
    .max_files = 5,
    .allocation_unit_size = 16 * 1024
};

// Defines the mount point for the SD card
sdmmc_card_t *card;
const char mount_point[] = MOUNT_POINT;

sdmmc_host_t host = SDSPI_HOST_DEFAULT();


// SD cards spi bus is configured
spi_bus_config_t bus_cfg = {
    .mosi_io_num = PIN_NUM_MOSI,
    .miso_io_num = PIN_NUM_MISO,
    .sclk_io_num = PIN_NUM_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4000,
};



void SD_Setup(void)
{
    esp_err_t ret;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = 4;
    slot_config.host_id = host.slot;

    // Attempts to initalize the spi bus
    ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

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

    //sdmmc_card_print_info(stdout, card);

    CD_status_old = 0;

}

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

void begin_new_track(FILE *file, const char* time) {
    fprintf(file, "<trk>\n");
    fprintf(file, "     <name>Track %s</name>\n", time);
    fprintf(file, "     <type>Placeholder</type>\n");
    fprintf(file, "<trkseg>\n");
    fprintf(file, "</trkseg>\n");
    fprintf(file, "</trk>\n");
    fprintf(file, "</gpx>\n");
}

void write_track_point(FILE *file, float lat, float lon, float ele, const char* time, float HDOP) {
    fprintf(file, "   <trkpt lat=\"%.6f\" lon=\"%.6f\"><ele>%f</ele><time>%s</time></trkpt><HDOP>%f</HDOP>\n", lat, lon, ele, time, HDOP);
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

void ConvertDateandTimeFormat(const char* DateStr, const char* TimeStr, char* result_gpx, int *unix_time) {
    int year, month, day, hour, minute; 
    float second;

    sscanf(DateStr, "%2d%2d%2d", &day, &month, &year);
    sscanf(TimeStr, "%2d%2d%6f", &hour, &minute, &second);

    struct tm time;
    
    time.tm_year = year + 100;
    time.tm_mon = month - 1;
    time.tm_mday = day;
    time.tm_hour = hour;
    time.tm_min = minute;
    time.tm_sec = floor(second);
    

    *unix_time = mktime(&time);

    sprintf(result_gpx, "20%02d-%02d-%02dT%02d:%02d:%06.3fZ", year, month, day, hour, minute, second);

    //sprintf(unix_time, "20%02d%02d%02d%02d%02d%06.3f", year, month, day, hour, minute, second); 
}

void GPIO_Setup(void)
{
    // Sets up status LED's gpio pin
    gpio_reset_pin(GPS_STATUS_LED);
    gpio_set_direction(GPS_STATUS_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(GPS_STATUS_LED, 0);

    gpio_set_direction(PIN_NUM_CD, GPIO_MODE_INPUT);

    gpio_set_intr_type(PIN_NUM_CD, GPIO_INTR_NEGEDGE);
}

static const char *TAG2 = "GPIO_INT";

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    ESP_EARLY_LOGI(TAG2, "GPIO %d went LOW", gpio_num);
}




void UART_Setup()
{
    const TickType_t xDelay = 500 / portTICK_PERIOD_MS;
    vTaskDelay(xDelay);


    uart_driver_install(UART_NUM_1, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, 2, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);


    char* GPS_baud_rate_command = "$PMTK251,19200*22\r\n";

    
    uart_set_baudrate(UART_NUM_1, 9600);

    uart_wait_tx_done(UART_NUM_1, 200);

    uart_write_bytes(UART_NUM_1, (const char*)GPS_baud_rate_command, 22);

    uart_wait_tx_done(UART_NUM_1, 100);

    uart_set_baudrate(UART_NUM_1, 19200);

    

    char* GPS_rate_command_fast = "$PMTK220,250*29\r\n";

    char* GPS_rate_command_slow = "$PMTK220,1000*1F\r\n";

    char* GPS_erase_flash = "$PMTK184,1*22\r\n";

    char* GPS_data_port_info = "$PMTK602*36\r\n";

    char* GPS_sys_msg = "$PMTK011,MTKGPS*08\r\n";

    char GPS_NMEA_sentence_command[] = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"; 



    uart_flush(UART_NUM_1);
    uart_flush_input(UART_NUM_1);
    
    uart_write_bytes(UART_NUM_1, (const char*)GPS_NMEA_sentence_command, 53);
    uart_wait_tx_done(UART_NUM_1, 100);

    uart_read_bytes(UART_NUM_1, GPS_data, BUF_SIZE, 5);

    //printf("%s\n", GPS_data);
    if(GPS_data[13] == '2')
    {
        printf("\nOutput Sentence Command Failed\n");
    }
    else if(GPS_data[13] == '3')
    {
        printf("\nOutput Sentence Command Succeeded\n");
    }

    uart_flush(UART_NUM_1);
    uart_flush_input(UART_NUM_1);


    uart_write_bytes(UART_NUM_1, (const char*)GPS_rate_command_fast, 20);
    uart_wait_tx_done(UART_NUM_1, 100);

    uart_read_bytes(UART_NUM_1, GPS_data, BUF_SIZE, 5);

    //printf("%s\n", GPS_data);
    if(GPS_data[13] == '2')
    {
        printf("Output Rate Command Failed\n");
    }
    else if(GPS_data[13] == '3')
    {
        printf("Output Rate Command Succeeded\n");
    }

    uart_flush(UART_NUM_1);
    uart_flush_input(UART_NUM_1);


    int actual_baudrate;

    uart_get_baudrate(UART_NUM_1, &actual_baudrate);

    printf("Output Baudrate: %d\n", actual_baudrate);


}

static void UART_Task()
{
    int previous_point_time = 0;

    while (1)
    {   
        int len = uart_read_bytes(UART_NUM_1, GPS_data, BUF_SIZE, 5);

        if (len>0)
        { 
            GPS_data[len] = '\0';

            //printf("%d\n", len);
            //printf("\n%s", GPS_data);

            
            int j = 0;
            int k = 0;
            int l = 0;

            char Sentence_Data[6];

            Sentence_Data[5] = '\0';

            for (size_t i = 0; i < len; i++)
            {
                if(GPS_data[i] == '$' && GPS_data[i+1] == 'G')
                {
                    strncpy(Sentence_Data, GPS_data+i+1, 5);
                    

                    l = 0;
    
                    if (strcmp("GPGLL",Sentence_Data) == 0)
                    {
                        j = 0;
                        i+=5;
                        l = 6;
                        strncpy(NMEA_data[j][0], Sentence_Data, 6);  
                    }
                    else if (strcmp("GPRMC",Sentence_Data) == 0)
                    {
                        j = 1;
                        i+=5;
                        l = 6;
                        strncpy(NMEA_data[j][0], Sentence_Data, 6);
                    }
                    else if (strcmp("GPVTG",Sentence_Data) == 0)
                    {
                        j = 2;
                        i+=5;
                        l = 6;
                        strncpy(NMEA_data[j][0], Sentence_Data, 6);
                    }
                    else if (strcmp("GPGGA",Sentence_Data) == 0)
                    {
                        j = 3;
                        i+=5;
                        l = 6;
                        strncpy(NMEA_data[j][0], Sentence_Data, 6);
                    }
                    else if (strcmp("GPGSA",Sentence_Data) == 0)
                    {
                        j = 4;
                        i+=5;
                        l = 6;
                        strncpy(NMEA_data[j][0], Sentence_Data, 6);
                    }
                    else if (strcmp("GPGSV",Sentence_Data) == 0)
                    {
                        j = 5;
                        i+=5;
                        l = 6;
                        strncpy(NMEA_data[j][0], Sentence_Data, 6);
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
                //printf("Valid data\n");
                gpio_set_level(GPS_STATUS_LED, 1);

                printf("CD_Status: %d CD_Status_Old: %d\n", CD_status, CD_status_old);

                float latitude;
                float longitude;
                char GPX_Time[30];
                int unix_time;
                float MSL = atof(NMEA_data[3][9]);
                float geoid_sep = atof(NMEA_data[3][11]);
                float elevation = MSL;
                float HDOP = atof(NMEA_data[3][8]);

                latitude = 0;
                longitude = 0;

                //printf(NMEA_data[1][2][0]);
                
                // Conditions to determine whether data is saved
                //printf("HDOP= %f\n", HDOP);

                if (HDOP <= 5)
                {
                    // Latitude, Longitude, and Time are all converted to a different format. See functions for details
                    latitude = ConvertLatToDecimalDegrees(NMEA_data[1][3], NMEA_data[1][4]);
                    longitude = ConvertLongToDecimalDegrees(NMEA_data[1][5], NMEA_data[1][6]);
                    ConvertDateandTimeFormat(NMEA_data[1][9], NMEA_data[1][1], GPX_Time, &unix_time);

                    //printf("Lat = %f, Long = %f, Time = %s\n", latitude, longitude, GPX_Time);
                    
                    char gpx_file_path[26]; 
                    strcpy(gpx_file_path, "\0");

                    strcat(gpx_file_path, "/sdcard/DATA_");
                    strcat(gpx_file_path, NMEA_data[1][9]);
                    //printf("Date= %s\n", NMEA_data[1][9]);
                    strcat(gpx_file_path, ".gpx");

                    CD_status = gpio_get_level(7);

                    //printf("CD_Status: %d CD_Status: %d\n", CD_status, CD_status_old);


                    if (CD_status == 1 && CD_status_old == 1)
                    {
                        printf("card connected\n");

                        

                        FILE *f_gpx = fopen(gpx_file_path, "r+");
                        

                        if (f_gpx == NULL)
                        {
                            generate_gpx_file(gpx_file_path);
                        }
                    
                        if((unix_time - previous_point_time) <= 2)
                        {
                            fseek(f_gpx, -24, SEEK_END);
                            write_track_point(f_gpx, latitude, longitude, elevation, GPX_Time, HDOP);

                            fprintf(f_gpx, "</trkseg>\n");
                            fprintf(f_gpx, "</trk>\n");
                            fprintf(f_gpx, "</gpx>\n");
                        }
                        else
                        {
                            printf("starting new track");
                            fseek(f_gpx, -7, SEEK_END);
                            begin_new_track(f_gpx, GPX_Time);

                            fseek(f_gpx, -24, SEEK_END);
                            write_track_point(f_gpx, latitude, longitude, elevation, GPX_Time, HDOP);

                            fprintf(f_gpx, "</trkseg>\n");
                            fprintf(f_gpx, "</trk>\n");
                            fprintf(f_gpx, "</gpx>\n");
                        }
                        previous_point_time = unix_time;

                        fclose(f_gpx);
                        f_gpx = NULL;
                    }
                    else if (CD_status == 1 && CD_status_old == 0)
                    {
                        printf("card reconnected, reinitialising\n");

                        esp_err_t ret;

                        sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
                        slot_config.gpio_cs = 4;
                        slot_config.host_id = host.slot;

                        printf("SDSPI configured\n");
                            
                        esp_vfs_fat_sdcard_unmount(mount_point, card);

                        printf("card unmounted\n");

                        ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

                        printf("attempting to mount card\n");

                        while (ret != ESP_OK) {
                            sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
                            slot_config.gpio_cs = 4;
                            slot_config.host_id = host.slot;

                            printf("SDSPI configured\n");
                            
                            esp_vfs_fat_sdcard_unmount(mount_point, card);

                            printf("card unmounted\n");
                            
                            ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

                            printf("attempting to mount card\n");

                            if (ret == ESP_FAIL) {
                                ESP_LOGE(TAG, "Failed to mount filesystem. "
                                        "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
                            } else {
                                ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                                        "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
                            }
                            
                        }
                        



                        FILE *f_gpx = fopen(gpx_file_path, "r+");

                        if (f_gpx == NULL)
                        {
                            generate_gpx_file(gpx_file_path);
                        }
                        else
                        {
                            if((unix_time - previous_point_time) <= 2)
                            {
                                fseek(f_gpx, -24, SEEK_END);
                                write_track_point(f_gpx, latitude, longitude, elevation, GPX_Time, HDOP);

                                fprintf(f_gpx, "</trkseg>\n");
                                fprintf(f_gpx, "</trk>\n");
                                fprintf(f_gpx, "</gpx>\n");
                            }
                            else
                            {
                                printf("starting new track");
                                fseek(f_gpx, -7, SEEK_END);
                                begin_new_track(f_gpx, GPX_Time);

                                fseek(f_gpx, -24, SEEK_END);
                                write_track_point(f_gpx, latitude, longitude, elevation, GPX_Time, HDOP);

                                fprintf(f_gpx, "</trkseg>\n");
                                fprintf(f_gpx, "</trk>\n");
                                fprintf(f_gpx, "</gpx>\n");
                            }
                            previous_point_time = unix_time;
                        }

                        CD_status_old = 1;
                    }

                    if (CD_status == 0)
                    {
                        printf("No Card Detected\n");

                        CD_status_old = 0;
                    }
            

                }
            }

            // Data is not valid
            else if (NMEA_data[1][2][0] == 'V')
            {
                //printf("Invalid data\n");
                gpio_set_level(GPS_STATUS_LED, 0);
                char gpx_file_path[23]; 
                strcpy(gpx_file_path, "\0");

                strcat(gpx_file_path, "/sdcard/DATA_TEST2.txt");

                CD_status = gpio_get_level(7);

                printf("CD_Status: %d CD_Status_Old: %d\n", CD_status, CD_status_old);

                counter += 1;

                    
                if (CD_status == 1 && CD_status_old == 1)
                {
                    printf("card connected\n");

                    FILE *f_gpx = fopen(gpx_file_path, "r+");

                    if (f_gpx == NULL)
                    {
                        generate_gpx_file(gpx_file_path);
                    }

                    //fseek(f_gpx, 0, SEEK_END);
                    //fprintf(f_gpx, "Test%d\n", counter);

                    //strcpy(gpx_file_path, "\0");
                    //strcpy(GPS_output, "\0");
                    fclose(f_gpx);
                    f_gpx = NULL;
                    
                }
                else if (CD_status == 1 && CD_status_old == 0)
                {
                    printf("card reconnected, reinitialising\n");
                    
                    esp_err_t ret;
                    
                    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
                    slot_config.gpio_cs = 4;
                    slot_config.host_id = host.slot;
                    
                    esp_vfs_fat_sdcard_unmount(mount_point, card);

                    //ESP_LOGI(TAG, "Mounting filesystem");
                    printf("Mounting Filesystem");

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
                    //ESP_LOGI(TAG, "Filesystem mounted");
                    printf("Filesystem Mounted");

                    //sdmmc_card_print_info(stdout, card);



                    FILE *f_gpx = fopen(gpx_file_path, "r+");

                    if (f_gpx == NULL)
                    {
                        generate_gpx_file(gpx_file_path);
                    }
                    else
                    {
                        fseek(f_gpx, 0, SEEK_END);
                        //fprintf(f_gpx, "Test - Reinitalised\n");

                        //strcpy(gpx_file_path, "\0");
                        //strcpy(GPS_output, "\0");
                        fclose(f_gpx);
                        f_gpx = NULL;
                    }

                    CD_status_old = 1;
                }

                if (CD_status == 0)
                {
                    printf("No Card Detected");

                    CD_status_old = 0;
                }
                
                
                
            }
            
            uart_flush(UART_NUM_1);   
        }     
    }
}

void app_main(void)
{
    GPIO_Setup();

    gpio_install_isr_service(0);

    // Attach ISR handler
    gpio_isr_handler_add(PIN_NUM_CD, gpio_isr_handler, (void*) PIN_NUM_CD);
    
    SD_Setup();

    UART_Setup();
    
    UART_Task();
}
