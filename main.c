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

bool OutputGLL = false;
bool OutputRMC = true;
bool OutputVTG = false;
bool OutputGGA = false;
bool OutputGSA = false;
bool OutputGSV = false;

void generate_gpx_file(const char* filename) {

    //char gpx_file[1024];

    //sprintf(gpx_file,"%s/%s.gpx",MOUNT_POINT,filename);

    //char *gpx_file = strcat("/sdcard","/");
    //char *gpx_file1 = strcat(gpx_file,filename);
    //char *gpx_file2 = strcat(gpx_file1,".gpx");

    //%s.gpx",filename);

    const char *gpx_file = "/sdcard/DATA_01042025.gpx";

    printf("%s", gpx_file);

    FILE *f_gpx = fopen(gpx_file, "wa");
    if (f_gpx == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
    }

    fprintf(f_gpx, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
    fprintf(f_gpx, "<gpx version=\"1.1\" creator=\"ESP32_GPS_Tracker\">\n");
    fprintf(f_gpx, "  <metadata>\n");
    fprintf(f_gpx, "    <name>GPS Data</name>\n");
    fprintf(f_gpx, "    <desc>A GPX file generated from EPS32 GPS tracker</desc>\n");
    fprintf(f_gpx, "  </metadata>\n");
}

void write_waypoint(FILE *file, double lat, double lon, const char* name, const char* desc) {
    fprintf(file, "  <wpt lat=\"%f\" lon=\"%f\">\n", lat, lon);
    fprintf(file, "    <name>%s</name>\n", name);
    fprintf(file, "    <desc>%s</desc>\n", desc);
    fprintf(file, "  </wpt>\n");
}

void begin_track(FILE *file) {
    fprintf(file, "  <trk><name>Example gpx</name><number>1</number><trkseg>\n");
}

void write_track_point(FILE *file) {
    fprintf(file, "  <trk><name>Example gpx</name><number>1</number><trkseg>\n");
}

void write_gpx_footer(FILE *file) {
    fprintf(file, "</gpx>\n");
}

static esp_err_t s_example_write_file(const char *path, char *data)
{
    FILE *f = fopen(path, "a");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);
    ESP_LOGI(TAG, "File written");

    return ESP_OK;
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

void Set_GPS_Baud(int baud) 
{
    
}

static void uart_task()
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



    char GPS_data[2048];

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


    uart_write_bytes(UART_NUM_1, (const char*)GPS_rate_command_fast, 19);
    uart_wait_tx_done(UART_NUM_1, 1000);



    //uart_flush(UART_NUM_1);

    while (1)
    {   
        int len = uart_read_bytes(UART_NUM_1, GPS_data, BUF_SIZE, 2);
        
        

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
            

            if (NMEA_data[1][2][0] == 'A')
            {
                
                gpio_set_level(GPS_STATUS_LED, 1);

                strcat(GPS_output, NMEA_data[1][1]);
                strcat(GPS_output, ",");
                strcat(GPS_output, NMEA_data[1][3]);
                strcat(GPS_output, ",");
                strcat(GPS_output, NMEA_data[1][4]);
                strcat(GPS_output, ",");
                strcat(GPS_output, NMEA_data[1][5]);
                strcat(GPS_output, ",");
                strcat(GPS_output, NMEA_data[1][6]);
                strcat(GPS_output, ",");
                strcat(GPS_output, NMEA_data[1][9]);
                strcat(GPS_output, ",");
                strcat(GPS_output, NMEA_data[3][8]);
                strcat(GPS_output, ",");
                strcat(GPS_output, NMEA_data[3][9]);
                strcat(GPS_output, ",");
                strcat(GPS_output, NMEA_data[3][11]);

                strcat(GPS_output, "\n\0");

                printf("%s\n", GPS_output);
                
                char file_name[6]; 

                const char *file = MOUNT_POINT "/data.txt";

                snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%s", GPS_output);
                s_example_write_file(file, data);

                strcpy(GPS_output, "\0");
            }
            else if (NMEA_data[1][2][0] == 'V')
            {
                gpio_set_level(GPS_STATUS_LED, 0);

                strcat(GPS_output, NMEA_data[1][1]);
                strcat(GPS_output, ",");
                strcat(GPS_output, NMEA_data[1][3]);
                strcat(GPS_output, ",");
                strcat(GPS_output, NMEA_data[1][4]);
                strcat(GPS_output, ",");
                strcat(GPS_output, NMEA_data[1][5]);
                strcat(GPS_output, ",");
                strcat(GPS_output, NMEA_data[1][6]);
                strcat(GPS_output, ",");
                strcat(GPS_output, NMEA_data[1][9]);
                
                strcat(GPS_output, "\n\0");
                printf("%s", GPS_output);

                strcpy(GPS_output, "\0");
            }
            
            
            
            
                
        }     
        
    }

    uart_flush(UART_NUM_1);

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

void LED_Setup(void)
{
    gpio_reset_pin(GPS_STATUS_LED);
    gpio_set_direction(GPS_STATUS_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(GPS_STATUS_LED, 0);
}


void app_main(void)
{
    LED_Setup();
    
    SD_Setup();

    char gpx_filename[14];

    strcpy(gpx_filename, "01012025_DATA");

     

    generate_gpx_file(gpx_filename);
    
    
    /*
    // Write some waypoints (latitude, longitude, name, description)
    write_waypoint(file2, 40.712776, -74.005974, "New York", "The Big Apple");
    write_waypoint(file2, 34.052235, -118.243683, "Los Angeles", "City of Angels");
    write_waypoint(file2, 51.507351, -0.127758, "London", "Capital of England");

    // Write footer
    write_gpx_footer(file2);*/

    //fclose(file2);

    
    
    uart_task();
}
