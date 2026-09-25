// https://cdn.sparkfun.com/datasheets/Sensors/GPS/NMEA%20Reference%20Manual1.pdf
// https://cdn-shop.adafruit.com/datasheets/PMTK%20command%20packet-Complete-C39-A01.pdf
// https://cdn.sparkfun.com/assets/parts/1/2/2/8/0/PMTK_Packet_User_Manual.pdf



#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <time.h>
#include <math.h>
#include <errno.h>
#include <dirent.h>
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

#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_wifi_netif.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "esp_err.h"

#define GPS_STATUS_LED GPIO_NUM_6
#define BUF_SIZE (1024*8)
#define EXAMPLE_MAX_CHAR_SIZE    2048
#define MAX_PATH 32

char filename[100];
char error_msg[100];
char GPS_data[2048];
char data[EXAMPLE_MAX_CHAR_SIZE];
char GPS_output[EXAMPLE_MAX_CHAR_SIZE];
char NMEA_data [6][15][15];
char baud_rates[7][6] = {"4800", "9600", "14400", "19200", "38400", "57600", "115200"};
int baud_rate = 6;
int CD_status = 0;
int CD_status_old = 0;
char date_old[] = "700101";
bool card_inserted = false;
bool button_press = false;
int SDMMC_TIMEOUT_MS = 500;
const TickType_t xDelay = 1000 / portTICK_PERIOD_MS;
FILE *f_gpx = NULL;
FILE *f_error = NULL;

// Method to determine when to start a new tracking
int max_time_difference = 2; // Maximum time between points in seconds

// UART configuration for GPS module
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

// Define the pins for SPI communication with SD card
#define PIN_NUM_MISO  9
#define PIN_NUM_MOSI  10
#define PIN_NUM_CLK   8
#define PIN_NUM_CS    4
#define PIN_NUM_CD    7

// initalizes wifi stuff
#define WIFI_SSID     "1209-2.4"
#define WIFI_PASSWORD "103ducks"

#define SERVER_IP   "10.0.0.15"
#define SERVER_PORT 5000

static EventGroupHandle_t wifi_event_group;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int retry_count = 0;
#define MAX_RETRY 5


static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT &&
        event_id == WIFI_EVENT_STA_START) {

        ESP_LOGI(TAG, "Wi-Fi started");
        esp_wifi_connect();

    } else if (event_base == WIFI_EVENT &&
               event_id == WIFI_EVENT_STA_DISCONNECTED) {

        if (retry_count < MAX_RETRY) {
            retry_count++;
            ESP_LOGI(TAG, "Retrying Wi-Fi...");
            esp_wifi_connect();
        } else {
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }

    } else if (event_base == IP_EVENT &&
               event_id == IP_EVENT_STA_GOT_IP) {

        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;

        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));

        retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}


static void wifi_init_sta(void)
{
    wifi_event_group = xEventGroupCreate();

    // Initialize TCP/IP network interface
    ESP_ERROR_CHECK(esp_netif_init());

    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create default Wi-Fi station interface
    esp_netif_create_default_wifi_sta();

    // Initialize Wi-Fi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register Wi-Fi events
    ESP_ERROR_CHECK(
        esp_event_handler_register(
            WIFI_EVENT,
            ESP_EVENT_ANY_ID,
            &wifi_event_handler,
            NULL));

    ESP_ERROR_CHECK(
        esp_event_handler_register(
            IP_EVENT,
            IP_EVENT_STA_GOT_IP,
            &wifi_event_handler,
            NULL));

    // Wi-Fi configuration
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(
        esp_wifi_set_mode(WIFI_MODE_STA));

    ESP_ERROR_CHECK(
        esp_wifi_set_config(
            WIFI_IF_STA,
            &wifi_config));

    ESP_ERROR_CHECK(
        esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi initialization complete");

    // Wait for connection
    EventBits_t bits = xEventGroupWaitBits(
        wifi_event_group,
        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        portMAX_DELAY
    );

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to Wi-Fi");
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to Wi-Fi");
    }
}


void tcp_client_task(void *pvParameters)
{
    char rx_buffer[128];

    

    while (1) {

        ESP_LOGI(TAG, "Creating socket...");

        int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        struct sockaddr_in dest_addr = {
            .sin_family = AF_INET,
            .sin_port = htons(SERVER_PORT),
            .sin_addr.s_addr = inet_addr(SERVER_IP)
        };

        ESP_LOGI(TAG, "Connecting to %s:%d",
                 SERVER_IP, SERVER_PORT);

        int err = connect(
            sock,
            (struct sockaddr *)&dest_addr,
            sizeof(dest_addr)
        );

        if (err != 0) {
            ESP_LOGE(TAG, "Connection failed: errno %d", errno);
            close(sock);

            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "Connected!");

        // Send data
        const char *message = "lat long etc\n";

        int sent = send(
            sock,
            message,
            strlen(message),
            0
        );

        if (sent < 0) {
            ESP_LOGE(TAG, "Send failed");
            close(sock);

            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "Sent: %s", message);

        // Receive response
        int len = recv(
            sock,
            rx_buffer,
            sizeof(rx_buffer) - 1,
            0
        );

        if (len > 0) {
            rx_buffer[len] = '\0';

            ESP_LOGI(TAG, "Received: %s", rx_buffer);
        } else if (len == 0) {
            ESP_LOGI(TAG, "Server closed connection");
        } else {
            ESP_LOGE(TAG, "Receive failed");
        }

        // Close connection
        shutdown(sock, 0);
        close(sock);

        ESP_LOGI(TAG, "Socket closed");

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void backup_gpx_files(void)
{
    char rx_buffer[128];

    ESP_LOGI(TAG, "Creating socket...");

    int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);

    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket");
        vTaskDelay(pdMS_TO_TICKS(1000));
        
    }

    struct sockaddr_in dest_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(SERVER_PORT),
        .sin_addr.s_addr = inet_addr(SERVER_IP)
    };

    ESP_LOGI(TAG, "Connecting to %s:%d",
            SERVER_IP, SERVER_PORT);

    int err = connect(
        sock,
        (struct sockaddr *)&dest_addr,
        sizeof(dest_addr)
    );

    if (err != 0) {
        ESP_LOGE(TAG, "Connection failed: errno %d", errno);
        close(sock);

        vTaskDelay(pdMS_TO_TICKS(1000));
        
    }

    ESP_LOGI(TAG, "Connected!");


    struct dirent *entry;
    
    DIR *dir = opendir("/sdcard"); 

    if (dir == NULL) {
        printf("Could not open directory.\n"); 
    }

    // Loop through and print all file and folder names
    while ((entry = readdir(dir)) != NULL) {
        strcpy(filename, entry->d_name);

        if (strncmp(filename, "DATA", 4) == 0)
        {  
            // Send data
            //const char *message = "lat long etc\n";
            int sent = send(
                sock,
                filename,
                strlen(filename),
                0
            );

            if (sent < 0) {
                ESP_LOGE(TAG, "Send failed");
                close(sock);

                vTaskDelay(pdMS_TO_TICKS(1000));
                
            }  
        }   
    }

    closedir(dir);











    

    

    // Receive response
    int len = recv(
        sock,
        rx_buffer,
        sizeof(rx_buffer) - 1,
        0
    );

    if (len > 0) {
        rx_buffer[len] = '\0';

        ESP_LOGI(TAG, "Received: %s", rx_buffer);
    } else if (len == 0) {
        ESP_LOGI(TAG, "Server closed connection");
    } else {
        ESP_LOGE(TAG, "Receive failed");
    }

    // Close connection
    shutdown(sock, 0);
    close(sock);

    ESP_LOGI(TAG, "Socket closed");

    vTaskDelay(pdMS_TO_TICKS(1000));
    
}









void wifi_scan(void)
{
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = 0,
        .show_hidden = true,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 300,
    };

    ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));

    uint16_t ap_count = 0;

    // Get number of APs found
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));

    printf("\nFound %u access points:\n", ap_count);
    printf("------------------------------------------------------------\n");

    if (ap_count == 0) {
        return;
    }

    wifi_ap_record_t *ap_records =
        malloc(ap_count * sizeof(wifi_ap_record_t));

    if (ap_records == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory");
        return;
    }

    // Get AP records
    ESP_ERROR_CHECK(
        esp_wifi_scan_get_ap_records(&ap_count, ap_records)
    );

    for (int i = 0; i < ap_count; i++) {
        printf("%2d: SSID: %-32s  RSSI: %4d  CH: %2d  AUTH: %d\n",
               i + 1,
               (char *)ap_records[i].ssid,
               ap_records[i].rssi,
               ap_records[i].primary,
               ap_records[i].authmode);
    }

    printf("------------------------------------------------------------\n");

    free(ap_records);
}


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

// Validates checksum for NMEA data
int nmea0183_checksum(char *nmea_data)
{
    int crc = 0;
    int i;
    
    for (i = 1; i < strlen(nmea_data)-5; i ++) {
        crc ^= nmea_data[i];
    }

    return crc;
}

// Convert integer to hex string, might be removable
void intToHexString(int num, char *hexStr) 
{
    sprintf(hexStr, "%x", num);  
}

// Converts a laitude in the format degrees, minutes, decimal minutes to the decimal degree format 
float ConvertLatToDecimalDegrees(const char* LatStr, const char* LatDirection)
{
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

// Converts a longitude in the format degrees, minutes, decimal minutes to the decimal degree format 
float ConvertLongToDecimalDegrees(const char* LongStr, const char* LongDirection)
{
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

// Converts datetime output of GPS to GPX format: YYYY-MM-DDTHH:MM:SS.SSSZ
void ConvertDateandTimeFormat(const char* DateStr, const char* TimeStr, char* result_gpx, int *unix_time)
{
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

// If an error occurs, this function can write an error message to a text file on the SD card
void Write_Error(char *error_msg)
{
    char error_file_path[18] = "/sdcard/ERROR.txt\0";                   
    f_error = fopen(error_file_path, "a+");
    fprintf(f_error, error_msg);
    fclose(f_error);
}

// Initalizes SD card and checks if it is valid
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
    else
    {
        printf("SPI bus initialized\n");
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
    else
    {
        card_inserted = true;
        ESP_LOGI(TAG, "Filesystem mounted");
    }
    

    //sdmmc_card_print_info(stdout, card);

    CD_status_old = 0;

}

// Generates a new GPX file based on a provided filename if one does not exist
void generate_gpx_file(char *filename)
{
    printf("Generating file: %s\n", filename);

    FILE *f_gpx = fopen(filename, "a+");

    if (f_gpx == NULL)
    {
        ESP_LOGE(TAG, "Failed to create or open file for writing");
    }
    else
    {
        fprintf(f_gpx, "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n");
        fprintf(f_gpx, "<gpx creator=\"ESP32_GPS_Tracker\">\n");
        fprintf(f_gpx, " <metadata>\n");
        fprintf(f_gpx, "  <name>GPS Data</name>\n");
        fprintf(f_gpx, "  <desc>A GPX file generated from EPS32 GPS tracker</desc>\n");
        fprintf(f_gpx, " </metadata>\n");
        fprintf(f_gpx, "</gpx>\n");

        fclose(f_gpx);

        printf("file generated\n");
    }
}

// Writes a GPX waypoint to GPX file. Contains latitude, longitude, elevation, and the current date and time.
void write_waypoint(FILE *file, float lat, float lon, float ele, const char* time) {
    fprintf(file, "  <wpt lat=\"%.6f\" lon=\"%.6f\">\n", lat, lon);
    fprintf(file, "    <ele>%f</ele>\n", ele);
    fprintf(file, "    <time>%s</time>\n", time);
    fprintf(file, "  </wpt>\n");
}

void begin_new_track(FILE *file, const char* time) {
    fprintf(file, " <trk>\n");
    fprintf(file, "  <name>Track %s</name>\n", time);
    fprintf(file, "  <type>Placeholder</type>\n");
    fprintf(file, "   <trkseg>\n");
    fprintf(file, "   </trkseg>\n");
    fprintf(file, " </trk>\n");
    fprintf(file, "</gpx>\n");
}

void write_track_point(FILE *file, float lat, float lon, float ele, const char* time, int unix_time, float HDOP, float speed) {
    fprintf(file, "    <trkpt lat=\"%.6f\" lon=\"%.6f\"><ele>%f</ele><time>%s</time></trkpt><UNIX_TIME>%d</UNIX_TIME><HDOP>%f</HDOP><speed>%f</speed>\n", lat, lon, ele, time, unix_time, HDOP, speed);
}

void GPIO_Setup(void)
{
    // Sets up status LED's gpio pin
    gpio_reset_pin(GPS_STATUS_LED);
    gpio_set_direction(GPS_STATUS_LED, GPIO_MODE_OUTPUT);
    gpio_set_level(GPS_STATUS_LED, 0);

    gpio_set_direction(PIN_NUM_CD, GPIO_MODE_INPUT);

    gpio_set_intr_type(PIN_NUM_CD, GPIO_INTR_POSEDGE);
}

void card_reinitalization()
{
    int attempts = 0;

    esp_err_t ret;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = 4;
    slot_config.host_id = host.slot;
        
    esp_vfs_fat_sdcard_unmount(mount_point, card);

    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    while (ret != ESP_OK) {
        sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
        slot_config.gpio_cs = 4;
        slot_config.host_id = host.slot;
        
        esp_vfs_fat_sdcard_unmount(mount_point, card);
        
        ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
        if (attempts > 10)
        {
            card_inserted = false;
            printf("Card initialisation failed\n");
            return;
        }

        attempts += 1;
    }

    if (attempts <= 10)
    {
        card_inserted = true;

        sdmmc_card_print_info(stdout, card);

        printf("Card initialisation succeeded\n");
    }
    
}

void GPX_file_error_correction(char *filename)
{
    char filenamenew[256];

    FILE *f_gpx = fopen(filename, "r+");

    char gpx_char[128];

    if (f_gpx == NULL)
    {
        printf("Failed to create or open file for writing\n");

        fclose(f_gpx);
    }
    else
    {
        fseek(f_gpx,-7,SEEK_END);

        fread(gpx_char, sizeof(char), 7, f_gpx);

        gpx_char[7] = '\0';

        //printf("{%s}\n", gpx_char);

        if (strncmp(gpx_char, "</gpx>\n", 7) == 0)
        {
            //printf("file ok\n");
            fclose(f_gpx);
        }
        else
        {
            printf("file not ok\n");

            int x = 7;

            fseek(f_gpx, -x, SEEK_END);

            fread(gpx_char, sizeof(char), 7, f_gpx);

            gpx_char[7] = '\0';

            //printf("{%s}\n", gpx_char);

            x = 9;

            while (x < 256)
            {

                fseek(f_gpx, -x, SEEK_END);

                fread(gpx_char, sizeof(char), 8, f_gpx);

                gpx_char[8] = '\0';

                if (strncmp(gpx_char, "</speed>", 8) == 0)
                {
                    //printf("{%s}\n", gpx_char);

                    //printf("%d\n", strlen(gpx_char));

                    fseek(f_gpx, -x+8, SEEK_END);

                    fprintf(f_gpx,"\n  </trkseg>\n </trk>\n</gpx>\n");

                    x = 255;
                }
                x += 1;
        

            }

            fseek(f_gpx, 0, SEEK_SET);

            int filename_length = strlen(filename) - 4;

            strncpy(filenamenew, filename, filename_length);

            filenamenew[filename_length] = '\0';

            strcat(filenamenew, "-new.gpx");

            printf("%s\n", filenamenew);

            FILE *f_gpx_new = fopen(filenamenew, "a+");

            char buffer[512];

            while (fgets(buffer, sizeof(buffer), f_gpx) != NULL) {

                if (strncmp(buffer, "</gpx>\n", 7) == 0)
                {
                    fputs("</gpx>\n", f_gpx_new);
                    break;
                }
                else
                {
                    fputs(buffer, f_gpx_new);
                }
            }

            fclose(f_gpx_new);

            fclose(f_gpx);

            remove(filename);

            rename(filenamenew, filename);

        }

    }

}

//static const char *TAG2 = "GPIO_INT";

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;

    gpio_intr_disable(gpio_num);
    
    button_press = true;

    //gpio_intr_enable(gpio_num); 
}

void UART_Setup()
{
    const TickType_t xDelay = 500 / portTICK_PERIOD_MS;
    vTaskDelay(xDelay);


    uart_driver_install(UART_NUM_1, BUF_SIZE, BUF_SIZE, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, 2, 3, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);


    char* GPS_baud_rate_command = "$PMTK251,115200*1F\r\n";

    
    uart_set_baudrate(UART_NUM_1, 9600);

    uart_wait_tx_done(UART_NUM_1, 200);

    uart_write_bytes(UART_NUM_1, (const char*)GPS_baud_rate_command, 22);

    uart_wait_tx_done(UART_NUM_1, 100);

    uart_set_baudrate(UART_NUM_1, 115200);

    

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


    uart_write_bytes(UART_NUM_1, (const char*)GPS_rate_command_fast, strlen(GPS_rate_command_fast));
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

static void GPS_Read_Write_Task()
{
    int previous_point_time = 0;

    while (1)
    {
        // If rising edge detection flag is true, sd card is either mounted or unmounted. 
        if (button_press == true)
        {
            if (card_inserted == false)
            {
                card_reinitalization();
            }
            else if (card_inserted == true)
            {
                esp_vfs_fat_sdcard_unmount(mount_point, card);
                CD_status = 0;
                CD_status_old = 0;
                card_inserted = false;
            }
            
            // Delay after button press to ensure multiple triggers do not occur
            vTaskDelay(xDelay);
            gpio_intr_enable(PIN_NUM_CD);
            button_press = false;
            printf("Button was pressed\n");
        }


        int len = uart_read_bytes(UART_NUM_1, GPS_data, BUF_SIZE, 5);

        if (len>0 && card_inserted == true)
        { 
            GPS_data[len] = '\0';

            //printf(GPS_data);

            
            int j = 0;
            int k = 0;
            int l = 0;

            char Sentence_Data[6];

            Sentence_Data[5] = '\0';

            

            // GPS data is seperated into matrix
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
                
                float latitude;
                float longitude;
                char GPX_Time[30];
                int unix_time;
                float MSL = atof(NMEA_data[3][9]);
                float geoid_sep = atof(NMEA_data[3][11]);
                float elevation = MSL;
                float HDOP = atof(NMEA_data[3][8]);
                float speed = 0;
                int num_of_sat = atoi(NMEA_data[3][7]);

                //printf("%d\n",num_of_sat);
                //printf("%f\n",HDOP);
                //printf("%c\n\n",NMEA_data[1][2][0]);

                latitude = 0;
                longitude = 0;

                //printf("%f\n", HDOP);

                //printf("Time = %s, Number of satellites = %d, HDOP = %f\n", GPX_Time, num_of_sat, HDOP);
                
                // Filters valid data by precision
                if (HDOP <= 2.5)
                {
                    gpio_set_level(GPS_STATUS_LED, 1);

                }
                else
                {
                    Write_Error("HDOP > 2.5\n");
                    char error_msg[64];
                    
                    sprintf(error_msg, "Time = %s, Num. of Sats. = %d\n", GPX_Time, num_of_sat);

                    Write_Error(error_msg);
                    vTaskDelay(50);
                    gpio_set_level(GPS_STATUS_LED, 0);
                    vTaskDelay(50);
                    gpio_set_level(GPS_STATUS_LED, 1);
                }

                // Latitude, Longitude, and Time are all converted to a different format. See functions for details
                latitude = ConvertLatToDecimalDegrees(NMEA_data[1][3], NMEA_data[1][4]);
                longitude = ConvertLongToDecimalDegrees(NMEA_data[1][5], NMEA_data[1][6]);
                ConvertDateandTimeFormat(NMEA_data[1][9], NMEA_data[1][1], GPX_Time, &unix_time);

                speed = atof(NMEA_data[1][7]);

                //printf("Lat = %f, Long = %f, Time = %s, Num. of Sats. = %d\n", latitude, longitude, GPX_Time, num_of_sat);
                

                // File path is /sdcard/DATA_DDMMYY
                char gpx_file_path[26]; 
                strcpy(gpx_file_path, "\0");

                strcat(gpx_file_path, "/sdcard/DATA_");
                strcat(gpx_file_path, NMEA_data[1][9]);
                strcat(gpx_file_path, ".gpx");

                if (access(gpx_file_path, F_OK) == 0)
                {
                    //printf("File exists.\n");

                    GPX_file_error_correction(gpx_file_path);
                }

                f_gpx = fopen(gpx_file_path, "r+");

                
                if (errno == EIO)
                {
                    //printf("I/O error\n");
                    card_reinitalization();
                }

                if (f_gpx == NULL)
                {
                    fclose(f_gpx);
                    //printf("No such file or directory\n");
                    generate_gpx_file(gpx_file_path);
                    f_gpx = fopen(gpx_file_path, "r+");

                    if (date_old != NMEA_data[1][9])
                    {
                        fseek(f_gpx, -7, SEEK_END);
                        begin_new_track(f_gpx, GPX_Time);
                        previous_point_time = unix_time;
                    }

                }
                
            
                if((unix_time - previous_point_time) <= 2)
                {
                    fseek(f_gpx, -27, SEEK_END);
                    write_track_point(f_gpx, latitude, longitude, elevation, GPX_Time, unix_time, HDOP, speed);

                    fprintf(f_gpx, "   </trkseg>\n");
                    fprintf(f_gpx, " </trk>\n");
                    fprintf(f_gpx, "</gpx>\n");
                }
                else
                {
                    fseek(f_gpx, -7, SEEK_END);
                    begin_new_track(f_gpx, GPX_Time);

                    fseek(f_gpx, -27, SEEK_END);
                    write_track_point(f_gpx, latitude, longitude, elevation, GPX_Time, unix_time, HDOP, speed);

                    fprintf(f_gpx, "   </trkseg>\n");
                    fprintf(f_gpx, " </trk>\n");
                    fprintf(f_gpx, "</gpx>\n");
                }
                previous_point_time = unix_time;
                strcpy(date_old, NMEA_data[1][9]);

                fclose(f_gpx);
                f_gpx = NULL;
                            
            }

            // Data is not valid
            else if (NMEA_data[1][2][0] == 'V')
            {

                int num_of_sat = atoi(NMEA_data[3][7]);
                char GPX_Time[30];
                int unix_time;

                ConvertDateandTimeFormat(NMEA_data[1][9], NMEA_data[1][1], GPX_Time, &unix_time);
                //printf("Time = %s, Number of satellites = %d\n", GPX_Time, num_of_sat);
                
                vTaskDelay(50);
                gpio_set_level(GPS_STATUS_LED, 0);
                vTaskDelay(50);
                gpio_set_level(GPS_STATUS_LED, 1);

                CD_status = gpio_get_level(7); 
            }
            
            uart_flush(UART_NUM_1);   
        }     

        else if (card_inserted == false)
        {
            gpio_set_level(GPS_STATUS_LED, 0);
        }
    }
}






void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {

        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    wifi_init_sta();

    //xTaskCreate(tcp_client_task,"tcp_client",4096,NULL,5,NULL);

    // Perform scan
    //wifi_scan();

    GPIO_Setup();

    gpio_install_isr_service(0);
    
    gpio_isr_handler_add(PIN_NUM_CD, gpio_isr_handler, (void*) PIN_NUM_CD);
    
    SD_Setup();

    backup_gpx_files();

    UART_Setup();
 
    GPS_Read_Write_Task();
}
