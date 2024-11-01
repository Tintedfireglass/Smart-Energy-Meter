#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include <stdio.h>
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "hardware/rtc.h"
#include "time.h"
#include "hardware/flash.h"
#include "string.h"
#include "lwip/apps/httpd.h"
#include "lwip/tcp.h"
#include "lwip/init.h"

// wifi credentials
const char WIFI_SSID[] = "Verma C1";
const char WIFI_PASS[] = "Vasuch3tan";

// ADC definitions
#define CURRENT_SENSOR 27
#define VOLTAGE_SENSOR 26
#define BUFFER_SIZE 512

// GPIO pins
#define RELAY_PIN 0
#define LED_PIN 1

// I2C definitions
#define LCD_ADDR 0x27
#define LCD_ENABLE 0x04
#define LCD_COMMAND 0x00
#define LCD_DATA 0x01
#define LCD_BACKLIGHT 0x08

// I2C pins
#define I2C_SCL 17
#define I2C_SDA 16

// Flash definitions
#define FLASH_TARGET_OFFSET (1 * 1024 * 1024)

// Datalogging Struct
typedef struct {
    uint8_t day;
    uint8_t month;
    uint16_t year;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t voltage;
    uint16_t current;
    uint32_t power;
    uint32_t energy;
}data;

// Variables for webserver
volatile data latest_readings = {0};
critical_section_t data_critsec;

#define ENTRIES_PER_PAGE (FLASH_PAGE_SIZE / sizeof(data)) // Calculate how many entries can fit

// variables
/************ADC VARIABLES************/
volatile uint16_t adc_buffer_0[BUFFER_SIZE];
volatile uint16_t adc_buffer_1[BUFFER_SIZE];
volatile bool buffer_0_ready = false;
volatile bool buffer_1_ready = false;
volatile uint16_t* current_dma_buffer = adc_buffer_0;
uint dma_chan;
// Timestamp each ADC batch with RTC
datetime_t timestamp;

/************CORE SHARED MEMORY VARIABLES************/
volatile bool data_ready = false;

/************DATALOGGING VARIABLES************/
// static data data_log[ENTRIES_PER_PAGE];           // Buffer to hold entries
uint8_t entry_count = 0;                       // Count of current entries
uint32_t flash_offset = FLASH_TARGET_OFFSET;        // Current flash write offset

data *data_log = {0};
data *read_from_flash = {0};
repeating_timer_t timer_one_minute;

/************GPIO INIT************/
// gpio_irq_callback_t gpio_callback(uint gpio, uint32_t events){
//     if(gpio == LED_PIN){
//         gpio_put(LED_PIN, !gpio_get(LED_PIN));
//     }
//     return 0;
// }

void init_all_gpio(){
    gpio_init(RELAY_PIN);
}

/************I2C INIT************/
void init_i2c_lcd(){
    i2c_init(i2c0, 100000);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SCL);
    gpio_pull_up(I2C_SDA);
}

/************I2C FUNCTIONS************/
void i2c_write_byte(uint8_t val){
    i2c_write_blocking(i2c0, LCD_ADDR, &val, 1, false);
}

void lcd_send_nibble(uint8_t nibble, uint8_t mode) {
    uint8_t data = (nibble << 4) | mode | LCD_BACKLIGHT;
    i2c_write_byte(data | LCD_ENABLE);
    sleep_us(1);
    i2c_write_byte(data & ~LCD_ENABLE);
    sleep_us(50);
}

void lcd_send_byte(uint8_t byte, uint8_t mode) {
    lcd_send_nibble(byte >> 4, mode);
    lcd_send_nibble(byte & 0x0F, mode);
}

void lcd_send_command(uint8_t cmd) {
    lcd_send_byte(cmd, LCD_COMMAND);
    sleep_us(2000);
}

void lcd_send_data(uint8_t data) {
    lcd_send_byte(data, LCD_DATA);
}

void lcd_clear() {
    lcd_send_command(0x01); // Clear display command
    sleep_ms(2); // Wait for the command to complete
}

void lcd_init() {
    sleep_ms(50);
    lcd_send_nibble(0x03, LCD_COMMAND);
    sleep_ms(5);
    lcd_send_nibble(0x03, LCD_COMMAND);
    sleep_us(150);
    lcd_send_nibble(0x03, LCD_COMMAND);
    lcd_send_nibble(0x02, LCD_COMMAND);

    lcd_send_command(0x28); // Function set: 4-bit mode, 2 lines, 5x8 dots
    lcd_send_command(0x0C); // Display on, cursor off, blink off
    lcd_send_command(0x06); // Entry mode set: increment cursor, no shift
    lcd_send_command(0x01); // Clear display
    sleep_ms(2);
}

void lcd_set_cursor(uint8_t col, uint8_t row) {
    static const uint8_t row_offsets[] = {0x00, 0x40};
    lcd_send_command(0x80 | (col + row_offsets[row]));
}

void lcd_print(const char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}
// core 1 setup

void core1_main();

void init_rtc(uint8_t day, uint8_t month, uint16_t year, uint8_t hour, uint8_t minute, uint8_t second) {
    datetime_t t = {
        .year  = year,
        .month = month,
        .day   = day,
        .dotw  = 0, // Day of the week, 0 = Sunday
        .hour  = hour,
        .min   = minute,
        .sec   = second
    };
    rtc_init();
    rtc_set_datetime(&t);
}

// Dma Interrupt handler
void dma_handler() {
    // Check if the interrupt is triggered by our DMA channel
    if (dma_hw->ints0 & (1u << dma_chan)) {
        // Clear the interrupt
        dma_hw->ints0 = (1u << dma_chan);

        // Signal which buffer is ready
        if (current_dma_buffer == adc_buffer_0) {
            buffer_0_ready = true;
            current_dma_buffer = adc_buffer_1;  // Switch to buffer 1
        } else {
            buffer_1_ready = true;
            current_dma_buffer = adc_buffer_0;  // Switch back to buffer 0
        }


        // Restart DMA for continuous capture
        dma_channel_set_read_addr(dma_chan, &adc_hw->fifo, true);
        dma_channel_set_write_addr(dma_chan, current_dma_buffer, true);
    }
}

/************ADC INIT SECTION************/
void adc_dma_init() {
    // Initialize ADC GPIO and ADC module
    adc_gpio_init(CURRENT_SENSOR);
    adc_gpio_init(VOLTAGE_SENSOR);
    adc_init();
    
    // Set ADC clock divider to 2
    adc_set_clkdiv(2);

    // adc round robin setup
    adc_set_round_robin(0x03); // 0x03 = 0b00000011 : Enables round-robin sampling for 2 ADC channels

    // ADC FIFO configuration
    adc_fifo_setup(
        true,   // Write each result to the FIFO
        true,   // Enable DMA data request (DREQ)
        1,      // DREQ when at least 1 sample in FIFO
        false,  // No error detection
        false   // No byte shifting
    );

    // Initialize DMA channel
    dma_chan = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_chan);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_16);
    channel_config_set_read_increment(&c, false);  // Read from the same FIFO address
    channel_config_set_write_increment(&c, true);  // Write incrementally to adc_buffer
    channel_config_set_dreq(&c, DREQ_ADC);         // Set DREQ to ADC FIFO

    // Configure the DMA channel
    dma_channel_configure(
        dma_chan,
        &c,
        current_dma_buffer,        // Destination buffer
        &adc_hw->fifo,     // Source (ADC FIFO)
        BUFFER_SIZE,     // Number of transfers
        true               // Start immediately
    );

    // Set up DMA interrupt on completion
    irq_set_priority(DMA_IRQ_0,1);
    dma_channel_set_irq0_enabled(dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    printf("ADC and DMA Initialized.\n");
}

/************ADC PROCESS SECTION************/
// Function to calculate energy and power from ADC data
data* calculate_energy_and_power(volatile uint16_t* buffer, size_t size) {
    float total_energy = 0.0f;
    float total_power = 0.0f;

    data* entry = (data*)malloc(sizeof(data));
    if (entry == NULL) {
        // Handle memory allocation failure
        return NULL;
    }

    for (size_t i = 0; i < size; i += 2) {
        float voltage = buffer[i] * (3.3f / 4095.0f); // adc 1 is current
        float current = buffer[i + 1] * (3.3f / 4095.0f); // adc 0 is voltage
        float power = voltage * current;

        total_power += power;
    }

    total_power /= (size / 2);

    entry->power = total_power;
    return entry;
}

// needs to send data to lcd every 0.5 seconds, update the display
void process_adc_data(volatile uint16_t* buffer, size_t size) {
    data* entry = calculate_energy_and_power(buffer, size);
    if (entry != NULL) {
        // Update LCD display
        char lcd_str[32];
        snprintf(lcd_str, sizeof(lcd_str), "Pwr:%.2fW Enr:%.2fWh", 
                (float)entry->power, (float)entry->energy);
        lcd_clear();
        lcd_set_cursor(0, 0);
        lcd_print(lcd_str);

        // Display timestamp
        char time_str[32];
        snprintf(time_str, sizeof(time_str), "%02d:%02d:%02d I:%d V:%d", 
                timestamp.hour, timestamp.min, timestamp.sec,
                entry->current, entry->voltage);
        lcd_set_cursor(0, 1);
        lcd_print(time_str);

        // Send data to Core 1 for web server
        multicore_fifo_push_blocking((uint32_t)entry);
        
        // Log data to flash
        log_data(entry);
        free(entry);
    }
}
void relay_control(bool state){
    gpio_put(RELAY_PIN, state);
    if(state == true)printf("Relay state: closed");
    else printf("Relay state: open");
}

/************DATALOGGING SECTION************/
void log_data(const data *entry) {
    // Copy the entry data into the next buffer slot
    data_log[entry_count++] = *entry;

    // If buffer is full, write it to flash
    if (entry_count >= ENTRIES_PER_PAGE) {
        uint32_t ints = save_and_disable_interrupts();  // Disable interrupts for safe flash access

        // Erase the sector if starting at a new sector
        if ((flash_offset % FLASH_SECTOR_SIZE) == 0) {
            flash_range_erase(flash_offset, FLASH_SECTOR_SIZE);
        }

        // Write the full buffer to flash
        flash_range_program(flash_offset, (const uint8_t*)data_log, sizeof(data) * entry_count);
        flash_offset += sizeof(data) * entry_count;  // Move to the next page in flash
        entry_count = 0;  // Reset buffer count

        restore_interrupts(ints);  // Re-enable interrupts
    }
}

// void read_from_flash(){
//     // uint32_t ints = save_and_disable_interrupts();
//     // flash_range_read(FLASH_TARGET_OFFSET, (uint8_t*)log, sizeof(data));
//     // restore_interrupts(ints);
    
//     // if(){
        
//     // }
// }


/************ WEB SERVER SECTION ************/


// CGI HANDLER FUNCTION
const char* get_readings_cgi_handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
    static char json_response[256];
    critical_section_enter_blocking(&data_critsec);
    snprintf(json_response, sizeof(json_response),
            "{\"voltage\":%d,\"current\":%d,\"power\":%lu,\"energy\":%lu,"
            "\"timestamp\":\"%02d:%02d:%02d %02d/%02d/%d\"}",
            latest_readings.voltage, latest_readings.current,
            latest_readings.power, latest_readings.energy,
            latest_readings.hour, latest_readings.minute, latest_readings.second,
            latest_readings.day, latest_readings.month, latest_readings.year);
    critical_section_exit(&data_critsec);
    return json_response;
}

const tCGI cgi_handlers[] = {
    {"/readings", get_readings_cgi_handler},
};

int main() {
    stdio_init_all();
    sleep_ms(3000);  // Delay to allow USB connection time

    // Initialise i2c bus 
    init_i2c_lcd();

    lcd_init();
    lcd_set_cursor(0, 0);
    lcd_print("Init...");

    lcd_set_cursor(0,1);
    lcd_print("SEM 1.0");

    sleep_ms(3000);
    
    lcd_clear();

    // Initialise adc and dma
    adc_dma_init();
    lcd_print("ADC init");

    // Start ADC
    adc_run(true);

    // Start core 1
    multicore_launch_core1(core1_main);

    while (true) {
        sleep_ms(1000);
        // Check if buffer 0 is ready and process it
        if (buffer_0_ready) {
            rtc_get_datetime(&timestamp);
            process_adc_data(adc_buffer_0, BUFFER_SIZE);
            buffer_0_ready = false;  // Reset the flag after processing
        }

        // Check if buffer 1 is ready and process it
        if (buffer_1_ready) {
            rtc_get_datetime(&timestamp);
            process_adc_data(adc_buffer_1, BUFFER_SIZE);
            buffer_1_ready = false;  // Reset the flag after processing
        }
    }

    return 0;
}

/************TIMER ISR SECTION************/
repeating_timer_callback_t timer_callback(repeating_timer_t *t){
    multicore_fifo_push_blocking(1);    
    return true;
}

void core1_main() {
    
    critical_section_init(&data_critsec);

    // Initialize WiFi
    if (cyw43_arch_init()) {
        printf("WiFi init failed\n");
        return;
    }

    cyw43_arch_enable_sta_mode();

    // Connect to WiFi
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASS, CYW43_AUTH_WPA2_AES_PSK, 10000) != 0) {
        printf("Failed to connect to WiFi... retrying\n");
        sleep_ms(1000);
    }
    printf("Connected to WiFi\n");

    // Initialize web server
    httpd_init();
    http_set_cgi_handlers(cgi_handlers, sizeof(cgi_handlers)/sizeof(tCGI));
    
    // Continuous polling send data to server
    while (true) {
        
        sleep_ms(10);  // server processing delay
        
        // Check for new data from Core 0
        if (multicore_fifo_rvalid()) {
            uint32_t signal = multicore_fifo_pop_blocking();
            if (signal == 1) {
                // Update latest readings in critical section
                critical_section_enter_blocking(&data_critsec);
                latest_readings = *((data*)signal);
                critical_section_exit(&data_critsec);
            }
        }
    }
}