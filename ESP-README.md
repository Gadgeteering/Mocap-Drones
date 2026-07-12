
Reflash ESP-DRONE  --> https://github.com/Gadgeteering/esp-fc

## Flashing

1. Download and unpack selected firmware from [Releases Page](https://github.com/rtlopez/esp-fc/releases)
2. Visit [ESP Tool Website](https://espressif.github.io/esptool-js/)
3. Click "Connect" and choose device port in dialog
4. Add firmware file and set Flash Address to `0x00`
5. Click "Program"
6. After success power cycle board

![ESP-FC Flashing](/docs/images/esptool-js-flash-connect.png)


## Setup

After flashing you need to configure few things first:

 1. Connect to [Betaflight Configurator](https://github.com/betaflight/betaflight-configurator/releases) 
 2. Configure pinout
    

set pin_input_rx -1
set pin_output_0 6
set pin_output_1 5
set pin_output_2 3
set pin_output_3 4
set pin_buzzer 39
set pin_serial_0_tx 43
set pin_serial_0_rx 44
set pin_serial_1_tx -1
set pin_serial_1_rx -1
set pin_i2c_scl 10
set pin_i2c_sda 11
set pin_input_adc_0 2
set pin_input_adc_1 -1
set pin_spi_0_sck -1
set pin_spi_0_mosi -1
set pin_spi_0_miso -1
set pin_spi_cs_0 -1
set pin_spi_cs_1 -1
set pin_spi_cs_2 -1
set pin_buzzer_invert 1
save
reboot


3. Configure the motors
4. Configure receiver
5. 
set pin_serial_0_rx -1
set pin_serial_1_tx -1
set pin_serial_1_rx -1
