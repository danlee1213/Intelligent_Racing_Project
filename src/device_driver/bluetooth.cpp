/*
 * bluetooth.cpp
 *
 *  Created on: 22 Mar 2019
 *      Author: LeeChunHei
 */

#include "device_driver/bluetooth.h"
#include <algorithm>
#include <assert.h>

#ifdef BLUETOOTH_USED

namespace DeviceDriver
{

const System::Pinout::Name bluetooth_pins[BLUETOOTH_USED * 2] = BLUETOOTH_PINS;

const Driver::Uart::Config Bluetooth::GetUartConfig(const Config &bluetooth_config, Driver::Uart::Uart_Listener tx_listener, Driver::Uart::Uart_Listener rx_listener)
{
    Driver::Uart::Config config;
    config.tx_pin = bluetooth_pins[bluetooth_config.id * 2];
    config.rx_pin = bluetooth_pins[bluetooth_config.id * 2 + 1];
    config.baud_rate = bluetooth_config.baud_rate;
    config.interrupt_priority = bluetooth_config.interrupt_priority;
    config.tx_empty_listener = tx_listener;
    config.rx_full_listener = rx_listener;
    config.rx_fifo_watermark = bluetooth_config.interrupt_threshold;
    return config;
}

Bluetooth::Bluetooth(const Config &config) : tx_buffer(config.tx_buffer_size), rx_buffer(config.rx_buffer_size), uart(GetUartConfig(config, [&](std::vector<uint8_t> &data) mutable { this->tx_buffer.Read(data, 4); if(config.tx_empty_listener)config.tx_empty_listener(this); }, [&](std::vector<uint8_t> &data) mutable { this->rx_buffer.Write(data);if(config.rx_full_listener&&rx_buffer.GetWrittenSize()>config.rx_full_threshold)config.rx_full_listener(this); }))
{
}

bool Bluetooth::SendBytes(const uint8_t *data, size_t size)
{
    std::vector<uint8_t> _data(data, data + size);
    return SendBytes(_data);
}

bool Bluetooth::SendBytes(const std::vector<uint8_t> &data)
{
    if(tx_buffer.Write(data)){
        uart.SetTXEmptyInterrupt(true);
        return true;
    }else{
        return false;
    }
}

bool Bluetooth::SendString(const char *data, size_t size)
{
    std::vector<uint8_t> _data(data, data + size);
    return SendBytes(_data);
}

bool Bluetooth::SendString(const std::string &data)
{
    std::vector<uint8_t> _data(data.begin(), data.end());
    return SendBytes(_data);
}

bool Bluetooth::GetBytes(uint8_t *data, size_t &size)
{
    std::vector<uint8_t> _data;
    bool retval = GetBytes(_data);
    std::copy(_data.begin(), _data.end(), data);
    size = _data.size();
    return retval;
}

bool Bluetooth::GetBytes(std::vector<uint8_t> &data)
{
    return rx_buffer.ReadAll(data);
}

bool Bluetooth::GetString(char *data, size_t &size)
{
    std::vector<uint8_t> _data;
    bool retval = GetBytes(_data);
    std::copy(_data.begin(), _data.end(), data);
    size = _data.size();
    return retval;
}

bool Bluetooth::GetString(std::string &data)
{
    std::vector<uint8_t> _data;
    bool retval = GetBytes(_data);
    data = std::string(_data.begin(), _data.end());
    std::copy(_data.begin(), _data.end(), data.begin());
    return retval;
}

} // namespace DeviceDriver

#endif