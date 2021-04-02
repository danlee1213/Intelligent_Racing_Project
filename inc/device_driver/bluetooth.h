/*
 * bluetooth.h
 *
 *    Created on: 22 Mar 2019
 *            Author: LeeChunHei
 */

#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

#include "../config/config.h"
#include "driver/uart.h"
#include <string>
#include <vector>

#ifdef BLUETOOTH_USED

namespace DeviceDriver {

    template <class c_type>
    class Buffer {
    public:
        Buffer(size_t size) : buffer_size(size), write_index(0), read_index(0) {
            data = new c_type[size];
        }
        ~Buffer() {
            delete data;
        }
        bool Write(const c_type &_data) {
            if (GetRemainSize()) {
                data[write_index++ % buffer_size] = _data;
                return true;
            } else {
                return false;
            }
        }
        bool Write(const c_type *_data, const size_t size) {
            if (GetRemainSize() >= size) {
                for (size_t i = 0; i < size; ++i) {
                    data[write_index++ % buffer_size] = _data[i];
                }
                return true;
            } else {
                return false;
            }
        }
        bool Write(const std::vector<c_type> &_data) {
            if (GetRemainSize() >= _data.size()) {
                for (c_type d : _data) {
                    data[write_index++ % buffer_size] = d;
                }
                return true;
            } else {
                return false;
            }
        }

        bool Read(c_type &_data) {
            if (write_index - read_index) {
                _data = data[read_index++ % buffer_size];
                return true;
            } else {
                return false;
            }
        }

        bool Read(c_type *_data, size_t &size) {
            size = std::min(size, write_index - read_index);
            if (size) {
                for (size_t i = 0; i < size; ++i) {
                    _data[i] = data[read_index++ % buffer_size];
                }
                return true;
            } else {
                return false;
            }
        }

        bool Read(std::vector<c_type> &_data, size_t size) {
            size = std::min(size, write_index - read_index);
            if (size) {
                _data.reserve(size);
                for (size_t i = 0; i < size; ++i) {
                    _data.push_back(data[read_index++ % buffer_size]);
                }
                return true;
            } else {
                return false;
            }
        }

        bool ReadAll(std::vector<c_type> &_data) {
            if (GetWrittenSize()) {
                Read(_data, GetWrittenSize());
                return true;
            } else {
                return false;
            }
        }

        size_t GetRemainSize() {
            return buffer_size - (write_index - read_index);
        }

        size_t GetWrittenSize() {
            return write_index - read_index;
        }

    private:
        const size_t buffer_size;
        size_t write_index, read_index;
        c_type *data;
    };
    /*!
 * @addtogroup bluetooth Bluetooth
 * @ingroup device_driver
 * @{
 */
    /**
 * @brief Bluetooth driver
 * 
 * ## Example
 * ~~~~~~~~~~~~~~~{.cpp}
 * ~~~~~~~~~~~~~~~
 */
    class Bluetooth {
    public:
        typedef std::function<void(Bluetooth *bt)> Bluetooth_Listener;
        struct Config {
            uint8_t id;                                     /*!< id of the bluetooth need to init */
            uint32_t baud_rate;                             /*!< Baud rate of the uart communication */
            size_t tx_buffer_size = 100;                    /*!< TX trasmit buffer in size of byte */
            size_t rx_buffer_size = 100;                    /*!< RX receive buffer in size of byte */
            uint8_t interrupt_threshold = 0;                /*!< In range of 0 to 4, larger means system will be interrupted when more data is ready */
            uint8_t interrupt_priority = 15;                /*!< Interrupt priority for the listeners */
            Bluetooth_Listener tx_empty_listener = nullptr; /*!< Interrupt listener when tx trasmition is complete(tx buffer is empty) */
            Bluetooth_Listener rx_full_listener = nullptr;  /*!< Interrupt listener when number of data in rx buffer larger than rx_full_threshold */
            uint8_t rx_full_threshold = 0;                  /*!< In range of 0 to rx_buffer_size */
        };
        Bluetooth(const Config &config);

        /**
     * @brief Send a list of byte with the given size
     * 
     * @param data: the byte array pointer
     * @param size: number of byte need to be sent
     * 
     * @return true:  data is successfully send
     * @return false: data failed to send, could happen when tx buffer is full
     */
        bool SendBytes(const uint8_t *data, size_t size);

        /**
     * @brief Send data out
     * 
     * @param data: the data that need to be send
     * 
     * @return true:  data is successfully send
     * @return false: data failed to send, could happen when tx buffer is full
     */
        bool SendBytes(const std::vector<uint8_t> &data);

        /**
     * @brief Send a string with the given size
     * 
     * @param data: the character array pointer
     * @param size: number of character need to be sent
     * 
     * @return true:  data is successfully send
     * @return false: data failed to send, could happen when tx buffer is full
     */
        bool SendString(const char *data, size_t size);

        /**
     * @brief Send a STL string
     * 
     * @param data: the string need to be send
     * 
     * @return true:  data is successfully send
     * @return false: data failed to send, could happen when tx buffer is full
     */
        bool SendString(const std::string &data);

        /**
     * @brief Get a list of byte received in rx buffer
     * 
     * @param data: pointer to a byte array for storing the received byte
     * @param size: will assign the number of data written in the array after the function is finished
     * 
     * @return true:  there is data in rx buffer, and had written in the byte array
     * @return false: rx buffer is empty
     */
        bool GetBytes(uint8_t *data, size_t &size);

        /**
     * @brief Get the data received in rx buffer
     * 
     * @param data: received data will be stored inside this vector after the function called
     * 
     * @return true:  there is data in rx buffer, and had written in the data vector
     * @return false: rx buffer is empty
     */
        bool GetBytes(std::vector<uint8_t> &data);

        /**
     * @brief Get the string received in rx buffer
     * 
     * @param data: pointer to a character array for storing the received byte 
     * @param size: will assign the number of data written in the array after the function is finished
     * 
     * @return true:  there is data in rx buffer, and had written in the character array
     * @return false: rx buffer is empty
     */
        bool GetString(char *data, size_t &size);

        /**
     * @brief Get the string received in rx buffer
     * 
     * @param data: received string will be stored inside this STL string after function called
     * 
     * @return true:  there is data in rx buffer, and had written in the character array
     * @return false: rx buffer is empty
     */
        bool GetString(std::string &data);

        inline bool GetByte(uint8_t &c) {
            return rx_buffer.Read(c);
        }

    private:
        static const Driver::Uart::Config
        GetUartConfig(const Config &bluetooth_config, Driver::Uart::Uart_Listener tx_listener, Driver::Uart::Uart_Listener rx_listener);

        Buffer<uint8_t> tx_buffer;
        Buffer<uint8_t> rx_buffer;
        Driver::Uart uart;
    };

    /** @}*/

} // namespace DeviceDriver

#endif

#endif /* BLUETOOTH_H_ */