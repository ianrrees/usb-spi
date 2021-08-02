#pragma once

// This file is automatically generated, modify at your peril

/**
 * Used as the request field of IN control transfers
 */
enum usb_spi_ControlIn {
  usb_spi_ControlIn_HwInfo,
  usb_spi_ControlIn_GetEvent,
  /**
   * Slave ID is sent in the value field
   */
  usb_spi_ControlIn_LinuxSlaveInfo,
};
typedef uint8_t usb_spi_ControlIn;

/**
 * USB convention of the directions used in this transfer
 *
 * USB is used because the protocol could be used for attaching SPI controllers
 * or peripherals, but the logic around headers going in OUT transfer wouldn't
 * change based on the SPI direction.
 */
enum usb_spi_Direction {
  usb_spi_Direction_None,
  usb_spi_Direction_OutOnly,
  usb_spi_Direction_InOnly,
  usb_spi_Direction_Both,
};
typedef uint8_t usb_spi_Direction;

enum usb_spi_EventType {
  usb_spi_EventType_NoEvent,
  usb_spi_EventType_Interrupt,
};
typedef uint8_t usb_spi_EventType;

/**
 * Functions as something like a communications protocol version
 */
enum usb_spi_HardwareType {
  usb_spi_HardwareType_SpiMaster,
};
typedef uint8_t usb_spi_HardwareType;

typedef struct usb_spi_MasterInfo {
  /**
   * Encode as SpiMaster
   */
  usb_spi_HardwareType hardware;
  uint16_t slave_count;
  /**
   * Bytes that can be read in by the SPI before USB IN transfers start
   */
  uint16_t in_buf_size;
} usb_spi_MasterInfo;

/**
 * Returned by SPI masters for REQUEST_IN_LINUX_SLAVE_INFO
 */
typedef struct usb_spi_ConnectedSlaveInfoLinux {
  /**
   * NULL-terminated, 32 comes from Linux's SPI_NAME_SIZE TODO use const
   */
  uint8_t modalias[32];
} usb_spi_ConnectedSlaveInfoLinux;

typedef struct usb_spi_Event {
  /**
   * For event types like Interrupt, this conveys which device is interrupting
   */
  uint16_t data;
  usb_spi_EventType event_type;
} usb_spi_Event;

/**
 * Sent through the bulk OUT endpoint, possibly before any OUT data
 */
typedef struct usb_spi_TransferHeader {
  uint16_t bytes;
  /**
   * Index of the slave device the transfer is intended for
   */
  uint16_t index;
  usb_spi_Direction direction;
} usb_spi_TransferHeader;
