// This file is automatically generated, modify at your peril

/**
 * Used as the request field of IN control transfers
 */
enum usb_spi_ControlIn {
  REQUEST_IN_HW_INFO,
  REQUEST_IN_GET_EVENT,
  /**
   * Slave ID is sent in the value field
   */
  REQUEST_IN_LINUX_SLAVE_INFO,
};
typedef uint8_t usb_spi_ControlIn;

/**
 * Used as the request field of OUT control transfers
 */
enum usb_spi_ControlOut {
  SetSlave,
};
typedef uint8_t usb_spi_ControlOut;

enum usb_spi_EventType {
  NONE,
};
typedef uint8_t usb_spi_EventType;

/**
 * Functions as something like a communications protocol version
 */
enum usb_spi_HardwareType {
  SpiMaster,
};
typedef uint8_t usb_spi_HardwareType;

typedef struct usb_spi_MasterInfo {
  /**
   * Encode as SpiMaster
   */
  usb_spi_HardwareType hardware;
  uint16_t slave_count;
} usb_spi_MasterInfo;

/**
 * Returned by SPI masters for REQUEST_IN_LINUX_SLAVE_INFO
 */
typedef struct usb_spi_ConnectedSlaveInfoLinux {
  uint8_t has_interrupt;
  /**
   * NULL-terminated, 32 comes from Linux's SPI_NAME_SIZE TODO use const
   */
  uint8_t modalias[32];
} usb_spi_ConnectedSlaveInfoLinux;

typedef struct usb_spi_Event {
  usb_spi_EventType event;
} usb_spi_Event;
