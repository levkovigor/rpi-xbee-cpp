/**
 * Copyright (c) 2009 Andrew Rapp. All rights reserved.
 *
 * This file is part of XBee-Arduino.
 *
 * XBee-Arduino is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * XBee-Arduino is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with XBee-Arduino.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef XBee_h
#define XBee_h

#include "HardwareSerial.h"
#include <inttypes.h>

#define SERIES_1
#define SERIES_2

// set to ATAP value of XBee. AP=2 is recommended
#define ATAP 2

#define START_BYTE 0x7e
#define ESCAPE 0x7d
#define XON 0x11
#define XOFF 0x13

// This value determines the size of the byte array for receiving RX packets
// Most users won't be dealing with packets this large so you can adjust this
// value to reduce memory consumption. But, remember that
// if a RX packet exceeds this size, it cannot be parsed!

// This value is determined by the largest packet size (100 byte payload + 64-bit address + option byte and rssi byte) of a series 1 radio
#define MAX_FRAME_DATA_SIZE 110

#define BROADCAST_ADDRESS 0xffff
#define ZB_BROADCAST_ADDRESS 0xfffe

// the non-variable length of the frame data (not including frame id or api id or variable data size (e.g. payload, at command set value)
#define ZB_TX_API_LENGTH 12
#define ZB_EXPLICIT_TX_API_LENGTH 18
#define TX_16_API_LENGTH 3
#define TX_64_API_LENGTH 9
#define AT_COMMAND_API_LENGTH 2
#define REMOTE_AT_COMMAND_API_LENGTH 13
// start/length(2)/api/frameid/checksum bytes
#define PACKET_OVERHEAD_LENGTH 6
// api is always the third byte in packet
#define API_ID_INDEX 3

// frame position of rssi byte
#define RX_16_RSSI_OFFSET 2
#define RX_64_RSSI_OFFSET 8

#define DEFAULT_FRAME_ID 1
#define NO_RESPONSE_FRAME_ID 0

// These are the parameters used by the XBee ZB modules when you do a
// regular "ZB TX request".
#define DEFAULT_ENDPOINT 232
#define DEFAULT_CLUSTER_ID 0x0011
#define DEFAULT_PROFILE_ID 0xc105

// TODO put in tx16 class
#define ACK_OPTION 0
#define DISABLE_ACK_OPTION 1
#define BROADCAST_OPTION 4

// RX options
#define ZB_PACKET_ACKNOWLEDGED 0x01
#define ZB_BROADCAST_PACKET 0x02

// not everything is implemented!
/**
 * Api Id constants
 */
#define TX_64_REQUEST 0x0
#define TX_16_REQUEST 0x1
#define AT_COMMAND_REQUEST 0x08
#define AT_COMMAND_QUEUE_REQUEST 0x09
#define REMOTE_AT_REQUEST 0x17
#define ZB_TX_REQUEST 0x10
#define ZB_EXPLICIT_TX_REQUEST 0x11
#define RX_64_RESPONSE 0x80
#define RX_16_RESPONSE 0x81
#define RX_64_IO_RESPONSE 0x82
#define RX_16_IO_RESPONSE 0x83
#define AT_RESPONSE 0x88
#define TX_STATUS_RESPONSE 0x89
#define MODEM_STATUS_RESPONSE 0x8a
#define ZB_RX_RESPONSE 0x90
#define ZB_EXPLICIT_RX_RESPONSE 0x91
#define ZB_TX_STATUS_RESPONSE 0x8b
#define ZB_IO_SAMPLE_RESPONSE 0x92
#define ZB_IO_NODE_IDENTIFIER_RESPONSE 0x95
#define AT_COMMAND_RESPONSE 0x88
#define REMOTE_AT_COMMAND_RESPONSE 0x97


/**
 * TX STATUS constants
 */
#define	SUCCESS 0x0
#define CCA_FAILURE 0x2
#define INVALID_DESTINATION_ENDPOINT_SUCCESS 0x15
#define	NETWORK_ACK_FAILURE 0x21
#define NOT_JOINED_TO_NETWORK 0x22
#define	SELF_ADDRESSED 0x23
#define ADDRESS_NOT_FOUND 0x24
#define ROUTE_NOT_FOUND 0x25
#define PAYLOAD_TOO_LARGE 0x74
// Returned by XBeeWithCallbacks::waitForStatus on timeout
#define XBEE_WAIT_TIMEOUT 0xff

// modem status
#define HARDWARE_RESET 0
#define WATCHDOG_TIMER_RESET 1
#define ASSOCIATED 2
#define DISASSOCIATED 3
#define SYNCHRONIZATION_LOST 4
#define COORDINATOR_REALIGNMENT 5
#define COORDINATOR_STARTED 6

#define ZB_BROADCAST_RADIUS_MAX_HOPS 0

#define ZB_TX_UNICAST 0
#define ZB_TX_BROADCAST 8

#define AT_OK 0
#define AT_ERROR  1
#define AT_INVALID_COMMAND 2
#define AT_INVALID_PARAMETER 3
#define AT_NO_RESPONSE 4

#define NO_ERROR 0
#define CHECKSUM_FAILURE 1
#define PACKET_EXCEEDS_BYTE_ARRAY_LENGTH 2
#define UNEXPECTED_START_BYTE 3

/**
 * C++11 introduced the constexpr as a hint to the compiler that things
 * can be evaluated at compiletime. This can help to remove
 * startup code for global objects, or otherwise help the compiler to
 * optimize. Since the keyword is introduced in C++11, but supporting
 * older compilers is a matter of removing the keyword, we use a macro
 * for this.
 */
#if __cplusplus >= 201103L
#define CONSTEXPR constexpr
#else
#define CONSTEXPR
#endif

/**
 * The super class of all XBee responses (RX packets)
 * Users should never attempt to create an instance of this class; instead
 * create an instance of a subclass
 * It is recommend to reuse subclasses to conserve memory
 */
class XBeeResponse {
public:
	//static const int MODEM_STATUS = 0x8a;
	/**
	 * Default constructor
	 */
	XBeeResponse();
	/**
	 * Returns Api Id of the response
	 */
	uint8_t getApiId();
	void setApiId(uint8_t apiId);
	/**
	 * Returns the MSB length of the packet
	 */
	uint8_t getMsbLength();
	void setMsbLength(uint8_t msbLength);
	/**
	 * Returns the LSB length of the packet
	 */
	uint8_t getLsbLength();
	void setLsbLength(uint8_t lsbLength);
	/**
	 * Returns the packet checksum
	 */
	uint8_t getChecksum();
	void setChecksum(uint8_t checksum);
	/**
	 * Returns the length of the frame data: all bytes after the api id, and prior to the checksum
	 * Note up to release 0.1.2, this was incorrectly including the checksum in the length.
	 */
	uint8_t getFrameDataLength();
	void setFrameData(uint8_t* frameDataPtr);
	/**
	 * Returns the buffer that contains the response.
	 * Starts with byte that follows API ID and includes all bytes prior to the checksum
	 * Length is specified by getFrameDataLength()
	 * Note: Unlike Digi's definition of the frame data, this does not start with the API ID..
	 * The reason for this is all responses include an API ID, whereas my frame data
	 * includes only the API specific data.
	 */
	uint8_t* getFrameData();

	void setFrameLength(uint8_t frameLength);
	// to support future 65535 byte packets I guess
	/**
	 * Returns the length of the packet
	 */
	uint16_t getPacketLength();
	/**
	 * Resets the response to default values
	 */
	void reset();
	/**
	 * Initializes the response
	 */
	void init();
#ifdef SERIES_2
	/**
	 * Call with instance of ZBTxStatusResponse class only if getApiId() == ZB_TX_STATUS_RESPONSE
	 * to populate response
	 */
	void getZBTxStatusResponse(XBeeResponse &response);
	/**
	 * Call with instance of ZBRxResponse class only if getApiId() == ZB_RX_RESPONSE
	 * to populate response
	 */
	void getZBRxResponse(XBeeResponse &response);
	/**
	 * Call with instance of ZBExplicitRxResponse class only if getApiId() == ZB_EXPLICIT_RX_RESPONSE
	 * to populate response
	 */
	void getZBExplicitRxResponse(XBeeResponse &response);
	/**
	 * Call with instance of ZBRxIoSampleResponse class only if getApiId() == ZB_IO_SAMPLE_RESPONSE
	 * to populate response
	 */
	void getZBRxIoSampleResponse(XBeeResponse &response);
#endif
#ifdef SERIES_1
	/**
	 * Call with instance of TxStatusResponse only if getApiId() == TX_STATUS_RESPONSE
	 */
	void getTxStatusResponse(XBeeResponse &response);
	/**
	 * Call with instance of Rx16Response only if getApiId() == RX_16_RESPONSE
	 */
	void getRx16Response(XBeeResponse &response);
	/**
	 * Call with instance of Rx64Response only if getApiId() == RX_64_RESPONSE
	 */
	void getRx64Response(XBeeResponse &response);
	/**
	 * Call with instance of Rx16IoSampleResponse only if getApiId() == RX_16_IO_RESPONSE
	 */
	void getRx16IoSampleResponse(XBeeResponse &response);
	/**
	 * Call with instance of Rx64IoSampleResponse only if getApiId() == RX_64_IO_RESPONSE
	 */
	void getRx64IoSampleResponse(XBeeResponse &response);
#endif
	/**
	 * Call with instance of AtCommandResponse only if getApiId() == AT_COMMAND_RESPONSE
	 */
	void getAtCommandResponse(XBeeResponse &responses);
	/**
	 * Call with instance of RemoteAtCommandResponse only if getApiId() == REMOTE_AT_COMMAND_RESPONSE
	 */
	void getRemoteAtCommandResponse(XBeeResponse &response);
	/**
	 * Call with instance of ModemStatusResponse only if getApiId() == MODEM_STATUS_RESPONSE
	 */
	void getModemStatusResponse(XBeeResponse &response);
	/**
	 * Returns true if the response has been successfully parsed and is complete and ready for use
	 */
	bool isAvailable();
	void setAvailable(bool complete);
	/**
	 * Returns true if the response contains errors
	 */
	bool isError();
	/**
	 * Returns an error code, or zero, if successful.
	 * Error codes include: CHECKSUM_FAILURE, PACKET_EXCEEDS_BYTE_ARRAY_LENGTH, UNEXPECTED_START_BYTE
	 */
	uint8_t getErrorCode();
	void setErrorCode(uint8_t errorCode);
protected:
	// pointer to frameData
	uint8_t* _frameDataPtr;
private:
	void setCommon(XBeeResponse &target);
	uint8_t _apiId;
	uint8_t _msbLength;
	uint8_t _lsbLength;
	uint8_t _checksum;
	uint8_t _frameLength;
	bool _complete;
	uint8_t _errorCode;
};

class XBeeAddress {
public:
	CONSTEXPR XBeeAddress() {};
};

/**
 * Represents a 64-bit XBee Address
 *
 * Note that avr-gcc as of 4.9 doesn't optimize uint64_t very well, so
 * for the smallest and fastest code, use msb and lsb separately. See
 * https://gcc.gnu.org/bugzilla/show_bug.cgi?id=66511
 */
class XBeeAddress64 : public XBeeAddress {
public:
	CONSTEXPR XBeeAddress64(uint64_t addr) : _msb(addr >> 32), _lsb(addr) {}
	CONSTEXPR XBeeAddress64(uint32_t msb, uint32_t lsb) : _msb(msb), _lsb(lsb) {}
	CONSTEXPR XBeeAddress64() : _msb(0), _lsb(0) {}
	uint32_t getMsb() {return _msb;}
	uint32_t getLsb() {return _lsb;}
	uint64_t get() {return (static_cast<uint64_t>(_msb) << 32) | _lsb;}
	operator uint64_t() {return get();}
	void setMsb(uint32_t msb) {_msb = msb;}
	void setLsb(uint32_t lsb) {_lsb = lsb;}
	void set(uint64_t addr) {
		_msb = addr >> 32;
		_lsb = addr;
	}
private:
	// Once https://gcc.gnu.org/bugzilla/show_bug.cgi?id=66511 is
	// fixed, it might make sense to merge these into a uint64_t.
	uint32_t _msb;
	uint32_t _lsb;
};

//class XBeeAddress16 : public XBeeAddress {
//public:
//	XBeeAddress16(uint16_t addr);
//	XBeeAddress16();
//	uint16_t getAddress();
//	void setAddress(uint16_t addr);
//private:
//	uint16_t _addr;
//};

/**
 * This class is extended by all Responses that include a frame id
 */
class FrameIdResponse : public XBeeResponse {
public:
	FrameIdResponse();
	uint8_t getFrameId();
private:
	uint8_t _frameId;
};

/**
 * Common functionality for both Series 1 and 2 data RX data packets
 */
class RxDataResponse : public XBeeResponse {
public:
	RxDataResponse();
	/**
	 * Returns the specified index of the payload.  The index may be 0 to getDataLength() - 1
	 * This method is deprecated; use uint8_t* getData()
	 */
	uint8_t getData(int index);
	/**
	 * Returns the payload array.  This may be accessed from index 0 to getDataLength() - 1
	 */
	uint8_t* getData();
	/**
	 * Returns the length of the payload
	 */
	virtual uint8_t getDataLength() = 0;
	/**
	 * Returns the position in the frame data where the data begins
	 */
	virtual uint8_t getDataOffset() = 0;
};

// getResponse to return the proper subclass:
// we maintain a pointer to each type of response, when a response is parsed, it is allocated only if NULL
// can we allocate an object in a function?

#ifdef SERIES_2
/**
 * Represents a Series 2 TX status packet
 */
class ZBTxStatusResponse : public FrameIdResponse {
	public:
		ZBTxStatusResponse();
		uint16_t getRemoteAddress();
		uint8_t getTxRetryCount();
		uint8_t getDeliveryStatus();
		uint8_t getDiscoveryStatus();
		bool isSuccess();

	static const uint8_t API_ID = ZB_TX_STATUS_RESPONSE;
};

/**
 * Represents a Series 2 RX packet
 */
class ZBRxResponse : public RxDataResponse {
public:
	ZBRxResponse();
	XBeeAddress64& getRemoteAddress64();
	uint16_t getRemoteAddress16();
	uint8_t getOption();
	uint8_t getDataLength();
	// frame position where data starts
	uint8_t getDataOffset();

	static const uint8_t API_ID = ZB_RX_RESPONSE;
private:
	XBeeAddress64 _remoteAddress64;
};

/**
 * Represents a Series 2 Explicit RX packet
 *
 * Note: The receive these responses, set AO=1. With the default AO=0,
 * you will receive ZBRxResponses, not knowing exact details.
 */
class ZBExplicitRxResponse : public ZBRxResponse {
public:
	ZBExplicitRxResponse();
	uint8_t getSrcEndpoint();
	uint8_t getDstEndpoint();
	uint16_t getClusterId();
	uint16_t getProfileId();
	uint8_t getOption();
	uint8_t getDataLength();
	// frame position where data starts
	uint8_t getDataOffset();

	static const uint8_t API_ID = ZB_EXPLICIT_RX_RESPONSE;
};

/**
 * Represents a Series 2 RX I/O Sample packet
 */
class ZBRxIoSampleResponse : public ZBRxResponse {
public:
	ZBRxIoSampleResponse();
	bool containsAnalog();
	bool containsDigital();
	/**
	 * Returns true if the pin is enabled
	 */
	bool isAnalogEnabled(uint8_t pin);
	/**
	 * Returns true if the pin is enabled
	 */
	bool isDigitalEnabled(uint8_t pin);
	/**
	 * Returns the 10-bit analog reading of the specified pin.
	 * Valid pins include ADC:xxx.
	 */
	uint16_t getAnalog(uint8_t pin);
	/**
	 * Returns true if the specified pin is high/on.
	 * Valid pins include DIO:xxx.
	 */
	bool isDigitalOn(uint8_t pin);
	uint8_t getDigitalMaskMsb();
	uint8_t getDigitalMaskLsb();
	uint8_t getAnalogMask();

	static const uint8_t API_ID = ZB_IO_SAMPLE_RESPONSE;
};

#endif

#ifdef SERIES_1
/**
 * Represents a Series 1 TX Status packet
 */
class TxStatusResponse : public FrameIdResponse {
	public:
		TxStatusResponse();
		uint8_t getStatus();
		bool isSuccess();

	static const uint8_t API_ID = TX_STATUS_RESPONSE;
};

/**
 * Represents a Series 1 RX packet
 */
class RxResponse : public RxDataResponse {
public:
	RxResponse();
	// remember rssi is negative but this is unsigned byte so it's up to you to convert
	uint8_t getRssi();
	uint8_t getOption();
	bool isAddressBroadcast();
	bool isPanBroadcast();
	uint8_t getDataLength();
	uint8_t getDataOffset();
	virtual uint8_t getRssiOffset() = 0;
};

/**
 * Represents a Series 1 16-bit address RX packet
 */
class Rx16Response : public RxResponse {
public:
	Rx16Response();
	uint8_t getRssiOffset();
	uint16_t getRemoteAddress16();

	static const uint8_t API_ID = RX_16_RESPONSE;
protected:
	uint16_t _remoteAddress;
};

/**
 * Represents a Series 1 64-bit address RX packet
 */
class Rx64Response : public RxResponse {
public:
	Rx64Response();
	uint8_t getRssiOffset();
	XBeeAddress64& getRemoteAddress64();

	static const uint8_t API_ID = RX_64_RESPONSE;
private:
	XBeeAddress64 _remoteAddress;
};

/**
 * Represents a Series 1 RX I/O Sample packet
 */
class RxIoSampleBaseResponse : public RxResponse {
	public:
		RxIoSampleBaseResponse();
		/**
		 * Returns the number of samples in this packet
		 */
		uint8_t getSampleSize();
		bool containsAnalog();
		bool containsDigital();
		/**
		 * Returns true if the specified analog pin is enabled
		 */
		bool isAnalogEnabled(uint8_t pin);
		/**
		 * Returns true if the specified digital pin is enabled
		 */
		bool isDigitalEnabled(uint8_t pin);
		/**
		 * Returns the 10-bit analog reading of the specified pin.
		 * Valid pins include ADC:0-5.  Sample index starts at 0
		 */
		uint16_t getAnalog(uint8_t pin, uint8_t sample);
		/**
		 * Returns true if the specified pin is high/on.
		 * Valid pins include DIO:0-8.  Sample index starts at 0
		 */
		bool isDigitalOn(uint8_t pin, uint8_t sample);
		uint8_t getSampleOffset();

		/**
		 * Gets the offset of the start of the given sample.
		 */
		uint8_t getSampleStart(uint8_t sample);
	private:
};

class Rx16IoSampleResponse : public RxIoSampleBaseResponse {
public:
	Rx16IoSampleResponse();
	uint16_t getRemoteAddress16();
	uint8_t getRssiOffset();

	static const uint8_t API_ID = RX_16_IO_RESPONSE;
};

class Rx64IoSampleResponse : public RxIoSampleBaseResponse {
public:
	Rx64IoSampleResponse();
	XBeeAddress64& getRemoteAddress64();
	uint8_t getRssiOffset();

	static const uint8_t API_ID = RX_64_IO_RESPONSE;
private:
	XBeeAddress64 _remoteAddress;
};

#endif

/**
 * Represents a Modem Status RX packet
 */
class ModemStatusResponse : public XBeeResponse {
public:
	ModemStatusResponse();
	uint8_t getStatus();

	static const uint8_t API_ID = MODEM_STATUS_RESPONSE;
};

/**
 * Represents an AT Command RX packet
 */
class AtCommandResponse : public FrameIdResponse {
	public:
		AtCommandResponse();
		/**
		 * Returns an array containing the two character command
		 */
		uint8_t* getCommand();
		/**
		 * Returns the command status code.
		 * Zero represents a successful command
		 */
		uint8_t getStatus();
		/**
		 * Returns an array containing the command value.
		 * This is only applicable to query commands.
		 */
		uint8_t* getValue();
		/**
		 * Returns the length of the command value array.
		 */
		uint8_t getValueLength();
		/**
		 * Returns true if status equals AT_OK
		 */
		bool isOk();

		static const uint8_t API_ID = AT_COMMAND_RESPONSE;
};

/**
 * Represents a Remote AT Command RX packet
 */
class RemoteAtCommandResponse : public AtCommandResponse {
	public:
		RemoteAtCommandResponse();
		/**
		 * Returns an array containing the two character command
		 */
		uint8_t* getCommand();
		/**
		 * Returns the command status code.
		 * Zero represents a successful command
		 */
		uint8_t getStatus();
		/**
		 * Returns an array containing the command value.
		 * This is only applicable to query commands.
		 */
		uint8_t* getValue();
		/**
		 * Returns the length of the command value array.
		 */
		uint8_t getValueLength();
		/**
		 * Returns the 16-bit address of the remote radio
		 */
		uint16_t getRemoteAddress16();
		/**
		 * Returns the 64-bit address of the remote radio
		 */
		XBeeAddress64& getRemoteAddress64();
		/**
		 * Returns true if command was successful
		 */
		bool isOk();

		static const uint8_t API_ID = REMOTE_AT_COMMAND_RESPONSE;
	private:
		XBeeAddress64 _remoteAddress64;
};


/**
 * Super class of all XBee requests (TX packets)
 * Users should never create an instance of this class; instead use an subclass of this class
 * It is recommended to reuse Subclasses of the class to conserve memory
 * <p/>
 * This class allocates a buffer to
 */
class XBeeRequest {
public:
	/**
	 * Constructor
	 * TODO make protected
	 */
	XBeeRequest(uint8_t apiId, uint8_t frameId);
	/**
	 * Sets the frame id.  Must be between 1 and 255 inclusive to get a TX status response.
	 */
	void setFrameId(uint8_t frameId);
	/**
	 * Returns the frame id
	 */
	uint8_t getFrameId();
	/**
	 * Returns the API id
	 */
	uint8_t getApiId();
	// setting = 0 makes this a pure virtual function, meaning the subclass must implement, like abstract in java
	/**
	 * Starting after the frame id (pos = 0) and up to but not including the checksum
	 * Note: Unlike Digi's definition of the frame data, this does not start with the API ID.
	 * The reason for this is the API ID and Frame ID are common to all requests, whereas my definition of
	 * frame data is only the API specific data.
	 */
	virtual uint8_t getFrameData(uint8_t pos) = 0;
	/**
	 * Returns the size of the api frame (not including frame id or api id or checksum).
	 */
	virtual uint8_t getFrameDataLength() = 0;
	//void reset();
protected:
	void setApiId(uint8_t apiId);
private:
	uint8_t _apiId;
	uint8_t _frameId;
};

// TODO add reset/clear method since responses are often reused
/**
 * Primary interface for communicating with an XBee Radio.
 * This class provides methods for sending and receiving packets with an XBee radio via the serial port.
 * The XBee radio must be configured in API (packet) mode (AP=2)
 * in order to use this software.
 * <p/>
 * Since this code is designed to run on a microcontroller, with only one thread, you are responsible for reading the
 * data off the serial buffer in a timely manner.  This involves a call to a variant of readPacket(...).
 * If your serial port is receiving data faster than you are reading, you can expect to lose packets.
 * Arduino only has a 128 byte serial buffer so it can easily overflow if two or more packets arrive
 * without a call to readPacket(...)
 * <p/>
 * In order to conserve resources, this class only supports storing one response packet in memory at a time.
 * This means that you must fully consume the packet prior to calling readPacket(...), because calling
 * readPacket(...) overwrites the previous response.
 * <p/>
 * This class creates an array of size MAX_FRAME_DATA_SIZE for storing the response packet.  You may want
 * to adjust this value to conserve memory.
 *
 * \author Andrew Rapp
 */
class XBee {
public:
	XBee();
	/**
	 * Reads all available serial bytes until a packet is parsed, an error occurs, or the buffer is empty.
	 * You may call <i>xbee</i>.getResponse().isAvailable() after calling this method to determine if
	 * a packet is ready, or <i>xbee</i>.getResponse().isError() to determine if
	 * a error occurred.
	 * <p/>
	 * This method should always return quickly since it does not wait for serial data to arrive.
	 * You will want to use this method if you are doing other timely stuff in your loop, where
	 * a delay would cause problems.
	 * NOTE: calling this method resets the current response, so make sure you first consume the
	 * current response
	 */
	void readPacket();
	/**
	 * Waits a maximum of <i>timeout</i> milliseconds for a response packet before timing out; returns true if packet is read.
	 * Returns false if timeout or error occurs.
	 */
	bool readPacket(int timeout);
	/**
	 * Reads until a packet is received or an error occurs.
	 * Caution: use this carefully since if you don't get a response, your Arduino code will hang on this
	 * call forever!! often it's better to use a timeout: readPacket(int)
	 */
	void readPacketUntilAvailable();
	/**
	 * Starts the serial connection on the specified serial port
	 */
	void begin(Stream &serial);
	void getResponse(XBeeResponse &response);
	/**
	 * Returns a reference to the current response
	 * Note: once readPacket is called again this response will be overwritten!
	 */
	XBeeResponse& getResponse();
	/**
	 * Sends a XBeeRequest (TX packet) out the serial port
	 */
	void send(XBeeRequest &request);
	//uint8_t sendAndWaitForResponse(XBeeRequest &request, int timeout);
	/**
	 * Returns a sequential frame id between 1 and 255
	 */
	uint8_t getNextFrameId();
	/**
	 * Specify the serial port.  Only relevant for Arduinos that support multiple serial ports (e.g. Mega)
	 */
	void setSerial(Stream &serial);
private:
	bool available();
	uint8_t read();
	void flush();
	void write(uint8_t val);
	void sendByte(uint8_t b, bool escape);
	void resetResponse();
	XBeeResponse _response;
	bool _escape;
	// current packet position for response.  just a state variable for packet parsing and has no relevance for the response otherwise
	uint8_t _pos;
	// last byte read
	uint8_t b;
	uint8_t _checksumTotal;
	uint8_t _nextFrameId;
	// buffer for incoming RX packets.  holds only the api specific frame data, starting after the api id byte and prior to checksum
	uint8_t _responseFrameData[MAX_FRAME_DATA_SIZE];
	Stream* _serial;
};


/**
 * This class can be used instead of the XBee class and allows
 * user-specified callback functions to be called when responses are
 * received, simplifying the processing code and reducing boilerplate.
 *
 * To use it, first register your callback functions using the onXxx
 * methods. Each method has a uintptr_t data argument, that can be used to
 * pass arbitrary data to the callback (useful when using the same
 * function for multiple callbacks, or have a generic function that can
 * behave differently in different circumstances). Supplying the data
 * parameter is optional, but the callback must always accept it (just
 * ignore it if it's unused). The uintptr_t type is an integer type
 * guaranteed to be big enough to fit a pointer (it is 16-bit on AVR,
 * 32-bit on ARM), so it can also be used to store a pointer to access
 * more data if required (using proper casts).
 *
 * There can be only one callback of each type registered at one time,
 * so registering callback overwrites any previously registered one. To
 * unregister a callback, pass NULL as the function.
 *
 * To ensure that the callbacks are actually called, call the loop()
 * method regularly (in your loop() function, for example). This takes
 * care of calling readPacket() and getResponse() other methods on the
 * XBee class, so there is no need to do so directly (though it should
 * not mess with this class if you do, it would only mean some callbacks
 * aren't called).
 *
 * Inside callbacks, you should generally not be blocking / waiting.
 * Since callbacks can be called from inside waitFor() and friends, a
 * callback that doesn't return quickly can mess up the waitFor()
 * timeout.
 *
 * Sending packets is not a problem inside a callback, but avoid
 * receiving a packet (e.g. calling readPacket(), loop() or waitFor()
 * and friends) inside a callback (since that would overwrite the
 * current response, messing up any pending callbacks and waitFor() etc.
 * methods already running).
 */
class XBeeWithCallbacks : public XBee {
public:

	/**
	 * Register a packet error callback. It is called whenever an
	 * error occurs in the packet reading process. Arguments to the
	 * callback will be the error code (as returned by
	 * XBeeResponse::getErrorCode()) and the data parameter.  while
	 * registering the callback.
	 */
	void onPacketError(void (*func)(uint8_t, uintptr_t), uintptr_t data = 0) { _onPacketError.set(func, data); }

	/**
	 * Register a response received callback. It is called whenever
	 * a response was succesfully received, before a response
	 * specific callback (or onOtherResponse) below is called.
	 *
	 * Arguments to the callback will be the received response and
	 * the data parameter passed while registering the callback.
	 */
	void onResponse(void (*func)(XBeeResponse&, uintptr_t), uintptr_t data = 0) { _onResponse.set(func, data); }

	/**
	 * Register an other response received callback. It is called
	 * whenever a response was succesfully received, but no response
	 * specific callback was registered using the functions below
	 * (after the onResponse callback is called).
	 *
	 * Arguments to the callback will be the received response and
	 * the data parameter passed while registering the callback.
	 */
	void onOtherResponse(void (*func)(XBeeResponse&, uintptr_t), uintptr_t data = 0) { _onOtherResponse.set(func, data); }

	// These functions register a response specific callback. They
	// are called whenever a response of the appropriate type was
	// succesfully received (after the onResponse callback is
	// called).
	//
	// Arguments to the callback will be the received response
	// (already converted to the appropriate type) and the data
	// parameter passed while registering the callback.
	void onZBTxStatusResponse(void (*func)(ZBTxStatusResponse&, uintptr_t), uintptr_t data = 0) { _onZBTxStatusResponse.set(func, data); }
	void onZBRxResponse(void (*func)(ZBRxResponse&, uintptr_t), uintptr_t data = 0) { _onZBRxResponse.set(func, data); }
	void onZBExplicitRxResponse(void (*func)(ZBExplicitRxResponse&, uintptr_t), uintptr_t data = 0) { _onZBExplicitRxResponse.set(func, data); }
	void onZBRxIoSampleResponse(void (*func)(ZBRxIoSampleResponse&, uintptr_t), uintptr_t data = 0) { _onZBRxIoSampleResponse.set(func, data); }
	void onTxStatusResponse(void (*func)(TxStatusResponse&, uintptr_t), uintptr_t data = 0) { _onTxStatusResponse.set(func, data); }
	void onRx16Response(void (*func)(Rx16Response&, uintptr_t), uintptr_t data = 0) { _onRx16Response.set(func, data); }
	void onRx64Response(void (*func)(Rx64Response&, uintptr_t), uintptr_t data = 0) { _onRx64Response.set(func, data); }
	void onRx16IoSampleResponse(void (*func)(Rx16IoSampleResponse&, uintptr_t), uintptr_t data = 0) { _onRx16IoSampleResponse.set(func, data); }
	void onRx64IoSampleResponse(void (*func)(Rx64IoSampleResponse&, uintptr_t), uintptr_t data = 0) { _onRx64IoSampleResponse.set(func, data); }
	void onModemStatusResponse(void (*func)(ModemStatusResponse&, uintptr_t), uintptr_t data = 0) { _onModemStatusResponse.set(func, data); }
	void onAtCommandResponse(void (*func)(AtCommandResponse&, uintptr_t), uintptr_t data = 0) { _onAtCommandResponse.set(func, data); }
	void onRemoteAtCommandResponse(void (*func)(RemoteAtCommandResponse&, uintptr_t), uintptr_t data = 0) { _onRemoteAtCommandResponse.set(func, data); }

	/**
	 * Regularly call this method, which ensures that the serial
	 * buffer is processed and the appropriate callbacks are called.
	 */
	void loop();

	/**
	 * Wait for a API response of the given type, optionally
	 * filtered by the given match function.
	 *
	 * If a match function is given it is called for every response
	 * of the right type received, passing the response and the data
	 * parameter passed to this method. If the function returns true
	 * (or if no function was passed), waiting stops and this method
	 * returns 0. If the function returns false, waiting
	 * continues. After the given timeout passes, this method
	 * returns XBEE_WAIT_TIMEOUT.
	 *
	 * If a valid frameId is passed (e.g. 0-255 inclusive) and a
	 * status API response frame is received while waiting, that has
	 * a *non-zero* status, waiting stops and that status is
	 * received. This is intended for when a TX packet was sent and
	 * you are waiting for an RX reply, which will most likely never
	 * arrive when TX failed. However, since the status reply is not
	 * guaranteed to arrive before the RX reply (a remote module can
	 * send a reply before the ACK), first calling waitForStatus()
	 * and then waitFor() can sometimes miss the reply RX packet.
	 *
	 * Note that when the intended response is received *before* the
	 * status reply, the latter will not be processed by this
	 * method and will be subsequently processed by e.g. loop()
	 * normally.
	 *
	 * While waiting, any other responses received are passed to the
	 * relevant callbacks, just as if calling loop() continuously
	 * (except for the response sought, that one is only passed to
	 * the OnResponse handler and no others).
	 *
	 * After this method returns, the response itself can still be
	 * retrieved using getResponse() as normal.
	 */
	template <typename Response>
	uint8_t waitFor(Response& response, uint16_t timeout, bool (*func)(Response&, uintptr_t) = NULL, uintptr_t data = 0, int16_t frameId = -1) {
		return waitForInternal(Response::API_ID, &response, timeout, (void*)func, data, frameId);
	}

	/**
	 * Sends a XBeeRequest (TX packet) out the serial port, and wait
	 * for a status response API frame (up until the given timeout).
	 * Essentially this just calls send() and waitForStatus().
	 * See waitForStatus for the meaning of the return value and
	 * more details.
	 */
	uint8_t sendAndWait(XBeeRequest &request, uint16_t timeout) {
		send(request);
		return waitForStatus(request.getFrameId(), timeout);
	}

	/**
	 * Wait for a status API response with the given frameId and
	 * return the status from the packet (for ZB_TX_STATUS_RESPONSE,
	 * this returns just the delivery status, not the routing
	 * status). If the timeout is reached before reading the
	 * response, XBEE_WAIT_TIMEOUT is returned instead.
	 *
	 * While waiting, any other responses received are passed to the
	 * relevant callbacks, just as if calling loop() continuously
	 * (except for the status response sought, that one is only
	 * passed to the OnResponse handler and no others).
	 *
	 * After this method returns, the response itself can still be
	 * retrieved using getResponse() as normal.
	 */
	uint8_t waitForStatus(uint8_t frameId, uint16_t timeout);
private:
	/**
	 * Internal version of waitFor that does not need to be
	 * templated (to prevent duplication the implementation for
	 * every response type you might want to wait for). Instead of
	 * using templates, this accepts the apiId to wait for and will
	 * cast the given response object and the argument to the given
	 * function to the corresponding type. This means that the
	 * void* given must match the api id!
	 */
	uint8_t waitForInternal(uint8_t apiId, void *response, uint16_t timeout, void *func, uintptr_t data, int16_t frameId);

	/**
	 * Helper that checks if the current response is a status
	 * response with the given frame id. If so, returns the status
	 * byte from the response, otherwise returns 0xff.
	 */
	uint8_t matchStatus(uint8_t frameId);

	/**
	 * Top half of a typical loop(). Calls readPacket(), calls
	 * onPacketError on error, calls onResponse when a response is
	 * available. Returns in the true in the latter case, after
	 * which a caller should typically call loopBottom().
	 */
	bool loopTop();

	/**
	 * Bottom half of a typical loop. Call only when a valid
	 * response was read, will call all response-specific callbacks.
	 */
	void loopBottom();

	template <typename Arg> struct Callback {
		void (*func)(Arg, uintptr_t);
		uintptr_t data;
		void set(void (*func)(Arg, uintptr_t), uintptr_t data) {
			this->func = func;
			this->data = data;
		}
		bool call(Arg arg) {
			if (this->func) {
				this->func(arg, this->data);
				return true;
			}
			return false;
		}
	};

	Callback<uint8_t> _onPacketError;
	Callback<XBeeResponse&> _onResponse;
	Callback<XBeeResponse&> _onOtherResponse;
	Callback<ZBTxStatusResponse&> _onZBTxStatusResponse;
	Callback<ZBRxResponse&> _onZBRxResponse;
	Callback<ZBExplicitRxResponse&> _onZBExplicitRxResponse;
	Callback<ZBRxIoSampleResponse&> _onZBRxIoSampleResponse;
	Callback<TxStatusResponse&> _onTxStatusResponse;
	Callback<Rx16Response&> _onRx16Response;
	Callback<Rx64Response&> _onRx64Response;
	Callback<Rx16IoSampleResponse&> _onRx16IoSampleResponse;
	Callback<Rx64IoSampleResponse&> _onRx64IoSampleResponse;
	Callback<ModemStatusResponse&> _onModemStatusResponse;
	Callback<AtCommandResponse&> _onAtCommandResponse;
	Callback<RemoteAtCommandResponse&> _onRemoteAtCommandResponse;
};

/**
 * All TX packets that support payloads extend this class
 */
class PayloadRequest : public XBeeRequest {
public:
	PayloadRequest(uint8_t apiId, uint8_t frameId, uint8_t *payload, uint8_t payloadLength);
	/**
	 * Returns the payload of the packet, if not null
	 */
	uint8_t* getPayload();
	/**
	 * Sets the payload array
	 */
	void setPayload(uint8_t* payloadPtr);

	/*
	 * Set the payload and its length in one call.
	 */
	void setPayload(uint8_t* payloadPtr, uint8_t payloadLength) {
		setPayload(payloadPtr);
		setPayloadLength(payloadLength);
	}

	/**
	 * Returns the length of the payload array, as specified by the user.
	 */
	uint8_t getPayloadLength();
	/**
	 * Sets the length of the payload to include in the request.  For example if the payload array
	 * is 50 bytes and you only want the first 10 to be included in the packet, set the length to 10.
	 * Length must be <= to the array length.
	 */
	void setPayloadLength(uint8_t payloadLength);
private:
	uint8_t* _payloadPtr;
	uint8_t _payloadLength;
};

#ifdef SERIES_1

/**
 * Represents a Series 1 TX packet that corresponds to Api Id: TX_16_REQUEST
 * <p/>
 * Be careful not to send a data array larger than the max packet size of your radio.
 * This class does not perform any validation of packet size and there will be no indication
 * if the packet is too large, other than you will not get a TX Status response.
 * The datasheet says 100 bytes is the maximum, although that could change in future firmware.
 */
class Tx16Request : public PayloadRequest {
public:
	Tx16Request(uint16_t addr16, uint8_t option, uint8_t *payload, uint8_t payloadLength, uint8_t frameId);
	/**
	 * Creates a Unicast Tx16Request with the ACK option and DEFAULT_FRAME_ID
	 */
	Tx16Request(uint16_t addr16, uint8_t *payload, uint8_t payloadLength);
	/**
	 * Creates a default instance of this class.  At a minimum you must specify
	 * a payload, payload length and a destination address before sending this request.
	 */
	Tx16Request();
	uint16_t getAddress16();
	void setAddress16(uint16_t addr16);
	uint8_t getOption();
	void setOption(uint8_t option);
	uint8_t getFrameData(uint8_t pos);
	uint8_t getFrameDataLength();
protected:
private:
	uint16_t _addr16;
	uint8_t _option;
};

/**
 * Represents a Series 1 TX packet that corresponds to Api Id: TX_64_REQUEST
 *
 * Be careful not to send a data array larger than the max packet size of your radio.
 * This class does not perform any validation of packet size and there will be no indication
 * if the packet is too large, other than you will not get a TX Status response.
 * The datasheet says 100 bytes is the maximum, although that could change in future firmware.
 */
class Tx64Request : public PayloadRequest {
public:
	Tx64Request(XBeeAddress64 &addr64, uint8_t option, uint8_t *payload, uint8_t payloadLength, uint8_t frameId);
	/**
	 * Creates a unicast Tx64Request with the ACK option and DEFAULT_FRAME_ID
	 */
	Tx64Request(XBeeAddress64 &addr64, uint8_t *payload, uint8_t payloadLength);
	/**
	 * Creates a default instance of this class.  At a minimum you must specify
	 * a payload, payload length and a destination address before sending this request.
	 */
	Tx64Request();
	XBeeAddress64& getAddress64();
	void setAddress64(XBeeAddress64& addr64);
	// TODO move option to superclass
	uint8_t getOption();
	void setOption(uint8_t option);
	uint8_t getFrameData(uint8_t pos);
	uint8_t getFrameDataLength();
private:
	XBeeAddress64 _addr64;
	uint8_t _option;
};

#endif


#ifdef SERIES_2

/**
 * Represents a Series 2 TX packet that corresponds to Api Id: ZB_TX_REQUEST
 *
 * Be careful not to send a data array larger than the max packet size of your radio.
 * This class does not perform any validation of packet size and there will be no indication
 * if the packet is too large, other than you will not get a TX Status response.
 * The datasheet says 72 bytes is the maximum for ZNet firmware and ZB Pro firmware provides
 * the ATNP command to get the max supported payload size.  This command is useful since the
 * maximum payload size varies according to certain settings, such as encryption.
 * ZB Pro firmware provides a PAYLOAD_TOO_LARGE that is returned if payload size
 * exceeds the maximum.
 */
class ZBTxRequest : public PayloadRequest {
public:
	/**
	 * Creates a unicast ZBTxRequest with the ACK option and DEFAULT_FRAME_ID
	 */
	ZBTxRequest(const XBeeAddress64 &addr64, uint8_t *payload, uint8_t payloadLength);
	ZBTxRequest(const XBeeAddress64 &addr64, uint16_t addr16, uint8_t broadcastRadius, uint8_t option, uint8_t *payload, uint8_t payloadLength, uint8_t frameId);
	/**
	 * Creates a default instance of this class.  At a minimum you must specify
	 * a payload, payload length and a 64-bit destination address before sending
	 * this request.
	 */
	ZBTxRequest();
	XBeeAddress64& getAddress64();
	uint16_t getAddress16();
	uint8_t getBroadcastRadius();
	uint8_t getOption();
	void setAddress64(const XBeeAddress64& addr64);
	void setAddress16(uint16_t addr16);
	void setBroadcastRadius(uint8_t broadcastRadius);
	void setOption(uint8_t option);
protected:
	// declare virtual functions
	uint8_t getFrameData(uint8_t pos);
	uint8_t getFrameDataLength();
	XBeeAddress64 _addr64;
	uint16_t _addr16;
	uint8_t _broadcastRadius;
	uint8_t _option;
};

/**
 * Represents a Series 2 TX packet that corresponds to Api Id: ZB_EXPLICIT_TX_REQUEST
 *
 * See the warning about maximum packet size for ZBTxRequest above,
 * which probably also applies here as well.
 *
 * Note that to distinguish reply packets from non-XBee devices, set
 * AO=1 to enable reception of ZBExplicitRxResponse packets.
 */
class ZBExplicitTxRequest : public ZBTxRequest {
public:
	/**
	 * Creates a unicast ZBExplicitTxRequest with the ACK option and
	 * DEFAULT_FRAME_ID.
	 *
	 * It uses the Maxstream profile (0xc105), both endpoints 232
	 * and cluster 0x0011, resulting in the same packet as sent by a
	 * normal ZBTxRequest.
	 */
	ZBExplicitTxRequest(XBeeAddress64 &addr64, uint8_t *payload, uint8_t payloadLength);
	/**
	 * Create a ZBExplicitTxRequest, specifying all fields.
	 */
	ZBExplicitTxRequest(XBeeAddress64 &addr64, uint16_t addr16, uint8_t broadcastRadius, uint8_t option, uint8_t *payload, uint8_t payloadLength, uint8_t frameId, uint8_t srcEndpoint, uint8_t dstEndpoint, uint16_t clusterId, uint16_t profileId);
	/**
	 * Creates a default instance of this class.  At a minimum you
	 * must specify a payload, payload length and a destination
	 * address before sending this request.
	 *
	 * Furthermore, it uses the Maxstream profile (0xc105), both
	 * endpoints 232 and cluster 0x0011, resulting in the same
	 * packet as sent by a normal ZBExplicitTxRequest.
	 */
	ZBExplicitTxRequest();
	uint8_t getSrcEndpoint();
	uint8_t getDstEndpoint();
	uint16_t getClusterId();
	uint16_t getProfileId();
	void setSrcEndpoint(uint8_t endpoint);
	void setDstEndpoint(uint8_t endpoint);
	void setClusterId(uint16_t clusterId);
	void setProfileId(uint16_t profileId);
protected:
	// declare virtual functions
	uint8_t getFrameData(uint8_t pos);
	uint8_t getFrameDataLength();
private:
	uint8_t _srcEndpoint;
	uint8_t _dstEndpoint;
	uint16_t _profileId;
	uint16_t _clusterId;
};

#endif

/**
 * Represents an AT Command TX packet
 * The command is used to configure the serially connected XBee radio
 */
class AtCommandRequest : public XBeeRequest {
public:
	AtCommandRequest();
	AtCommandRequest(uint8_t *command);
	AtCommandRequest(uint8_t *command, uint8_t *commandValue, uint8_t commandValueLength);
	uint8_t getFrameData(uint8_t pos);
	uint8_t getFrameDataLength();
	uint8_t* getCommand();
	void setCommand(uint8_t* command);
	uint8_t* getCommandValue();
	void setCommandValue(uint8_t* command);
	uint8_t getCommandValueLength();
	void setCommandValueLength(uint8_t length);
	/**
	 * Clears the optional commandValue and commandValueLength so that a query may be sent
	 */
	void clearCommandValue();
	//void reset();
private:
	uint8_t *_command;
	uint8_t *_commandValue;
	uint8_t _commandValueLength;
};

/**
 * Represents an Remote AT Command TX packet
 * The command is used to configure a remote XBee radio
 */
class RemoteAtCommandRequest : public AtCommandRequest {
public:
	RemoteAtCommandRequest();
	/**
	 * Creates a RemoteAtCommandRequest with 16-bit address to set a command.
	 * 64-bit address defaults to broadcast and applyChanges is true.
	 */
	RemoteAtCommandRequest(uint16_t remoteAddress16, uint8_t *command, uint8_t *commandValue, uint8_t commandValueLength);
	/**
	 * Creates a RemoteAtCommandRequest with 16-bit address to query a command.
	 * 64-bit address defaults to broadcast and applyChanges is true.
	 */
	RemoteAtCommandRequest(uint16_t remoteAddress16, uint8_t *command);
	/**
	 * Creates a RemoteAtCommandRequest with 64-bit address to set a command.
	 * 16-bit address defaults to broadcast and applyChanges is true.
	 */
	RemoteAtCommandRequest(XBeeAddress64 &remoteAddress64, uint8_t *command, uint8_t *commandValue, uint8_t commandValueLength);
	/**
	 * Creates a RemoteAtCommandRequest with 16-bit address to query a command.
	 * 16-bit address defaults to broadcast and applyChanges is true.
	 */
	RemoteAtCommandRequest(XBeeAddress64 &remoteAddress64, uint8_t *command);
	uint16_t getRemoteAddress16();
	void setRemoteAddress16(uint16_t remoteAddress16);
	XBeeAddress64& getRemoteAddress64();
	void setRemoteAddress64(XBeeAddress64 &remoteAddress64);
	bool getApplyChanges();
	void setApplyChanges(bool applyChanges);
	uint8_t getFrameData(uint8_t pos);
	uint8_t getFrameDataLength();
	static XBeeAddress64 broadcastAddress64;
//	static uint16_t broadcast16Address;
private:
	XBeeAddress64 _remoteAddress64;
	uint16_t _remoteAddress16;
	bool _applyChanges;
};


XBeeResponse::XBeeResponse() {

}

uint8_t XBeeResponse::getApiId() {
	return _apiId;
}

void XBeeResponse::setApiId(uint8_t apiId) {
	_apiId = apiId;
}

uint8_t XBeeResponse::getMsbLength() {
	return _msbLength;
}

void XBeeResponse::setMsbLength(uint8_t msbLength) {
	_msbLength = msbLength;
}

uint8_t XBeeResponse::getLsbLength() {
	return _lsbLength;
}

void XBeeResponse::setLsbLength(uint8_t lsbLength) {
	_lsbLength = lsbLength;
}

uint8_t XBeeResponse::getChecksum() {
	return _checksum;
}

void XBeeResponse::setChecksum(uint8_t checksum) {
	_checksum = checksum;
}

uint8_t XBeeResponse::getFrameDataLength() {
	return _frameLength;
}

void XBeeResponse::setFrameLength(uint8_t frameLength) {
	_frameLength = frameLength;
}

bool XBeeResponse::isAvailable() {
	return _complete;
}

void XBeeResponse::setAvailable(bool complete) {
	_complete = complete;
}

bool XBeeResponse::isError() {
	return _errorCode > 0;
}

uint8_t XBeeResponse::getErrorCode() {
	return _errorCode;
}

void XBeeResponse::setErrorCode(uint8_t errorCode) {
	_errorCode = errorCode;
}

// copy common fields from xbee response to target response
void XBeeResponse::setCommon(XBeeResponse &target) {
	target.setApiId(getApiId());
	target.setAvailable(isAvailable());
	target.setChecksum(getChecksum());
	target.setErrorCode(getErrorCode());
	target.setFrameLength(getFrameDataLength());
	target.setMsbLength(getMsbLength());
	target.setLsbLength(getLsbLength());
}

#ifdef SERIES_2

ZBTxStatusResponse::ZBTxStatusResponse() : FrameIdResponse() {

}

uint16_t ZBTxStatusResponse::getRemoteAddress() {
	return  (getFrameData()[1] << 8) + getFrameData()[2];
}

uint8_t ZBTxStatusResponse::getTxRetryCount() {
	return getFrameData()[3];
}

uint8_t ZBTxStatusResponse::getDeliveryStatus() {
	return getFrameData()[4];
}

uint8_t ZBTxStatusResponse::getDiscoveryStatus() {
	return getFrameData()[5];
}

bool ZBTxStatusResponse::isSuccess() {
	return getDeliveryStatus() == SUCCESS;
}

void XBeeResponse::getZBTxStatusResponse(XBeeResponse &zbXBeeResponse) {

	// way off?
	ZBTxStatusResponse* zb = static_cast<ZBTxStatusResponse*>(&zbXBeeResponse);
	// pass pointer array to subclass
	zb->setFrameData(getFrameData());
	setCommon(zbXBeeResponse);
}

ZBRxResponse::ZBRxResponse(): RxDataResponse() {
	_remoteAddress64 = XBeeAddress64();
}

uint16_t ZBRxResponse::getRemoteAddress16() {
	return 	(getFrameData()[8] << 8) + getFrameData()[9];
}

uint8_t ZBRxResponse::getOption() {
	return getFrameData()[10];
}

// markers to read data from packet array.  this is the index, so the 12th item in the array
uint8_t ZBRxResponse::getDataOffset() {
	return 11;
}

uint8_t ZBRxResponse::getDataLength() {
	return getPacketLength() - getDataOffset() - 1;
}

XBeeAddress64& ZBRxResponse::getRemoteAddress64() {
	return _remoteAddress64;
}

void XBeeResponse::getZBRxResponse(XBeeResponse &rxResponse) {

	ZBRxResponse* zb = static_cast<ZBRxResponse*>(&rxResponse);

	//TODO verify response api id matches this api for this response

	// pass pointer array to subclass
	zb->setFrameData(getFrameData());
	setCommon(rxResponse);

	zb->getRemoteAddress64().setMsb((uint32_t(getFrameData()[0]) << 24) + (uint32_t(getFrameData()[1]) << 16) + (uint16_t(getFrameData()[2]) << 8) + getFrameData()[3]);
	zb->getRemoteAddress64().setLsb((uint32_t(getFrameData()[4]) << 24) + (uint32_t(getFrameData()[5]) << 16) + (uint16_t(getFrameData()[6]) << 8) + (getFrameData()[7]));
}

ZBExplicitRxResponse::ZBExplicitRxResponse(): ZBRxResponse() {
}

uint8_t ZBExplicitRxResponse::getSrcEndpoint() {
	return getFrameData()[10];
}

uint8_t ZBExplicitRxResponse::getDstEndpoint() {
	return getFrameData()[11];
}

uint16_t ZBExplicitRxResponse::getClusterId() {
	return (uint16_t)(getFrameData()[12]) << 8 | getFrameData()[13];
}

uint16_t ZBExplicitRxResponse::getProfileId() {
	return (uint16_t)(getFrameData()[14]) << 8 | getFrameData()[15];
}

uint8_t ZBExplicitRxResponse::getOption() {
	return getFrameData()[16];
}

// markers to read data from packet array.
uint8_t ZBExplicitRxResponse::getDataOffset() {
	return 17;
}

uint8_t ZBExplicitRxResponse::getDataLength() {
	return getPacketLength() - getDataOffset() - 1;
}

void XBeeResponse::getZBExplicitRxResponse(XBeeResponse &rxResponse) {
	// Nothing to add to that
	getZBRxResponse(rxResponse);
}


ZBRxIoSampleResponse::ZBRxIoSampleResponse() : ZBRxResponse() {

}

// 64 + 16 addresses, sample size, option = 12 (index 11), so this starts at 12
uint8_t ZBRxIoSampleResponse::getDigitalMaskMsb() {
	return getFrameData()[12] & 0x1c;
}

uint8_t ZBRxIoSampleResponse::getDigitalMaskLsb() {
	return getFrameData()[13];
}

uint8_t ZBRxIoSampleResponse::getAnalogMask() {
	return getFrameData()[14] & 0x8f;
}

bool ZBRxIoSampleResponse::containsAnalog() {
	return getAnalogMask() > 0;
}

bool ZBRxIoSampleResponse::containsDigital() {
	return getDigitalMaskMsb() > 0 || getDigitalMaskLsb() > 0;
}

bool ZBRxIoSampleResponse::isAnalogEnabled(uint8_t pin) {
	return ((getAnalogMask() >> pin) & 1) == 1;
}

bool ZBRxIoSampleResponse::isDigitalEnabled(uint8_t pin) {
	if (pin <= 7) {
		// added extra parens to calm avr compiler
		return ((getDigitalMaskLsb() >> pin) & 1) == 1;
	} else {
		return ((getDigitalMaskMsb() >> (pin - 8)) & 1) == 1;
	}
}

uint16_t ZBRxIoSampleResponse::getAnalog(uint8_t pin) {
	// analog starts 13 bytes after sample size, if no dio enabled
	uint8_t start = 15;

	if (containsDigital()) {
		// make room for digital i/o
		start+=2;
	}

	// start depends on how many pins before this pin are enabled
	for (int i = 0; i < pin; i++) {
		if (isAnalogEnabled(i)) {
			start+=2;
		}
	}

	return (uint16_t)((getFrameData()[start] << 8) + getFrameData()[start + 1]);
}

bool ZBRxIoSampleResponse::isDigitalOn(uint8_t pin) {
	if (pin <= 7) {
		// D0-7
		// DIO LSB is index 5
		return ((getFrameData()[16] >> pin) & 1) == 1;
	} else {
		// D10-12
		// DIO MSB is index 4
		return ((getFrameData()[15] >> (pin - 8)) & 1) == 1;
	}
}

void XBeeResponse::getZBRxIoSampleResponse(XBeeResponse &response) {
	ZBRxIoSampleResponse* zb = static_cast<ZBRxIoSampleResponse*>(&response);


	// pass pointer array to subclass
	zb->setFrameData(getFrameData());
	setCommon(response);

	zb->getRemoteAddress64().setMsb((uint32_t(getFrameData()[0]) << 24) + (uint32_t(getFrameData()[1]) << 16) + (uint16_t(getFrameData()[2]) << 8) + getFrameData()[3]);
	zb->getRemoteAddress64().setLsb((uint32_t(getFrameData()[4]) << 24) + (uint32_t(getFrameData()[5]) << 16) + (uint16_t(getFrameData()[6]) << 8) + (getFrameData()[7]));
}

#endif

#ifdef SERIES_1

RxResponse::RxResponse() : RxDataResponse() {

}

uint16_t Rx16Response::getRemoteAddress16() {
	return (getFrameData()[0] << 8) + getFrameData()[1];
}

XBeeAddress64& Rx64Response::getRemoteAddress64() {
	return _remoteAddress;
}

Rx64Response::Rx64Response() : RxResponse() {
	_remoteAddress = XBeeAddress64();
}

Rx16Response::Rx16Response() : RxResponse() {

}

RxIoSampleBaseResponse::RxIoSampleBaseResponse() : RxResponse() {

}

uint8_t RxIoSampleBaseResponse::getSampleOffset() {
	// sample starts 2 bytes after rssi
	return getRssiOffset() + 2;
}


uint8_t RxIoSampleBaseResponse::getSampleSize() {
	return getFrameData()[getSampleOffset()];
}

bool RxIoSampleBaseResponse::containsAnalog() {
	return (getFrameData()[getSampleOffset() + 1] & 0x7e) > 0;
}

bool RxIoSampleBaseResponse::containsDigital() {
	return (getFrameData()[getSampleOffset() + 1] & 0x1) > 0 || getFrameData()[getSampleOffset() + 2] > 0;
}

//uint16_t RxIoSampleBaseResponse::getAnalog0(uint8_t sample) {
//	return getAnalog(0, sample);
//}

bool RxIoSampleBaseResponse::isAnalogEnabled(uint8_t pin) {
	return (((getFrameData()[getSampleOffset() + 1] >> (pin + 1)) & 1) == 1);
}

bool RxIoSampleBaseResponse::isDigitalEnabled(uint8_t pin) {
	if (pin < 8) {
		return ((getFrameData()[getSampleOffset() + 2] >> pin) & 1) == 1;
	} else {
		return (getFrameData()[getSampleOffset() + 1] & 1) == 1;
	}
}

//	// verified (from XBee-API)
//	private int getSampleWidth() {
//		int width = 0;
//
//		// width of sample depends on how many I/O pins are enabled. add one for each analog that's enabled
//		for (int i = 0; i <= 5; i++) {
//			if (isAnalogEnabled(i)) {
//				// each analog is two bytes
//				width+=2;
//			}
//		}
//
//		if (this.containsDigital()) {
//			// digital enabled takes two bytes, no matter how many pins enabled
//			width+= 2;
//		}
//
//		return width;
//	}
//
//	private int getStartIndex() {
//
//		int startIndex;
//
//		if (this.getSourceAddress() instanceof XBeeAddress16) {
//			// 16 bit
//			startIndex = 7;
//		} else {
//			// 64 bit
//			startIndex = 13;
//		}
//
//		return startIndex;
//	}
//
//	public int getDigitalMsb(int sample) {
//		// msb digital always starts 3 bytes after sample size
//		return this.getProcessedPacketBytes()[this.getStartIndex() + 3 + this.getSampleWidth() * sample];
//	}
//
//	public int getDigitalLsb(int sample) {
//		return this.getProcessedPacketBytes()[this.getStartIndex() + 3 + this.getSampleWidth() * sample + 1];
//	}
//
//	public Boolean isDigitalOn(int pin, int sample) {
//
//		if (sample < 0 || sample >= this.getSampleSize()) {
//			throw new IllegalArgumentException("invalid sample size: " + sample);
//		}
//
//		if (!this.containsDigital()) {
//			throw new RuntimeException("Digital is not enabled");
//		}
//
//		if (pin >= 0 && pin < 8) {
//			return ((this.getDigitalLsb(sample) >> pin) & 1) == 1;
//		} else if (pin == 8) {
//			// uses msb dio line
//			return (this.getDigitalMsb(sample) & 1) == 1;
//		} else {
//			throw new IllegalArgumentException("Invalid pin: " + pin);
//		}
//	}
//
//	public Integer getAnalog(int pin, int sample) {
//
//		if (sample < 0 || sample >= this.getSampleSize()) {
//			throw new IllegalArgumentException("invalid sample size: " + sample);
//		}
//
//		// analog starts 3 bytes after start of sample, if no dio enabled
//		int startIndex = this.getStartIndex() + 3;
//
//		if (this.containsDigital()) {
//			// make room for digital i/o sample (2 bytes per sample)
//			startIndex+= 2;
//		}
//
//		startIndex+= this.getSampleWidth() * sample;
//
//		// start depends on how many pins before this pin are enabled
//		// this will throw illegalargumentexception if invalid pin
//		for (int i = 0; i < pin; i++) {
//			if (isAnalogEnabled(i)) {
//				startIndex+=2;
//			}
//		}
//
//		return (this.getProcessedPacketBytes()[startIndex] << 8) + this.getProcessedPacketBytes()[startIndex + 1];
//	}

uint8_t RxIoSampleBaseResponse::getSampleStart(uint8_t sample) {
	uint8_t spacing = 0;

	if (containsDigital()) {
		// make room for digital i/o sample (2 bytes per sample)
		spacing += 2;
	}

	// spacing between samples depends on how many are enabled. add
	// 2 bytes for each analog that's enabled
	for (int i = 0; i <= 5; i++) {
		if (isAnalogEnabled(i)) {
			// each analog is two bytes
			spacing+=2;
		}
	}

	// Skip 3-byte header and "sample" full samples
	return getSampleOffset() + 3 + sample * spacing;
}

uint16_t RxIoSampleBaseResponse::getAnalog(uint8_t pin, uint8_t sample) {
	uint8_t start = getSampleStart(sample);

	if (containsDigital()) {
		// Skip digital sample info
		start += 2;
	}

	// Skip any analog samples before this pin
	for (int i = 0; i < pin; i++) {
		if (isAnalogEnabled(i)) {
			start+=2;
		}
	}

	return (uint16_t)((getFrameData()[start] << 8) + getFrameData()[start + 1]);
}

bool RxIoSampleBaseResponse::isDigitalOn(uint8_t pin, uint8_t sample) {
	if (pin < 8) {
		return ((getFrameData()[getSampleStart(sample) + 1] >> pin) & 1) == 1;
	} else {
		return (getFrameData()[getSampleStart(sample)] & 1) == 1;
	}
}


//bool RxIoSampleBaseResponse::isDigital0On(uint8_t sample) {
//	return isDigitalOn(0, sample);
//}

Rx16IoSampleResponse::Rx16IoSampleResponse() : RxIoSampleBaseResponse() {

}

uint16_t Rx16IoSampleResponse::getRemoteAddress16() {
	return (uint16_t)((getFrameData()[0] << 8) + getFrameData()[1]);
}

uint8_t Rx16IoSampleResponse::getRssiOffset() {
	return 2;
}

void XBeeResponse::getRx16IoSampleResponse(XBeeResponse &response) {
	Rx16IoSampleResponse* rx = static_cast<Rx16IoSampleResponse*>(&response);

	rx->setFrameData(getFrameData());
	setCommon(response);
}


Rx64IoSampleResponse::Rx64IoSampleResponse() : RxIoSampleBaseResponse() {
	_remoteAddress = XBeeAddress64();
}

XBeeAddress64& Rx64IoSampleResponse::getRemoteAddress64() {
	return _remoteAddress;
}

uint8_t Rx64IoSampleResponse::getRssiOffset() {
	return 8;
}

void XBeeResponse::getRx64IoSampleResponse(XBeeResponse &response) {
	Rx64IoSampleResponse* rx = static_cast<Rx64IoSampleResponse*>(&response);

	rx->setFrameData(getFrameData());
	setCommon(response);

	rx->getRemoteAddress64().setMsb((uint32_t(getFrameData()[0]) << 24) + (uint32_t(getFrameData()[1]) << 16) + (uint16_t(getFrameData()[2]) << 8) + getFrameData()[3]);
	rx->getRemoteAddress64().setLsb((uint32_t(getFrameData()[4]) << 24) + (uint32_t(getFrameData()[5]) << 16) + (uint16_t(getFrameData()[6]) << 8) + getFrameData()[7]);
}

TxStatusResponse::TxStatusResponse() : FrameIdResponse() {

}

uint8_t TxStatusResponse::getStatus() {
	return getFrameData()[1];
}

bool TxStatusResponse::isSuccess() {
	return getStatus() == SUCCESS;
}

void XBeeResponse::getTxStatusResponse(XBeeResponse &txResponse) {

	TxStatusResponse* txStatus = static_cast<TxStatusResponse*>(&txResponse);

	// pass pointer array to subclass
	txStatus->setFrameData(getFrameData());
	setCommon(txResponse);
}

uint8_t RxResponse::getRssi() {
	return getFrameData()[getRssiOffset()];
}

uint8_t RxResponse::getOption() {
	return getFrameData()[getRssiOffset() + 1];
}

bool RxResponse::isAddressBroadcast() {
	return (getOption() & 2) == 2;
}

bool RxResponse::isPanBroadcast() {
	return (getOption() & 4) == 4;
}

uint8_t RxResponse::getDataLength() {
	return getPacketLength() - getDataOffset() - 1;
}

uint8_t RxResponse::getDataOffset() {
	return getRssiOffset() + 2;
}

uint8_t Rx16Response::getRssiOffset() {
	return RX_16_RSSI_OFFSET;
}

void XBeeResponse::getRx16Response(XBeeResponse &rx16Response) {

	Rx16Response* rx16 = static_cast<Rx16Response*>(&rx16Response);

	// pass pointer array to subclass
	rx16->setFrameData(getFrameData());
	setCommon(rx16Response);
}

uint8_t Rx64Response::getRssiOffset() {
	return RX_64_RSSI_OFFSET;
}

void XBeeResponse::getRx64Response(XBeeResponse &rx64Response) {

	Rx64Response* rx64 = static_cast<Rx64Response*>(&rx64Response);

	// pass pointer array to subclass
	rx64->setFrameData(getFrameData());
	setCommon(rx64Response);

	rx64->getRemoteAddress64().setMsb((uint32_t(getFrameData()[0]) << 24) + (uint32_t(getFrameData()[1]) << 16) + (uint16_t(getFrameData()[2]) << 8) + getFrameData()[3]);
	rx64->getRemoteAddress64().setLsb((uint32_t(getFrameData()[4]) << 24) + (uint32_t(getFrameData()[5]) << 16) + (uint16_t(getFrameData()[6]) << 8) + getFrameData()[7]);
}

#endif

RemoteAtCommandResponse::RemoteAtCommandResponse() : AtCommandResponse() {

}

uint8_t* RemoteAtCommandResponse::getCommand() {
	return getFrameData() + 11;
}

uint8_t RemoteAtCommandResponse::getStatus() {
	return getFrameData()[13];
}

bool RemoteAtCommandResponse::isOk() {
	// weird c++ behavior.  w/o this method, it calls AtCommandResponse::isOk(), which calls the AtCommandResponse::getStatus, not this.getStatus!!!
	return getStatus() == AT_OK;
}

uint8_t RemoteAtCommandResponse::getValueLength() {
	return getFrameDataLength() - 14;
}

uint8_t* RemoteAtCommandResponse::getValue() {
	if (getValueLength() > 0) {
		// value is only included for query commands.  set commands does not return a value
		return getFrameData() + 14;
	}

	return NULL;
}

uint16_t RemoteAtCommandResponse::getRemoteAddress16() {
	return uint16_t((getFrameData()[9] << 8) + getFrameData()[10]);
}

XBeeAddress64& RemoteAtCommandResponse::getRemoteAddress64() {
	return _remoteAddress64;
}

void XBeeResponse::getRemoteAtCommandResponse(XBeeResponse &response) {

	// TODO no real need to cast.  change arg to match expected class
	RemoteAtCommandResponse* at = static_cast<RemoteAtCommandResponse*>(&response);

	// pass pointer array to subclass
	at->setFrameData(getFrameData());
	setCommon(response);

	at->getRemoteAddress64().setMsb((uint32_t(getFrameData()[1]) << 24) + (uint32_t(getFrameData()[2]) << 16) + (uint16_t(getFrameData()[3]) << 8) + getFrameData()[4]);
	at->getRemoteAddress64().setLsb((uint32_t(getFrameData()[5]) << 24) + (uint32_t(getFrameData()[6]) << 16) + (uint16_t(getFrameData()[7]) << 8) + (getFrameData()[8]));

}

RxDataResponse::RxDataResponse() : XBeeResponse() {

}

uint8_t RxDataResponse::getData(int index) {
	return getFrameData()[getDataOffset() + index];
}

uint8_t* RxDataResponse::getData() {
	return getFrameData() + getDataOffset();
}

FrameIdResponse::FrameIdResponse() {

}

uint8_t FrameIdResponse::getFrameId() {
	return getFrameData()[0];
}


ModemStatusResponse::ModemStatusResponse() {

}

uint8_t ModemStatusResponse::getStatus() {
	return getFrameData()[0];
}

void XBeeResponse::getModemStatusResponse(XBeeResponse &modemStatusResponse) {

	ModemStatusResponse* modem = static_cast<ModemStatusResponse*>(&modemStatusResponse);

	// pass pointer array to subclass
	modem->setFrameData(getFrameData());
	setCommon(modemStatusResponse);

}

AtCommandResponse::AtCommandResponse() {

}

uint8_t* AtCommandResponse::getCommand() {
	return getFrameData() + 1;
}

uint8_t AtCommandResponse::getStatus() {
	return getFrameData()[3];
}

uint8_t AtCommandResponse::getValueLength() {
	return getFrameDataLength() - 4;
}

uint8_t* AtCommandResponse::getValue() {
	if (getValueLength() > 0) {
		// value is only included for query commands.  set commands does not return a value
		return getFrameData() + 4;
	}

	return NULL;
}

bool AtCommandResponse::isOk() {
	return getStatus() == AT_OK;
}

void XBeeResponse::getAtCommandResponse(XBeeResponse &atCommandResponse) {

	AtCommandResponse* at = static_cast<AtCommandResponse*>(&atCommandResponse);

	// pass pointer array to subclass
	at->setFrameData(getFrameData());
	setCommon(atCommandResponse);
}

uint16_t XBeeResponse::getPacketLength() {
	return ((_msbLength << 8) & 0xff) + (_lsbLength & 0xff);
}

uint8_t* XBeeResponse::getFrameData() {
	return _frameDataPtr;
}

void XBeeResponse::setFrameData(uint8_t* frameDataPtr) {
	_frameDataPtr = frameDataPtr;
}

void XBeeResponse::init() {
	_complete = false;
	_errorCode = NO_ERROR;
	_checksum = 0;
}

void XBeeResponse::reset() {
	init();
	_apiId = 0;
	_msbLength = 0;
	_lsbLength = 0;
	_checksum = 0;
	_frameLength = 0;

	_errorCode = NO_ERROR;
}

void XBee::resetResponse() {
	_pos = 0;
	_escape = false;
	_checksumTotal = 0;
	_response.reset();
}

XBee::XBee(): _response(XBeeResponse()) {
        _pos = 0;
        _escape = false;
        _checksumTotal = 0;
        _nextFrameId = 0;

        _response.init();
        _response.setFrameData(_responseFrameData);
		// Contributed by Paul Stoffregen for Teensy support
#if defined(__AVR_ATmega32U4__) || (defined(TEENSYDUINO) && (defined(KINETISK) || defined(KINETISL)))
        _serial = &Serial1;
#else
        _serial = &Serial;
#endif
}

uint8_t XBee::getNextFrameId() {

	_nextFrameId++;

	if (_nextFrameId == 0) {
		// can't send 0 because that disables status response
		_nextFrameId = 1;
	}

	return _nextFrameId;
}

// Support for SoftwareSerial. Contributed by Paul Stoffregen
void XBee::begin(Stream &serial) {
	_serial = &serial;
}

void XBee::setSerial(Stream &serial) {
	_serial = &serial;
}

bool XBee::available() {
	return _serial->available();
}

uint8_t XBee::read() {
	return _serial->read();
}

void XBee::write(uint8_t val) {
	_serial->write(val);
}

XBeeResponse& XBee::getResponse() {
	return _response;
}

// TODO how to convert response to proper subclass?
void XBee::getResponse(XBeeResponse &response) {

	response.setMsbLength(_response.getMsbLength());
	response.setLsbLength(_response.getLsbLength());
	response.setApiId(_response.getApiId());
	response.setFrameLength(_response.getFrameDataLength());

	response.setFrameData(_response.getFrameData());
}

void XBee::readPacketUntilAvailable() {
	while (!(getResponse().isAvailable() || getResponse().isError())) {
		// read some more
		readPacket();
	}
}

bool XBee::readPacket(int timeout) {

	if (timeout < 0) {
		return false;
	}

	unsigned long start = millis();

    while (int((millis() - start)) < timeout) {

     	readPacket();

     	if (getResponse().isAvailable()) {
     		return true;
     	} else if (getResponse().isError()) {
     		return false;
     	}
    }

    // timed out
    return false;
}

void XBee::readPacket() {
	// reset previous response
	if (_response.isAvailable() || _response.isError()) {
		// discard previous packet and start over
		resetResponse();
	}

    while (available()) {

        b = read();

        if (_pos > 0 && b == START_BYTE && ATAP == 2) {
        	// new packet start before previous packeted completed -- discard previous packet and start over
        	_response.setErrorCode(UNEXPECTED_START_BYTE);
        	return;
        }

		if (_pos > 0 && b == ESCAPE) {
			if (available()) {
				b = read();
				b = 0x20 ^ b;
			} else {
				// escape byte.  next byte will be
				_escape = true;
				continue;
			}
		}

		if (_escape == true) {
			b = 0x20 ^ b;
			_escape = false;
		}

		// checksum includes all bytes starting with api id
		if (_pos >= API_ID_INDEX) {
			_checksumTotal+= b;
		}

        switch(_pos) {
			case 0:
		        if (b == START_BYTE) {
		        	_pos++;
		        }

		        break;
			case 1:
				// length msb
				_response.setMsbLength(b);
				_pos++;

				break;
			case 2:
				// length lsb
				_response.setLsbLength(b);
				_pos++;

				break;
			case 3:
				_response.setApiId(b);
				_pos++;

				break;
			default:
				// starts at fifth byte

				if (_pos > MAX_FRAME_DATA_SIZE) {
					// exceed max size.  should never occur
					_response.setErrorCode(PACKET_EXCEEDS_BYTE_ARRAY_LENGTH);
					return;
				}

				// check if we're at the end of the packet
				// packet length does not include start, length, or checksum bytes, so add 3
				if (_pos == (_response.getPacketLength() + 3)) {
					// verify checksum

					if ((_checksumTotal & 0xff) == 0xff) {
						_response.setChecksum(b);
						_response.setAvailable(true);

						_response.setErrorCode(NO_ERROR);
					} else {
						// checksum failed
						_response.setErrorCode(CHECKSUM_FAILURE);
					}

					// minus 4 because we start after start,msb,lsb,api and up to but not including checksum
					// e.g. if frame was one byte, _pos=4 would be the byte, pos=5 is the checksum, where end stop reading
					_response.setFrameLength(_pos - 4);

					// reset state vars
					_pos = 0;

					return;
				} else {
					// add to packet array, starting with the fourth byte of the apiFrame
					_response.getFrameData()[_pos - 4] = b;
					_pos++;
				}
        }
    }
}

// it's peanut butter jelly time!!

XBeeRequest::XBeeRequest(uint8_t apiId, uint8_t frameId) {
	_apiId = apiId;
	_frameId = frameId;
}

void XBeeRequest::setFrameId(uint8_t frameId) {
	_frameId = frameId;
}

uint8_t XBeeRequest::getFrameId() {
	return _frameId;
}

uint8_t XBeeRequest::getApiId() {
	return _apiId;
}

void XBeeRequest::setApiId(uint8_t apiId) {
	_apiId = apiId;
}

//void XBeeRequest::reset() {
//	_frameId = DEFAULT_FRAME_ID;
//}

//uint8_t XBeeRequest::getPayloadOffset() {
//	return _payloadOffset;
//}
//
//uint8_t XBeeRequest::setPayloadOffset(uint8_t payloadOffset) {
//	_payloadOffset = payloadOffset;
//}


PayloadRequest::PayloadRequest(uint8_t apiId, uint8_t frameId, uint8_t *payload, uint8_t payloadLength) : XBeeRequest(apiId, frameId) {
	_payloadPtr = payload;
	_payloadLength = payloadLength;
}

uint8_t* PayloadRequest::getPayload() {
	return _payloadPtr;
}

void PayloadRequest::setPayload(uint8_t* payload) {
	_payloadPtr = payload;
}

uint8_t PayloadRequest::getPayloadLength() {
	return _payloadLength;
}

void PayloadRequest::setPayloadLength(uint8_t payloadLength) {
	_payloadLength = payloadLength;
}

#ifdef SERIES_2

ZBTxRequest::ZBTxRequest() : PayloadRequest(ZB_TX_REQUEST, DEFAULT_FRAME_ID, NULL, 0) {
	_addr16 = ZB_BROADCAST_ADDRESS;
	_broadcastRadius = ZB_BROADCAST_RADIUS_MAX_HOPS;
	_option = ZB_TX_UNICAST;
}

ZBTxRequest::ZBTxRequest(const XBeeAddress64 &addr64, uint16_t addr16, uint8_t broadcastRadius, uint8_t option, uint8_t *data, uint8_t dataLength, uint8_t frameId): PayloadRequest(ZB_TX_REQUEST, frameId, data, dataLength) {
	_addr64 = addr64;
	_addr16 = addr16;
	_broadcastRadius = broadcastRadius;
	_option = option;
}

ZBTxRequest::ZBTxRequest(const XBeeAddress64 &addr64, uint8_t *data, uint8_t dataLength): PayloadRequest(ZB_TX_REQUEST, DEFAULT_FRAME_ID, data, dataLength) {
	_addr64 = addr64;
	_addr16 = ZB_BROADCAST_ADDRESS;
	_broadcastRadius = ZB_BROADCAST_RADIUS_MAX_HOPS;
	_option = ZB_TX_UNICAST;
}

uint8_t ZBTxRequest::getFrameData(uint8_t pos) {
	if (pos == 0) {
		return (_addr64.getMsb() >> 24) & 0xff;
	} else if (pos == 1) {
		return (_addr64.getMsb() >> 16) & 0xff;
	} else if (pos == 2) {
		return (_addr64.getMsb() >> 8) & 0xff;
	} else if (pos == 3) {
		return _addr64.getMsb() & 0xff;
	} else if (pos == 4) {
		return (_addr64.getLsb() >> 24) & 0xff;
	} else if (pos == 5) {
		return  (_addr64.getLsb() >> 16) & 0xff;
	} else if (pos == 6) {
		return (_addr64.getLsb() >> 8) & 0xff;
	} else if (pos == 7) {
		return _addr64.getLsb() & 0xff;
	} else if (pos == 8) {
		return (_addr16 >> 8) & 0xff;
	} else if (pos == 9) {
		return _addr16 & 0xff;
	} else if (pos == 10) {
		return _broadcastRadius;
	} else if (pos == 11) {
		return _option;
	} else {
		return getPayload()[pos - ZB_TX_API_LENGTH];
	}
}

uint8_t ZBTxRequest::getFrameDataLength() {
	return ZB_TX_API_LENGTH + getPayloadLength();
}

XBeeAddress64& ZBTxRequest::getAddress64() {
	return _addr64;
}

uint16_t ZBTxRequest::getAddress16() {
	return _addr16;
}

uint8_t ZBTxRequest::getBroadcastRadius() {
	return _broadcastRadius;
}

uint8_t ZBTxRequest::getOption() {
	return _option;
}

void ZBTxRequest::setAddress64(const XBeeAddress64& addr64) {
	_addr64 = addr64;
}

void ZBTxRequest::setAddress16(uint16_t addr16) {
	_addr16 = addr16;
}

void ZBTxRequest::setBroadcastRadius(uint8_t broadcastRadius) {
	_broadcastRadius = broadcastRadius;
}

void ZBTxRequest::setOption(uint8_t option) {
	_option = option;
}



ZBExplicitTxRequest::ZBExplicitTxRequest() : ZBTxRequest() {
	_srcEndpoint = DEFAULT_ENDPOINT;
	_dstEndpoint = DEFAULT_ENDPOINT;
	_profileId = DEFAULT_PROFILE_ID;
	_clusterId = DEFAULT_CLUSTER_ID;
	setApiId(ZB_EXPLICIT_TX_REQUEST);
}

ZBExplicitTxRequest::ZBExplicitTxRequest(XBeeAddress64 &addr64, uint16_t addr16, uint8_t broadcastRadius, uint8_t option, uint8_t *payload, uint8_t payloadLength, uint8_t frameId, uint8_t srcEndpoint, uint8_t dstEndpoint, uint16_t clusterId, uint16_t profileId)
: ZBTxRequest(addr64, addr16, broadcastRadius, option, payload, payloadLength, frameId) {
	_srcEndpoint = srcEndpoint;
	_dstEndpoint = dstEndpoint;
	_profileId = profileId;
	_clusterId = clusterId;
	setApiId(ZB_EXPLICIT_TX_REQUEST);
}

ZBExplicitTxRequest::ZBExplicitTxRequest(XBeeAddress64 &addr64, uint8_t *payload, uint8_t payloadLength)
: ZBTxRequest(addr64, payload, payloadLength) {
	_srcEndpoint = DEFAULT_ENDPOINT;
	_dstEndpoint = DEFAULT_ENDPOINT;
	_profileId = DEFAULT_PROFILE_ID;
	_clusterId = DEFAULT_CLUSTER_ID;
	setApiId(ZB_EXPLICIT_TX_REQUEST);
}

uint8_t ZBExplicitTxRequest::getFrameData(uint8_t pos) {
	if (pos < 10) {
		return ZBTxRequest::getFrameData(pos);
	} else if (pos == 10) {
		return _srcEndpoint;
	} else if (pos == 11) {
		return _dstEndpoint;
	} else if (pos == 12) {
		return (_clusterId >> 8) & 0xff;
	} else if (pos == 13) {
		return _clusterId & 0xff;
	} else if (pos == 14) {
		return (_profileId >> 8) & 0xff;
	} else if (pos == 15) {
		return _profileId & 0xff;
	} else if (pos == 16) {
		return _broadcastRadius;
	} else if (pos == 17) {
		return _option;
	} else {
		return getPayload()[pos - ZB_EXPLICIT_TX_API_LENGTH];
	}
}

uint8_t ZBExplicitTxRequest::getFrameDataLength() {
	return ZB_EXPLICIT_TX_API_LENGTH + getPayloadLength();
}

uint8_t ZBExplicitTxRequest::getSrcEndpoint() {
	return _srcEndpoint;
}

uint8_t ZBExplicitTxRequest::getDstEndpoint() {
	return _dstEndpoint;
}

uint16_t ZBExplicitTxRequest::getClusterId() {
	return _clusterId;
}

uint16_t ZBExplicitTxRequest::getProfileId() {
	return _profileId;
}

void ZBExplicitTxRequest::setSrcEndpoint(uint8_t endpoint) {
	_srcEndpoint = endpoint;
}

void ZBExplicitTxRequest::setDstEndpoint(uint8_t endpoint) {
	_dstEndpoint = endpoint;
}

void ZBExplicitTxRequest::setClusterId(uint16_t clusterId) {
	_clusterId = clusterId;
}

void ZBExplicitTxRequest::setProfileId(uint16_t profileId) {
	_profileId = profileId;
}
#endif

#ifdef SERIES_1

Tx16Request::Tx16Request() : PayloadRequest(TX_16_REQUEST, DEFAULT_FRAME_ID, NULL, 0) {
	_option = ACK_OPTION;
}

Tx16Request::Tx16Request(uint16_t addr16, uint8_t option, uint8_t *data, uint8_t dataLength, uint8_t frameId) : PayloadRequest(TX_16_REQUEST, frameId, data, dataLength) {
	_addr16 = addr16;
	_option = option;
}

Tx16Request::Tx16Request(uint16_t addr16, uint8_t *data, uint8_t dataLength) : PayloadRequest(TX_16_REQUEST, DEFAULT_FRAME_ID, data, dataLength) {
	_addr16 = addr16;
	_option = ACK_OPTION;
}

uint8_t Tx16Request::getFrameData(uint8_t pos) {

	if (pos == 0) {
		return (_addr16 >> 8) & 0xff;
	} else if (pos == 1) {
		return _addr16 & 0xff;
	} else if (pos == 2) {
		return _option;
	} else {
		return getPayload()[pos - TX_16_API_LENGTH];
	}
}

uint8_t Tx16Request::getFrameDataLength() {
	return TX_16_API_LENGTH + getPayloadLength();
}

uint16_t Tx16Request::getAddress16() {
	return _addr16;
}

void Tx16Request::setAddress16(uint16_t addr16) {
	_addr16 = addr16;
}

uint8_t Tx16Request::getOption() {
	return _option;
}

void Tx16Request::setOption(uint8_t option) {
	_option = option;
}

Tx64Request::Tx64Request() : PayloadRequest(TX_64_REQUEST, DEFAULT_FRAME_ID, NULL, 0) {
	_option = ACK_OPTION;
}

Tx64Request::Tx64Request(XBeeAddress64 &addr64, uint8_t option, uint8_t *data, uint8_t dataLength, uint8_t frameId) : PayloadRequest(TX_64_REQUEST, frameId, data, dataLength) {
	_addr64 = addr64;
	_option = option;
}

Tx64Request::Tx64Request(XBeeAddress64 &addr64, uint8_t *data, uint8_t dataLength) : PayloadRequest(TX_64_REQUEST, DEFAULT_FRAME_ID, data, dataLength) {
	_addr64 = addr64;
	_option = ACK_OPTION;
}

uint8_t Tx64Request::getFrameData(uint8_t pos) {

	if (pos == 0) {
		return (_addr64.getMsb() >> 24) & 0xff;
	} else if (pos == 1) {
		return (_addr64.getMsb() >> 16) & 0xff;
	} else if (pos == 2) {
		return (_addr64.getMsb() >> 8) & 0xff;
	} else if (pos == 3) {
		return _addr64.getMsb() & 0xff;
	} else if (pos == 4) {
		return (_addr64.getLsb() >> 24) & 0xff;
	} else if (pos == 5) {
		return (_addr64.getLsb() >> 16) & 0xff;
	} else if (pos == 6) {
		return(_addr64.getLsb() >> 8) & 0xff;
	} else if (pos == 7) {
		return _addr64.getLsb() & 0xff;
	} else if (pos == 8) {
		return _option;
	} else {
		return getPayload()[pos - TX_64_API_LENGTH];
	}
}

uint8_t Tx64Request::getFrameDataLength() {
	return TX_64_API_LENGTH + getPayloadLength();
}

XBeeAddress64& Tx64Request::getAddress64() {
	return _addr64;
}

void Tx64Request::setAddress64(XBeeAddress64& addr64) {
	_addr64 = addr64;
}

uint8_t Tx64Request::getOption() {
	return _option;
}

void Tx64Request::setOption(uint8_t option) {
	_option = option;
}

#endif

AtCommandRequest::AtCommandRequest() : XBeeRequest(AT_COMMAND_REQUEST, DEFAULT_FRAME_ID) {
	_command = NULL;
	clearCommandValue();
}

AtCommandRequest::AtCommandRequest(uint8_t *command, uint8_t *commandValue, uint8_t commandValueLength) : XBeeRequest(AT_COMMAND_REQUEST, DEFAULT_FRAME_ID) {
	_command = command;
	_commandValue = commandValue;
	_commandValueLength = commandValueLength;
}

AtCommandRequest::AtCommandRequest(uint8_t *command) : XBeeRequest(AT_COMMAND_REQUEST, DEFAULT_FRAME_ID) {
	_command = command;
	clearCommandValue();
}

uint8_t* AtCommandRequest::getCommand() {
	return _command;
}

uint8_t* AtCommandRequest::getCommandValue() {
	return _commandValue;
}

uint8_t AtCommandRequest::getCommandValueLength() {
	return _commandValueLength;
}

void AtCommandRequest::setCommand(uint8_t* command) {
	_command = command;
}

void AtCommandRequest::setCommandValue(uint8_t* value) {
	_commandValue = value;
}

void AtCommandRequest::setCommandValueLength(uint8_t length) {
	_commandValueLength = length;
}

uint8_t AtCommandRequest::getFrameData(uint8_t pos) {

	if (pos == 0) {
		return _command[0];
	} else if (pos == 1) {
		return _command[1];
	} else {
		return _commandValue[pos - AT_COMMAND_API_LENGTH];
	}
}

void AtCommandRequest::clearCommandValue() {
	_commandValue = NULL;
	_commandValueLength = 0;
}

//void AtCommandRequest::reset() {
//	 XBeeRequest::reset();
//}

uint8_t AtCommandRequest::getFrameDataLength() {
	// command is 2 byte + length of value
	return AT_COMMAND_API_LENGTH + _commandValueLength;
}

XBeeAddress64 RemoteAtCommandRequest::broadcastAddress64 = XBeeAddress64(0x0, BROADCAST_ADDRESS);

RemoteAtCommandRequest::RemoteAtCommandRequest() : AtCommandRequest(NULL, NULL, 0) {
	_remoteAddress16 = 0;
	_applyChanges = false;
	setApiId(REMOTE_AT_REQUEST);
}

RemoteAtCommandRequest::RemoteAtCommandRequest(uint16_t remoteAddress16, uint8_t *command, uint8_t *commandValue, uint8_t commandValueLength) : AtCommandRequest(command, commandValue, commandValueLength) {
	_remoteAddress64 = broadcastAddress64;
	_remoteAddress16 = remoteAddress16;
	_applyChanges = true;
	setApiId(REMOTE_AT_REQUEST);
}

RemoteAtCommandRequest::RemoteAtCommandRequest(uint16_t remoteAddress16, uint8_t *command) : AtCommandRequest(command, NULL, 0) {
	_remoteAddress64 = broadcastAddress64;
	_remoteAddress16 = remoteAddress16;
	_applyChanges = false;
	setApiId(REMOTE_AT_REQUEST);
}

RemoteAtCommandRequest::RemoteAtCommandRequest(XBeeAddress64 &remoteAddress64, uint8_t *command, uint8_t *commandValue, uint8_t commandValueLength) : AtCommandRequest(command, commandValue, commandValueLength) {
	_remoteAddress64 = remoteAddress64;
	// don't worry.. works for series 1 too!
	_remoteAddress16 = ZB_BROADCAST_ADDRESS;
	_applyChanges = true;
	setApiId(REMOTE_AT_REQUEST);
}

RemoteAtCommandRequest::RemoteAtCommandRequest(XBeeAddress64 &remoteAddress64, uint8_t *command) : AtCommandRequest(command, NULL, 0) {
	_remoteAddress64 = remoteAddress64;
	_remoteAddress16 = ZB_BROADCAST_ADDRESS;
	_applyChanges = false;
	setApiId(REMOTE_AT_REQUEST);
}

uint16_t RemoteAtCommandRequest::getRemoteAddress16() {
	return _remoteAddress16;
}

void RemoteAtCommandRequest::setRemoteAddress16(uint16_t remoteAddress16) {
	_remoteAddress16 = remoteAddress16;
}

XBeeAddress64& RemoteAtCommandRequest::getRemoteAddress64() {
	return _remoteAddress64;
}

void RemoteAtCommandRequest::setRemoteAddress64(XBeeAddress64 &remoteAddress64) {
	_remoteAddress64 = remoteAddress64;
}

bool RemoteAtCommandRequest::getApplyChanges() {
	return _applyChanges;
}

void RemoteAtCommandRequest::setApplyChanges(bool applyChanges) {
	_applyChanges = applyChanges;
}


uint8_t RemoteAtCommandRequest::getFrameData(uint8_t pos) {
	if (pos == 0) {
		return (_remoteAddress64.getMsb() >> 24) & 0xff;
	} else if (pos == 1) {
		return (_remoteAddress64.getMsb() >> 16) & 0xff;
	} else if (pos == 2) {
		return (_remoteAddress64.getMsb() >> 8) & 0xff;
	} else if (pos == 3) {
		return _remoteAddress64.getMsb() & 0xff;
	} else if (pos == 4) {
		return (_remoteAddress64.getLsb() >> 24) & 0xff;
	} else if (pos == 5) {
		return (_remoteAddress64.getLsb() >> 16) & 0xff;
	} else if (pos == 6) {
		return(_remoteAddress64.getLsb() >> 8) & 0xff;
	} else if (pos == 7) {
		return _remoteAddress64.getLsb() & 0xff;
	} else if (pos == 8) {
		return (_remoteAddress16 >> 8) & 0xff;
	} else if (pos == 9) {
		return _remoteAddress16 & 0xff;
	} else if (pos == 10) {
		return _applyChanges ? 2: 0;
	} else if (pos == 11) {
		return getCommand()[0];
	} else if (pos == 12) {
		return getCommand()[1];
	} else {
		return getCommandValue()[pos - REMOTE_AT_COMMAND_API_LENGTH];
	}
}

uint8_t RemoteAtCommandRequest::getFrameDataLength() {
	return REMOTE_AT_COMMAND_API_LENGTH + getCommandValueLength();
}


// TODO
//GenericRequest::GenericRequest(uint8_t* frame, uint8_t len, uint8_t apiId): XBeeRequest(apiId, *(frame), len) {
//	_frame = frame;
//}

void XBee::send(XBeeRequest &request) {
	// the new new deal

	sendByte(START_BYTE, false);

	// send length
	uint8_t msbLen = ((request.getFrameDataLength() + 2) >> 8) & 0xff;
	uint8_t lsbLen = (request.getFrameDataLength() + 2) & 0xff;

	sendByte(msbLen, true);
	sendByte(lsbLen, true);

	// api id
	sendByte(request.getApiId(), true);
	sendByte(request.getFrameId(), true);

	uint8_t checksum = 0;

	// compute checksum, start at api id
	checksum+= request.getApiId();
	checksum+= request.getFrameId();

	for (int i = 0; i < request.getFrameDataLength(); i++) {
		sendByte(request.getFrameData(i), true);
		checksum+= request.getFrameData(i);
	}

	// perform 2s complement
	checksum = 0xff - checksum;

	// send checksum
	sendByte(checksum, true);
}

void XBee::sendByte(uint8_t b, bool escape) {

	if (escape && (b == START_BYTE || b == ESCAPE || b == XON || b == XOFF)) {
		write(ESCAPE);
		write(b ^ 0x20);
	} else {
		write(b);
	}
}


void XBeeWithCallbacks::loop() {
	if (loopTop())
		loopBottom();
}

bool XBeeWithCallbacks::loopTop() {
	readPacket();
	if (getResponse().isAvailable()) {
		_onResponse.call(getResponse());
		return true;
	} else if (getResponse().isError()) {
		_onPacketError.call(getResponse().getErrorCode());
	}
	return false;
}

void XBeeWithCallbacks::loopBottom() {
	bool called = false;
	uint8_t id = getResponse().getApiId();

	if (id == ZB_TX_STATUS_RESPONSE) {
		ZBTxStatusResponse response;
		getResponse().getZBTxStatusResponse(response);
		called = _onZBTxStatusResponse.call(response);
	} else if (id == ZB_RX_RESPONSE) {
		ZBRxResponse response;
		getResponse().getZBRxResponse(response);
		called = _onZBRxResponse.call(response);
	} else if (id == ZB_EXPLICIT_RX_RESPONSE) {
		ZBExplicitRxResponse response;
		getResponse().getZBExplicitRxResponse(response);
		called = _onZBExplicitRxResponse.call(response);
	} else if (id == ZB_IO_SAMPLE_RESPONSE) {
		ZBRxIoSampleResponse response;
		getResponse().getZBRxIoSampleResponse(response);
		called = _onZBRxIoSampleResponse.call(response);
	} else if (id == TX_STATUS_RESPONSE) {
		TxStatusResponse response;
		getResponse().getTxStatusResponse(response);
		called = _onTxStatusResponse.call(response);
	} else if (id == RX_16_RESPONSE) {
		Rx16Response response;
		getResponse().getRx16Response(response);
		called = _onRx16Response.call(response);
	} else if (id == RX_64_RESPONSE) {
		Rx64Response response;
		getResponse().getRx64Response(response);
		called = _onRx64Response.call(response);
	} else if (id == RX_16_IO_RESPONSE) {
		Rx16IoSampleResponse response;
		getResponse().getRx16IoSampleResponse(response);
		called = _onRx16IoSampleResponse.call(response);
	} else if (id == RX_64_IO_RESPONSE) {
		Rx64IoSampleResponse response;
		getResponse().getRx64IoSampleResponse(response);
		called = _onRx64IoSampleResponse.call(response);
	} else if (id == MODEM_STATUS_RESPONSE) {
		ModemStatusResponse response;
		getResponse().getModemStatusResponse(response);
		called = _onModemStatusResponse.call(response);
	} else if (id == AT_COMMAND_RESPONSE) {
		AtCommandResponse response;
		getResponse().getAtCommandResponse(response);
		called = _onAtCommandResponse.call(response);
	} else if (id == REMOTE_AT_COMMAND_RESPONSE) {
		RemoteAtCommandResponse response;
		getResponse().getRemoteAtCommandResponse(response);
		called = _onRemoteAtCommandResponse.call(response);
	}

	if (!called)
		_onOtherResponse.call(getResponse());
}

uint8_t XBeeWithCallbacks::matchStatus(uint8_t frameId) {
	uint8_t id = getResponse().getApiId();
	uint8_t *data = getResponse().getFrameData();
	uint8_t len = getResponse().getFrameDataLength();
	uint8_t offset = 0;

	// Figure out if this frame has a frameId and if so, where the
	// status byte to return is located
	if (id == AT_COMMAND_RESPONSE)
		offset = 3;
	else if (id == REMOTE_AT_COMMAND_RESPONSE)
		offset = 13;
	else if (id == TX_STATUS_RESPONSE)
		offset = 1;
	else if (id == ZB_TX_STATUS_RESPONSE)
		offset = 4;

	// If this is an API frame that contains a status, the frame is
	// long enough to contain it and the frameId matches the one
	// given, return the status byte
	if (offset && offset < len && data[0] == frameId)
		return data[offset];
	return 0xff;
}

uint8_t XBeeWithCallbacks::waitForInternal(uint8_t apiId, void *response, uint16_t timeout, void *func, uintptr_t data, int16_t frameId) {
	unsigned long start = millis();
	do {
		// Wait for a packet of the right type
		if (loopTop()) {
			if (frameId >= 0) {
				uint8_t status = matchStatus(frameId);
				// If a status was found, but it was not
				// a zero success status, stop waiting
				if (status != 0xff && status != 0)
					return status;
			}

			if (getResponse().getApiId() == apiId) {
				// If the type is right, call the right
				// conversion function based on the
				// ApiId and call the match function.
				// Because the match function is
				// essentially called in the same way,
				// regardless of the subclass used, the
				// compiler can reduce most of the below
				// mess into a single piece of code
				// (though for fully optimizing, the
				// separate getXxxResponse() methods
				// must be unified as well).
				switch(apiId) {
					case ZBTxStatusResponse::API_ID: {
						ZBTxStatusResponse *r = (ZBTxStatusResponse*)response;
						bool(*f)(ZBTxStatusResponse&,uintptr_t) = (bool(*)(ZBTxStatusResponse&,uintptr_t))func;
						getResponse().getZBTxStatusResponse(*r);
						if(!f || f(*r, data))
							return 0;
						break;
					}
					case ZBRxResponse::API_ID: {
						ZBRxResponse *r = (ZBRxResponse*)response;
						bool(*f)(ZBRxResponse&,uintptr_t) = (bool(*)(ZBRxResponse&,uintptr_t))func;
						getResponse().getZBRxResponse(*r);
						if(!f || f(*r, data))
							return 0;
						break;
					}
					case ZBExplicitRxResponse::API_ID: {
						ZBExplicitRxResponse *r = (ZBExplicitRxResponse*)response;
						bool(*f)(ZBExplicitRxResponse&,uintptr_t) = (bool(*)(ZBExplicitRxResponse&,uintptr_t))func;
						getResponse().getZBExplicitRxResponse(*r);
						if(!f || f(*r, data))
							return 0;
						break;
					}
					case ZBRxIoSampleResponse::API_ID: {
						ZBRxIoSampleResponse *r = (ZBRxIoSampleResponse*)response;
						bool(*f)(ZBRxIoSampleResponse&,uintptr_t) = (bool(*)(ZBRxIoSampleResponse&,uintptr_t))func;
						getResponse().getZBRxIoSampleResponse(*r);
						if(!f || f(*r, data))
							return 0;
						break;
					}
					case TxStatusResponse::API_ID: {
						TxStatusResponse *r = (TxStatusResponse*)response;
						bool(*f)(TxStatusResponse&,uintptr_t) = (bool(*)(TxStatusResponse&,uintptr_t))func;
						getResponse().getTxStatusResponse(*r);
						if(!f || f(*r, data))
							return 0;
						break;
					}
					case Rx16Response::API_ID: {
						Rx16Response *r = (Rx16Response*)response;
						bool(*f)(Rx16Response&,uintptr_t) = (bool(*)(Rx16Response&,uintptr_t))func;
						getResponse().getRx16Response(*r);
						if(!f || f(*r, data))
							return 0;
						break;
					}
					case Rx64Response::API_ID: {
						Rx64Response *r = (Rx64Response*)response;
						bool(*f)(Rx64Response&,uintptr_t) = (bool(*)(Rx64Response&,uintptr_t))func;
						getResponse().getRx64Response(*r);
						if(!f || f(*r, data))
							return 0;
						break;
					}
					case Rx16IoSampleResponse::API_ID: {
						Rx16IoSampleResponse *r = (Rx16IoSampleResponse*)response;
						bool(*f)(Rx16IoSampleResponse&,uintptr_t) = (bool(*)(Rx16IoSampleResponse&,uintptr_t))func;
						getResponse().getRx16IoSampleResponse(*r);
						if(!f || f(*r, data))
							return 0;
						break;
					}
					case Rx64IoSampleResponse::API_ID: {
						Rx64IoSampleResponse *r = (Rx64IoSampleResponse*)response;
						bool(*f)(Rx64IoSampleResponse&,uintptr_t) = (bool(*)(Rx64IoSampleResponse&,uintptr_t))func;
						getResponse().getRx64IoSampleResponse(*r);
						if(!f || f(*r, data))
							return 0;
						break;
					}
					case ModemStatusResponse::API_ID: {
						ModemStatusResponse *r = (ModemStatusResponse*)response;
						bool(*f)(ModemStatusResponse&,uintptr_t) = (bool(*)(ModemStatusResponse&,uintptr_t))func;
						getResponse().getModemStatusResponse(*r);
						if(!f || f(*r, data))
							return 0;
						break;
					}
					case AtCommandResponse::API_ID: {
						AtCommandResponse *r = (AtCommandResponse*)response;
						bool(*f)(AtCommandResponse&,uintptr_t) = (bool(*)(AtCommandResponse&,uintptr_t))func;
						getResponse().getAtCommandResponse(*r);
						if(!f || f(*r, data))
							return 0;
						break;
					}
					case RemoteAtCommandResponse::API_ID: {
						RemoteAtCommandResponse *r = (RemoteAtCommandResponse*)response;
						bool(*f)(RemoteAtCommandResponse&,uintptr_t) = (bool(*)(RemoteAtCommandResponse&,uintptr_t))func;
						getResponse().getRemoteAtCommandResponse(*r);
						if(!f || f(*r, data))
							return 0;
						break;
					}
				}
			}
			// Call regular callbacks
			loopBottom();
		}
	} while (millis() - start < timeout);
	return XBEE_WAIT_TIMEOUT;
}

uint8_t XBeeWithCallbacks::waitForStatus(uint8_t frameId, uint16_t timeout) {
	unsigned long start = millis();
	do {
		if (loopTop()) {
			uint8_t status = matchStatus(frameId);
			if (status != 0xff)
				return status;

			// Call regular callbacks
			loopBottom();
		}
	} while (millis() - start < timeout);
	return XBEE_WAIT_TIMEOUT ;
}

#endif //XBee_h
