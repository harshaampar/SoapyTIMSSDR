#pragma once
#include <timssdr.h>
#include <string.h>
#include <mutex>
#include <condition_variable>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Logger.hpp>
#include <set>

#define BUF_LEN			262144
#define BUF_NUM			15
#define BYTES_PER_SAMPLE	2

enum TimsSDR_Format {
	TIMSSDR_FORMAT_FLOAT32	=0,
	TIMSSDR_FORMAT_INT16	=1,
	TIMSSDR_FORMAT_INT8	=2,
	TIMSSDR_FORMAT_FLOAT64 =3,
};

typedef enum {
	TIMSSDR_TRANSCEIVER_MODE_OFF = 0,
	TIMSSDR_TRANSCEIVER_MODE_RX = 1,
	TIMSSDR_TRANSCEIVER_MODE_TX = 2,
} TimsSDR_transceiver_mode_t;

std::set<std::string> &TimsSDR_getClaimedSerials(void);

/*!
 * The session object manages timssdr_init/exit
 * with a process-wide reference count.
 */
class SoapyTimsSDRSession
{
public:
	SoapyTimsSDRSession(void);
	~SoapyTimsSDRSession(void);
};

class SoapyTimsSDR : public SoapySDR::Device
{
public:
	SoapyTimsSDR( const SoapySDR::Kwargs & args );

	~SoapyTimsSDR( void );


	/*******************************************************************
	 * Identification API
	 ******************************************************************/

	std::string getDriverKey( void ) const;


	std::string getHardwareKey( void ) const;


	SoapySDR::Kwargs getHardwareInfo( void ) const;


	/*******************************************************************
	 * Channels API
	 ******************************************************************/

	size_t getNumChannels( const int ) const;


	bool getFullDuplex( const int direction, const size_t channel ) const;


	/*******************************************************************
	 * Stream API
	 ******************************************************************/

	std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const;

	std::string getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const;

	SoapySDR::ArgInfoList getStreamArgsInfo(const int direction, const size_t channel) const;

	SoapySDR::Stream *setupStream(
		const int direction,
		const std::string &format,
		const std::vector<size_t> &channels = std::vector<size_t>(),
		const SoapySDR::Kwargs &args = SoapySDR::Kwargs() );


	void closeStream( SoapySDR::Stream *stream );


	size_t getStreamMTU( SoapySDR::Stream *stream ) const;


	int activateStream(
		SoapySDR::Stream *stream,
		const int flags = 0,
		const long long timeNs = 0,
		const size_t numElems = 0 );


	int deactivateStream(
		SoapySDR::Stream *stream,
		const int flags = 0,
		const long long timeNs = 0 );


	int readStream(
		SoapySDR::Stream *stream,
		void * const *buffs,
		const size_t numElems,
		int &flags,
		long long &timeNs,
		const long timeoutUs = 100000 );


	int writeStream(
			SoapySDR::Stream *stream,
			const void * const *buffs,
			const size_t numElems,
			int &flags,
			const long long timeNs = 0,
			const long timeoutUs = 100000);

	int readStreamStatus(
			SoapySDR::Stream *stream,
			size_t &chanMask,
			int &flags,
			long long &timeNs,
			const long timeoutUs
	);


	int acquireReadBuffer(
			SoapySDR::Stream *stream,
			size_t &handle,
			const void **buffs,
			int &flags,
			long long &timeNs,
			const long timeoutUs = 100000);

	void releaseReadBuffer(
			SoapySDR::Stream *stream,
			const size_t handle);

	int acquireWriteBuffer(
			SoapySDR::Stream *stream,
			size_t &handle,
			void **buffs,
			const long timeoutUs = 100000);

	void releaseWriteBuffer(
			SoapySDR::Stream *stream,
			const size_t handle,
			const size_t numElems,
			int &flags,
			const long long timeNs = 0);

	size_t getNumDirectAccessBuffers(SoapySDR::Stream *stream);

	int getDirectAccessBufferAddrs(SoapySDR::Stream *stream, const size_t handle, void **buffs);

	/*******************************************************************
	 * Settings API
	 ******************************************************************/

	SoapySDR::ArgInfoList getSettingInfo(void) const;


	void writeSetting(const std::string &key, const std::string &value);


	std::string readSetting(const std::string &key) const;


	/*******************************************************************
	 * Antenna API
	 ******************************************************************/

	std::vector<std::string> listAntennas( const int direction, const size_t channel ) const;


	void setAntenna( const int direction, const size_t channel, const std::string &name );


	std::string getAntenna( const int direction, const size_t channel ) const;


	/*******************************************************************
	 * Frontend corrections API
	 ******************************************************************/

	bool hasDCOffsetMode( const int direction, const size_t channel ) const;


	/*******************************************************************
	 * Gain API
	 ******************************************************************/

	std::vector<std::string> listGains( const int direction, const size_t channel ) const;


	void setGainMode( const int direction, const size_t channel, const bool automatic );


	bool getGainMode( const int direction, const size_t channel ) const;


	void setGain( const int direction, const size_t channel, const double value );


	void setGain( const int direction, const size_t channel, const std::string &name, const double value );


	double getGain( const int direction, const size_t channel, const std::string &name ) const;


	SoapySDR::Range getGainRange( const int direction, const size_t channel, const std::string &name ) const;


	/*******************************************************************
	 * Frequency API
	 ******************************************************************/

	void setFrequency( const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &args = SoapySDR::Kwargs() );


	double getFrequency( const int direction, const size_t channel, const std::string &name ) const;


	SoapySDR::ArgInfoList getFrequencyArgsInfo(const int direction, const size_t channel) const;


	std::vector<std::string> listFrequencies( const int direction, const size_t channel ) const;


	SoapySDR::RangeList getFrequencyRange( const int direction, const size_t channel, const std::string &name ) const;


	/*******************************************************************
	 * Sample Rate API
	 ******************************************************************/

	void setSampleRate( const int direction, const size_t channel, const double rate );


	double getSampleRate( const int direction, const size_t channel ) const;


	std::vector<double> listSampleRates( const int direction, const size_t channel ) const;


	void setBandwidth( const int direction, const size_t channel, const double bw );


	double getBandwidth( const int direction, const size_t channel ) const;


	std::vector<double> listBandwidths( const int direction, const size_t channel ) const;

	/*******************************************************************
 	 * TimsSDR callback
 	 ******************************************************************/
	int timssdr_tx_callback( int8_t *buffer, int32_t length );


	int timssdr_rx_callback( int8_t *buffer, int32_t length );




private:

	SoapySDR::Stream* const TX_STREAM = (SoapySDR::Stream*) 0x1;
	SoapySDR::Stream* const RX_STREAM = (SoapySDR::Stream*) 0x2;

	struct Stream {
		Stream(): opened(false), buf_num(BUF_NUM), buf_len(BUF_LEN), buf(nullptr),
				  buf_head(0), buf_tail(0), buf_count(0),
				  remainderHandle(-1), remainderSamps(0), remainderOffset(0), remainderBuff(nullptr),
				  format(TIMSSDR_FORMAT_INT8) {}

		bool opened;
		uint32_t	buf_num;
		uint32_t	buf_len;
		int8_t		**buf;
		uint32_t	buf_head;
		uint32_t	buf_tail;
		uint32_t	buf_count;

		int32_t remainderHandle;
		size_t remainderSamps;
		size_t remainderOffset;
		int8_t* remainderBuff;
		uint32_t format;

		~Stream() { clear_buffers(); }
		void clear_buffers();
		void allocate_buffers();
	};

	struct RXStream: Stream {
		uint32_t vga_gain;
		uint32_t lna_gain;
		uint8_t amp_gain;
		double samplerate;
		uint32_t bandwidth;
		uint64_t frequency;

		bool overflow;
	};

	struct TXStream: Stream {
		uint32_t vga_gain;
		uint8_t amp_gain;
		double samplerate;
		uint32_t bandwidth;
		uint64_t frequency;
		bool bias;

		bool underflow;

		bool burst_end;
		int32_t burst_samps;
	} ;

	RXStream _rx_stream;
	TXStream _tx_stream;

	bool _auto_bandwidth;

	timssdr_device * _dev;
	std::string _serial;

	uint64_t _current_frequency;

	double _current_samplerate;

	uint32_t _current_bandwidth;

	uint8_t _current_amp;

	/// Mutex protecting all use of the timssdr device _dev and other instance variables.
	/// Most of the timssdr API is thread-safe because it only calls libusb, however
	/// the activateStream() method in this library can close and re-open the device,
	/// so all use of _dev must be protected
	mutable std::mutex	_device_mutex;
	std::mutex	_buf_mutex;
	std::condition_variable _buf_cond;

	TimsSDR_transceiver_mode_t _current_mode;

	SoapyTimsSDRSession _sess;
};
