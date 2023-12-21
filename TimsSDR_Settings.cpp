/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015-2016 Wei Jiang
 * Copyright (c) 2015-2017 Josh Blum
 * Copyright (c) 2017 Kevin Mehall
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "SoapyTimsSDR.hpp"

std::set<std::string> &TimsSDR_getClaimedSerials(void)
{
	static std::set<std::string> serials;
	return serials;
}

SoapyTimsSDR::SoapyTimsSDR( const SoapySDR::Kwargs &args )
{
	if (args.count("label") != 0)
		SoapySDR_logf( SOAPY_SDR_INFO, "Opening %s...", args.at("label").c_str());

	_rx_stream.vga_gain=16;
	_rx_stream.lna_gain=16;
	_rx_stream.amp_gain=0;
	_rx_stream.frequency=0;
	_rx_stream.samplerate=0;
	_rx_stream.bandwidth=0;
	_rx_stream.overflow = false;

	_tx_stream.vga_gain=0;
	_tx_stream.amp_gain=0;
	_tx_stream.frequency=0;
	_tx_stream.samplerate=0;
	_tx_stream.bandwidth=0;
	_tx_stream.burst_samps=0;
	_tx_stream.burst_end=false;
	_tx_stream.underflow = false;

	_current_mode= TIMSSDR_TRANSCEIVER_MODE_OFF;

	_auto_bandwidth=true;

	_dev		= nullptr;

	if (args.count("serial") == 0)
		throw std::runtime_error("no timssdr device matches");
	_serial = args.at("serial");

	_current_amp = 0;

	_current_frequency = 0;

	_current_samplerate = 0;

	_current_bandwidth=0;

	int ret = timssdr_open_by_serial(_serial.c_str(), &_dev);
	if ( ret != TIMSSDR_SUCCESS )
	{
		SoapySDR_logf( SOAPY_SDR_INFO, "Could not Open HackRF Device" );
		throw std::runtime_error("timssdr open failed");
	}

	TimsSDR_getClaimedSerials().insert(_serial);
}


SoapyTimsSDR::~SoapyTimsSDR( void )
{
	TimsSDR_getClaimedSerials().erase(_serial);

	if ( _dev )
	{
		timssdr_close( _dev );
	}

	/* cleanup device handles */
}


/*******************************************************************
 * Identification API
 ******************************************************************/

std::string SoapyTimsSDR::getDriverKey( void ) const
{

	return("TimsSDR");
}


std::string SoapyTimsSDR::getHardwareKey( void ) const
{
	return "TimsSDR";
}


SoapySDR::Kwargs SoapyTimsSDR::getHardwareInfo( void ) const
{
	std::lock_guard<std::mutex> lock(_device_mutex);
	SoapySDR::Kwargs info;

	info["version"] = "v1.0";

	read_partid_serialno_t read_partid_serialno;

	timssdr_board_partid_serialno_read(_dev, &read_partid_serialno);

	char part_id_str[100];

	sprintf(part_id_str, "%08x%08x", read_partid_serialno.part_id[0], read_partid_serialno.part_id[1]);

	info["part id"] = part_id_str;

	char serial_str[100];
	sprintf(serial_str, "%08x%08x%08x%08x", read_partid_serialno.serial_no[0], read_partid_serialno.serial_no[1], read_partid_serialno.serial_no[2], read_partid_serialno.serial_no[3]);
	info["serial"] = serial_str;

	uint16_t clock;

	info["clock source"]="external";

	return(info);

}


/*******************************************************************
 * Channels API
 ******************************************************************/

size_t SoapyTimsSDR::getNumChannels( const int dir ) const
{
	return(1);
}


bool SoapyTimsSDR::getFullDuplex( const int direction, const size_t channel ) const
{
	return(false);
}

/*******************************************************************
 * Settings API
 ******************************************************************/

SoapySDR::ArgInfoList SoapyTimsSDR::getSettingInfo(void) const
{
	SoapySDR::ArgInfoList setArgs;

	return setArgs;
}

void SoapyTimsSDR::writeSetting(const std::string &key, const std::string &value)
{

}

std::string SoapyTimsSDR::readSetting(const std::string &key) const
{
	return "";
}
/*******************************************************************
 * Antenna API
 ******************************************************************/

std::vector<std::string> SoapyTimsSDR::listAntennas( const int direction, const size_t channel ) const
{
	std::vector<std::string> options;
	options.push_back( "TX/RX" );
	return(options);
}


void SoapyTimsSDR::setAntenna( const int direction, const size_t channel, const std::string &name )
{
	/* TODO delete this function or throw if name != RX... */
}


std::string SoapyTimsSDR::getAntenna( const int direction, const size_t channel ) const
{
	return("TX/RX");
}


/*******************************************************************
 * Frontend corrections API
 ******************************************************************/


bool SoapyTimsSDR::hasDCOffsetMode( const int direction, const size_t channel ) const
{
	return(false);
}


/*******************************************************************
 * Gain API
 ******************************************************************/

std::vector<std::string> SoapyTimsSDR::listGains( const int direction, const size_t channel ) const
{
	std::vector<std::string> options;

	return(options);
	/*
	 * list available gain elements,
	 * the functions below have a "name" parameter
	 */
}


void SoapyTimsSDR::setGainMode( const int direction, const size_t channel, const bool automatic )
{
	/* enable AGC if the hardware supports it, or remove this function */
}


bool SoapyTimsSDR::getGainMode( const int direction, const size_t channel ) const
{
	return(false);
	/* ditto for the AGC */
}


void SoapyTimsSDR::setGain( const int direction, const size_t channel, const double value )
{
	return;
}


void SoapyTimsSDR::setGain( const int direction, const size_t channel, const std::string &name, const double value )
{
	return;
	/* set individual gain element by name */
}


double SoapyTimsSDR::getGain( const int direction, const size_t channel, const std::string &name ) const
{
	std::lock_guard<std::mutex> lock(_device_mutex);
	double gain = 0.0;
	return(gain);
}


SoapySDR::Range SoapyTimsSDR::getGainRange( const int direction, const size_t channel, const std::string &name ) const
{
	return(SoapySDR::Range( 0, 0 ) );
}


/*******************************************************************
 * Frequency API
 ******************************************************************/

void SoapyTimsSDR::setFrequency( const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &args )
{
	return;
}


double SoapyTimsSDR::getFrequency( const int direction, const size_t channel, const std::string &name ) const
{
	return(0.0);
}

SoapySDR::ArgInfoList SoapyTimsSDR::getFrequencyArgsInfo(const int direction, const size_t channel) const
{
	SoapySDR::ArgInfoList freqArgs;
	// TODO: frequency arguments
	return freqArgs;
}

std::vector<std::string> SoapyTimsSDR::listFrequencies( const int direction, const size_t channel ) const
{
	std::vector<std::string> names;
	names.push_back( "BB" );
	return(names);
}


SoapySDR::RangeList SoapyTimsSDR::getFrequencyRange( const int direction, const size_t channel, const std::string &name ) const
{
	return(SoapySDR::RangeList( 1, SoapySDR::Range( 0.0, 0.0 ) ) );
}


/*******************************************************************
 * Sample Rate API
 ******************************************************************/

void SoapyTimsSDR::setSampleRate( const int direction, const size_t channel, const double rate )
{
	return;
}


double SoapyTimsSDR::getSampleRate( const int direction, const size_t channel ) const
{
	std::lock_guard<std::mutex> lock(_device_mutex);
	double samp(0.0);

	return(samp);
}


std::vector<double> SoapyTimsSDR::listSampleRates( const int direction, const size_t channel ) const
{
	std::vector<double> options;
	return(options);
}


void SoapyTimsSDR::setBandwidth( const int direction, const size_t channel, const double bw )
{
	return;
}


double SoapyTimsSDR::getBandwidth( const int direction, const size_t channel ) const
{
	std::lock_guard<std::mutex> lock(_device_mutex);
	double bw(0.0);
	if(direction==SOAPY_SDR_RX){

		bw = _rx_stream.bandwidth;
	}
	if(direction==SOAPY_SDR_TX){

		bw = _tx_stream.bandwidth;
	}

	return (bw);
}


std::vector<double> SoapyTimsSDR::listBandwidths( const int direction, const size_t channel ) const
{
	std::vector<double> options;
	return(options);
}
