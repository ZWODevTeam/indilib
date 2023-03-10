/*******************************************************************************
 Copyright(c) 2010, 2017 Ilia Platone, Jasem Mutlaq. All rights reserved.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Library General Public
 License version 2 as published by the Free Software Foundation.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Library General Public License for more details.

 You should have received a copy of the GNU Library General Public License
 along with this library; see the file COPYING.LIB.  If not, write to
 the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 Boston, MA 02110-1301, USA.
*******************************************************************************/

#pragma once

#include "defaultdevice.h"
#include "dsp.h"
#include <fitsio.h>

#ifdef HAVE_WEBSOCKET
#include "indiwsserver.h"
#endif

#include <fitsio.h>

#include <memory>
#include <cstring>
#include <chrono>
#include <stdint.h>
#include <mutex>
#include <thread>

//JM 2019-01-17: Disabled until further notice
//#define WITH_EXPOSURE_LOOPING

extern const char *CAPTURE_SETTINGS_TAB;
extern const char *CAPTURE_INFO_TAB;
extern const char *GUIDE_HEAD_TAB;


/**
 * \class INDI::Detector
 * \brief Class to provide general functionality of Monodimensional Detector.
 *
 * The Detector capabilities must be set to select which features are exposed to the clients.
 * SetDetectorCapability() is typically set in the constructor or initProperties(), but can also be
 * called after connection is established with the Detector, but must be called /em before returning
 * true in Connect().
 *
 * Developers need to subclass INDI::Detector to implement any driver for Detectors within INDI.
 *
 * \example Detector Simulator
 * \author Jasem Mutlaq, Ilia Platone
 *
 */

namespace INDI
{
class StreamManager;

/**
 * @brief The DetectorDevice class provides functionality of a Detector Device within a Detector.
 */
class DetectorDevice
{
    public:
        DetectorDevice();
        ~DetectorDevice();

        typedef enum
        {
            DETECTOR_SAMPLERATE,
            DETECTOR_FREQUENCY,
            DETECTOR_BITSPERSAMPLE,
            DETECTOR_GAIN,
            DETECTOR_BANDWIDTH,
            DETECTOR_CHANNEL,
            DETECTOR_ANTENNA,
        } DETECTOR_INFO_INDEX;

        typedef enum
        {
            DETECTOR_BLOB_CONTINUUM,
            DETECTOR_BLOB_SPECTRUM,
            DETECTOR_BLOB_TDEV,
        } DETECTOR_BLOB_INDEX;

        /**
         * @brief getBPS Get Detector depth (bits per sample).
         * @return bits per sample.
         */
        inline int getBPS()
        {
            return BPS;
        }

        /**
         * @brief getContinuumBufferSize Get allocated continuum buffer size to hold the Detector captured stream.
         * @return allocated continuum buffer size to hold the Detector capture stream.
         */
        inline int getContinuumBufferSize()
        {
            return ContinuumBufferSize;
        }

        /**
         * @brief getTimeDeviationBufferSize Get allocated time deviation buffer size to hold the Detector time correction stream.
         * @return allocated time correction buffer size to hold the Detector time correction stream.
         */
        inline int getTimeDeviationBufferSize()
        {
            return TimeDeviationBufferSize;
        }

        /**
         * @brief getSpectrumBufferSize Get allocated spectrum buffer size to hold the Detector spectrum.
         * @return allocated spectrum buffer size (in doubles) to hold the Detector spectrum.
         */
        inline int getSpectrumBufferSize()
        {
            return SpectrumBufferSize;
        }

        /**
         * @brief getCaptureLeft Get Capture time left in seconds.
         * @return Capture time left in seconds.
         */
        inline double getCaptureLeft()
        {
            return FramedCaptureN[0].value;
        }

        /**
         * @brief getSampleRate Get requested sample rate for the Detector device in Hz.
         * @return requested sample rate for the Detector device in Hz.
         */
        inline double getSampleRate()
        {
            return Samplerate;
        }

        /**
         * @brief getBandwidth Get requested capture bandwidth for the Detector device in Hz.
         * @return requested capture bandwidth for the Detector device in Hz.
         */
        inline double getBandwidth()
        {
            return Bandwidth;
        }

        /**
         * @brief getGain Get requested capture gain for the Detector device.
         * @return requested capture gain for the Detector device.
         */
        inline double getGain()
        {
            return Gain;
        }

        /**
         * @brief getFrequency Get requested capture frequency for the Detector device in Hz.
         * @return requested Capture frequency for the Detector device in Hz.
         */
        inline double getFrequency()
        {
            return Frequency;
        }

        /**
         * @brief getCaptureDuration Get requested Capture duration for the Detector device in seconds.
         * @return requested Capture duration for the Detector device in seconds.
         */
        inline double getCaptureDuration()
        {
            return captureDuration;
        }

        /**
         * @brief getCaptureStartTime
         * @return Capture start time in ISO 8601 format.
         */
        const char *getCaptureStartTime();

        /**
         * @brief getContinuumBuffer Get raw buffer of the continuum stream of the Detector device.
         * @return raw continuum buffer of the Detector device.
         */
        inline uint8_t *getContinuumBuffer()
        {
            return ContinuumBuffer;
        }

        /**
         * @brief getTimeDeviationBuffer Get raw buffer of the time correction stream of the Detector device.
         * @return raw time correction buffer of the Detector device.
         */
        inline uint8_t *getTimeDeviationBuffer()
        {
            return TimeDeviationBuffer;
        }

        /**
         * @brief getSpectrumBuffer Get raw buffer of the spectrum of the Detector device.
         * @return raw continuum buffer of the Detector device.
         */
        inline uint8_t *getSpectrumBuffer()
        {
            return SpectrumBuffer;
        }

        /**
         * @brief setContinuumBuffer Set raw frame buffer pointer.
         * @param buffer pointer to continuum buffer
         * /note Detector Device allocates the frame buffer internally once SetContinuumBufferSize is called
         * with allocMem set to true which is the default behavior. If you allocated the memory
         * yourself (i.e. allocMem is false), then you must call this function to set the pointer
         * to the raw frame buffer.
         */
        inline void setContinuumBuffer(uint8_t *buffer)
        {
            ContinuumBuffer = buffer;
        }

        /**
         * @brief setTimeDeviationBuffer Set raw frame buffer pointer.
         * @param buffer pointer to time correction buffer
         * /note Detector Device allocates the frame buffer internally once SetTimeDeviationBufferSize is called
         * with allocMem set to true which is the default behavior. If you allocated the memory
         * yourself (i.e. allocMem is false), then you must call this function to set the pointer
         * to the raw frame buffer.
         */
        inline void setTimeDeviationBuffer(uint8_t *buffer)
        {
            TimeDeviationBuffer = buffer;
        }

        /**
         * @brief setSpectrumBuffer Set raw frame buffer pointer.
         * @param buffer pointer to spectrum buffer
         * /note Detector Device allocates the frame buffer internally once SetSpectrumBufferSize is called
         * with allocMem set to true which is the default behavior. If you allocated the memory
         * yourself (i.e. allocMem is false), then you must call this function to set the pointer
         * to the raw frame buffer.
         */
        inline void setSpectrumBuffer(uint8_t *buffer)
        {
            SpectrumBuffer = buffer;
        }

        /**
         * @brief Return Detector Info Property
         */
        inline INumberVectorProperty *getDetectorSettings()
        {
            return &DetectorSettingsNP;
        }

        /**
         * @brief setMinMaxStep for a number property element
         * @param property Property name
         * @param element Element name
         * @param min Minimum element value
         * @param max Maximum element value
         * @param step Element step value
         * @param sendToClient If true (default), the element limits are updated and is sent to the
         * client. If false, the element limits are updated without getting sent to the client.
         */
        void setMinMaxStep(const char *property, const char *element, double min, double max, double step,
                           bool sendToClient = true);

        /**
         * @brief setContinuumBufferSize Set desired continuum buffer size. The function will allocate memory
         * accordingly. The frame size depends on the desired capture time, sampling frequency, and
         * sample depth of the Detector device (bps). You must set the frame size any time any of
         * the prior parameters gets updated.
         * @param nbuf size of buffer in bytes.
         * @param allocMem if True, it will allocate memory of nbut size bytes.
         */
        void setContinuumBufferSize(int nbuf, bool allocMem = true);

        /**
         * @brief setTimeDeviationBufferSize Set desired time deviation buffer size. The function will allocate memory
         * accordingly. The frame size depends on the desired capture time, sampling frequency, and
         * sample depth of the Detector device (bps). You must set the frame size any time any of
         * the prior parameters gets updated.
         * @param nbuf size of buffer in bytes.
         * @param allocMem if True, it will allocate memory of nbut size bytes.
         */
        void setTimeDeviationBufferSize(int nbuf, bool allocMem = true);

        /**
         * @brief setSpectrumBufferSize Set desired spectrum buffer size. The function will allocate memory
         * accordingly. The frame size depends on the size of the spectrum. You must set the frame size any
         * time the spectrum size changes.
         * @param nbuf size of buffer in doubles.
         * @param allocMem if True, it will allocate memory of nbut size doubles.
         */
        void setSpectrumBufferSize(int nbuf, bool allocMem = true);

        /**
         * @brief setSampleRate Set depth of Detector device.
         * @param bpp bits per pixel
         */
        void setSampleRate(float sr);

        /**
         * @brief setBandwidth Set bandwidth of Detector device.
         * @param bandwidth The detector bandwidth
         */
        void setBandwidth(float bandwidth);

        /**
         * @brief setGain Set gain of Detector device.
         * @param gain The requested gain
         */
        void setGain(float gain);

        /**
         * @brief setFrequency Set the frequency observed.
         * @param freq capture frequency
         */
        void setFrequency(float freq);

        /**
         * @brief setBPP Set depth of Detector device.
         * @param bpp bits per pixel
         */
        void setBPS(int bps);

        /**
         * @brief setCaptureDuration Set desired Detector frame Capture duration for next Capture. You must
         * call this function immediately before starting the actual Capture as it is used to calculate
         * the timestamp used for the FITS header.
         * @param duration Capture duration in seconds.
         */
        void setCaptureDuration(double duration);

        /**
         * @brief setCaptureLeft Update Capture time left. Inform the client of the new Capture time
         * left value.
         * @param duration Capture duration left in seconds.
         */
        void setCaptureLeft(double duration);

        /**
         * @brief setCaptureFailed Alert the client that the Capture failed.
         */
        void setCaptureFailed();

        /**
         * @return Get number of FITS axis in capture. By default 2
         */
        int getNAxis() const;

        /**
         * @brief setNAxis Set FITS number of axis
         * @param value number of axis
         */
        void setNAxis(int value);

        /**
         * @brief setCaptureExtension Set capture exntension
         * @param ext extension (fits, jpeg, raw..etc)
         */
        void setCaptureExtension(const char *ext);

        /**
         * @return Return capture extension (fits, jpeg, raw..etc)
         */
        inline char *getCaptureExtension()
        {
            return captureExtention;
        }

        /**
         * @return True if Detector is currently exposing, false otherwise.
         */
        inline bool isCapturing()
        {
            return (FramedCaptureNP.s == IPS_BUSY);
        }

    private:
        /// # of Axis
        int NAxis;
        /// Bytes per Sample
        int BPS;
        double Samplerate;
        double Frequency;
        double Bandwidth;
        double Gain;
        uint8_t *ContinuumBuffer;
        int ContinuumBufferSize;
        uint8_t *TimeDeviationBuffer;
        int TimeDeviationBufferSize;
        uint8_t *SpectrumBuffer;
        int SpectrumBufferSize;
        double captureDuration;
        timespec startCaptureTime;
        char captureExtention[MAXINDIBLOBFMT];

        INumberVectorProperty FramedCaptureNP;
        INumber FramedCaptureN[1];

        INumberVectorProperty DetectorSettingsNP;
        INumber DetectorSettingsN[5];

        ISwitchVectorProperty AbortCaptureSP;
        ISwitch AbortCaptureS[1];

        IBLOB FitsB[3];
        IBLOBVectorProperty FitsBP;

        friend class INDI::Detector;
};
class Detector : public DefaultDevice
{
    public:
        Detector();
        virtual ~Detector();

        enum
        {
            DETECTOR_CAN_ABORT                  = 1 << 0,  /*!< Can the Detector Capture be aborted?  */
            DETECTOR_HAS_SHUTTER                = 1 << 1,  /*!< Does the Detector have a mechanical shutter?  */
            DETECTOR_HAS_COOLER                 = 1 << 2,  /*!< Does the Detector have a cooler and temperature control?  */
            DETECTOR_HAS_CONTINUUM              = 1 << 3,  /*!< Does the Detector support live streaming?  */
            DETECTOR_HAS_SPECTRUM               = 1 << 4,  /*!< Does the Detector support spectrum analysis?  */
            DETECTOR_HAS_TDEV                   = 1 << 5,  /*!< Does the Detector support time deviation correction?  */
            DETECTOR_HAS_STREAMING              = 1 << 6,  /*!< Does the Detector supports streaming?  */
        } DetectorCapability;

        virtual bool initProperties();
        virtual bool updateProperties();
        virtual void ISGetProperties(const char *dev);
        virtual bool ISNewNumber(const char *dev, const char *name, double values[], char *names[], int n);
        virtual bool ISNewSwitch(const char *dev, const char *name, ISState *states, char *names[], int n);
        virtual bool ISNewText(const char *dev, const char *name, char *texts[], char *names[], int n);
        virtual bool ISSnoopDevice(XMLEle *root);

    protected:
        /**
         * @brief GetDetectorCapability returns the Detector capabilities.
         */
        uint32_t GetDetectorCapability() const
        {
            return capability;
        }

        /**
         * @brief SetDetectorCapability Set the Detector capabilities. Al fields must be initilized.
         * @param cap pointer to DetectorCapability struct.
         */
        void SetDetectorCapability(uint32_t cap);

        /**
         * @return True if Detector can abort Capture. False otherwise.
         */
        bool CanAbort()
        {
            return capability & DETECTOR_CAN_ABORT;
        }

        /**
         * @return True if Detector has mechanical or electronic shutter. False otherwise.
         */
        bool HasShutter()
        {
            return capability & DETECTOR_HAS_SHUTTER;
        }

        /**
         * @return True if Detector has cooler and temperature can be controlled. False otherwise.
         */
        bool HasCooler()
        {
            return capability & DETECTOR_HAS_COOLER;
        }

        /**
         * @return  True if the Detector supports continuum blobs. False otherwise.
         */
        bool HasContinuum()
        {
            return capability & DETECTOR_HAS_CONTINUUM;
        }

        /**
         * @return  True if the Detector supports spectrum blobs. False otherwise.
         */
        bool HasSpectrum()
        {
            return capability & DETECTOR_HAS_SPECTRUM;
        }

        /**
         * @return  True if the Detector supports time deviation correction blobs. False otherwise.
         */
        bool HasTimeDeviation()
        {
            return capability & DETECTOR_HAS_TDEV;
        }

        /**
         * @return  True if the Detector supports live video streaming. False otherwise.
         */
        bool HasStreaming()
        {
            return capability & DETECTOR_HAS_STREAMING;
        }

        /**
         * @brief Set Detector temperature
         * @param temperature Detector temperature in degrees celsius.
         * @return 0 or 1 if setting the temperature call to the hardware is successful. -1 if an
         * error is encountered.
         * Return 0 if setting the temperature to the requested value takes time.
         * Return 1 if setting the temperature to the requested value is complete.
         * \note Upon returning 0, the property becomes BUSY. Once the temperature reaches the requested
         * value, change the state to OK.
         * \note This function is not implemented in Detector, it must be implemented in the child class
         */
        virtual int SetTemperature(double temperature);

        /**
         * \brief Start capture from the Detector device
         * \param duration Duration in seconds
         * \return true if OK and Capture will take some time to complete, false on error.
         * \note This function is not implemented in Detector, it must be implemented in the child class
         */
        virtual bool StartCapture(float duration);

        /**
         * \brief Set common capture params
         * \param sr Detector samplerate (in Hz)
         * \param cfreq Capture frequency of the detector (Hz, observed frequency).
         * \param sfreq Sampling frequency of the detector (Hz, electronic speed of the detector).
         * \param bps Bit resolution of a single sample.
         * \param bw Bandwidth (Hz).
         * \param gain Gain of the detector.
         * \return true if OK and Capture will take some time to complete, false on error.
         * \note This function is not implemented in Detector, it must be implemented in the child class
         */
        virtual bool CaptureParamsUpdated(float sr, float freq, float bps, float bw, float gain);

        /**
         * \brief Uploads target Device exposed buffer as FITS to the client. Dervied classes should class
         * this function when an Capture is complete.
         * @param targetDevice device that contains upload capture data
         * \note This function is not implemented in Detector, it must be implemented in the child class
         */
        virtual bool CaptureComplete(DetectorDevice *targetDevice);

        /**
         * \brief Abort ongoing Capture
         * \return true is abort is successful, false otherwise.
         * \note This function is not implemented in Detector, it must be implemented in the child class
         */
        virtual bool AbortCapture();

        /**
         * \brief Setup Detector parameters for the Detector. Child classes call this function to update
         * Detector parameters
         * \param samplerate Detector samplerate (in Hz)
         * \param freq Center frequency of the detector (Hz, observed frequency).
         * \param bps Bit resolution of a single sample.
         * \param bw Detector bandwidth (in Hz).
         * \param gain Detector gain.
         */
         virtual void SetDetectorParams(float samplerate, float freq, float bps, float bw, float gain);

        // DSP API Related functions

        /**
         * @brief Create a histogram
         * @param buf the buffer from which extract the histogram
         * @param out the buffer where to copy the histogram
         * @param buf_len the length in bytes of the input buffer
         * @param histogram_size the size of the histogram
         * @param bits_per_sample can be one of 8,16,32,64 for unsigned types, -32,-64 for floating single and double types
         */
        void Histogram(void *buf, void *out, int buf_len, int histogram_size, int bits_per_sample);

        /**
         * @brief Create a Fourier transform
         * @param buf the buffer from which extract the Fourier transform
         * @param out the buffer where to copy the Fourier transform
         * @param dims the number of dimensions of the input buffer
         * @param sizes the sizes of each dimension
         * @param bits_per_sample can be one of 8,16,32,64 for unsigned types, -32,-64 for floating single and double types
         */
        void FourierTransform(void *buf, void *out, int dims, int *sizes, int bits_per_sample);

        /**
         * @brief Create a Spectrum
         * @param buf the buffer from which extract the spectrum
         * @param out the buffer where to copy the spectrum
         * @param buf_len the length in bytes of the input buffer
         * @param size the size of the spectrum
         * @param bits_per_sample can be one of 8,16,32,64 for unsigned types, -32,-64 for floating single and double types
         */
        void Spectrum(void *buf, void *out, int buf_len, int size, int bits_per_sample);

        /**
         * @brief Convolute
         * @param buf the buffer to convolute
         * @param matrix the convolution matrix
         * @param out the buffer where to copy the convoluted buffer
         * @param dims the number of dimensions of the input buffer
         * @param sizes the sizes of each dimension of the input buffer
         * @param matrix_dims the number of dimensions of the matrix
         * @param matrix_sizes the sizes of each dimension of the matrix
         * @param bits_per_sample can be one of 8,16,32,64 for unsigned types, -32,-64 for floating single and double types
         */
        void Convolution(void *buf, void *matrix, void *out, int dims, int *sizes, int matrix_dims, int *matrix_sizes, int bits_per_sample);

        /**
         * @brief White noise generator
         * @param buf the buffer to fill
         * @param size the size of the input buffer
         * @param bits_per_sample can be one of 8,16,32,64 for unsigned types, -32,-64 for floating single and double types
         */
        void WhiteNoise(void *out, int size, int bits_per_sample);


        /**
         * @brief StartStreaming Start live video streaming
         * @return True if successful, false otherwise.
         */
        virtual bool StartStreaming();

        /**
         * @brief StopStreaming Stop live video streaming
         * @return True if successful, false otherwise.
         */
        virtual bool StopStreaming();
        /**
         * \brief Add FITS keywords to a fits file
         * \param fptr pointer to a valid FITS file.
         * \param targetDevice The target device to extract the keywords from.
         * \param blobIndex The blob index of this FITS (0: continuum, 1: spectrum, 2: timedev).
         * \note In additional to the standard FITS keywords, this function write the following
         * keywords the FITS file:
         * <ul>
         * <li>EXPTIME: Total Capture Time (s)</li>
         * <li>DATAMIN: Minimum value</li>
         * <li>DATAMAX: Maximum value</li>
         * <li>INSTRUME: Detector Name</li>
         * <li>DATE-OBS: UTC start date of observation</li>
         * </ul>
         *
         * To add additional information, override this function in the child class and ensure to call
         * Detector::addFITSKeywords.
         */
        virtual void addFITSKeywords(fitsfile *fptr, DetectorDevice *targetDevice, uint8_t* buf, int len);

        void* sendFITS(DetectorDevice *targetDevice, int bIndex,  uint8_t* buf, int len);
        /** A function to just remove GCC warnings about deprecated conversion */
        void fits_update_key_s(fitsfile *fptr, int type, std::string name, void *p, std::string explanation, int *status);

        /**
         * @brief activeDevicesUpdated Inform children that ActiveDevices property was updated so they can
         * snoop on the updated devices if desired.
         */
        virtual void activeDevicesUpdated() {}

        /**
         * @brief saveConfigItems Save configuration items in XML file.
         * @param fp pointer to file to write to
         * @return True if successful, false otherwise
         */
        virtual bool saveConfigItems(FILE *fp);

        double primaryAperture;
        double primaryFocalLength;
        bool InCapture;

        bool AutoLoop;
        bool SendCapture;
        bool ShowMarker;

        float CaptureTime;

        // Sky Quality
        double MPSAS;

        // Threading
        std::mutex detectorBufferLock;


        std::unique_ptr<StreamManager> Streamer;
        DetectorDevice PrimaryDetector;

        //  We are going to snoop these from a telescope
        INumberVectorProperty EqNP;
        INumber EqN[2];

        ITextVectorProperty ActiveDeviceTP;
        IText ActiveDeviceT[4] {};

        INumber TemperatureN[1];
        INumberVectorProperty TemperatureNP;

        IText FileNameT[1] {};
        ITextVectorProperty FileNameTP;

        ISwitch DatasetS[1];
        ISwitchVectorProperty DatasetSP;

        ISwitch UploadS[3];
        ISwitchVectorProperty UploadSP;

        IText UploadSettingsT[2] {};
        ITextVectorProperty UploadSettingsTP;
        enum
        {
            UPLOAD_DIR,
            UPLOAD_PREFIX
        };

        ISwitch TelescopeTypeS[2];
        ISwitchVectorProperty TelescopeTypeSP;
        enum
        {
            TELESCOPE_PRIMARY
        };

        // FITS Header
        IText FITSHeaderT[2] {};
        ITextVectorProperty FITSHeaderTP;
        enum
        {
            FITS_OBSERVER,
            FITS_OBJECT
        };
        double Lat, Lon, El;
        double RA, Dec;
    private:

        uint32_t capability;

        bool uploadFile(DetectorDevice *targetDevice, const void *fitsData, size_t totalBytes, bool sendCapture, bool saveCapture, int blobindex);
        void getMinMax(double *min, double *max, uint8_t *buf, int len, int bpp);
        int getFileIndex(const char *dir, const char *prefix, const char *ext);

        /////////////////////////////////////////////////////////////////////////////
        /// Misc.
        /////////////////////////////////////////////////////////////////////////////
        friend class StreamManager;
        };
}
