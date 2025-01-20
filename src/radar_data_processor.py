"""
Data processor module for radar data handling and processing.
"""
import numpy as np
import pandas as pd
from datetime import datetime
import time

class RadarDataProcessor:
    """Process radar data and track targets."""
    
    def __init__(self):
        self.byte_buffer = np.zeros(2**15, dtype='uint8')
        self.byte_buffer_length = 0
        self.MAGIC_WORD = [2, 1, 4, 3, 6, 5, 8, 7]
        self.count_entered = 0
        self.count_exit = 0
        self.temp = {i: [] for i in range(7)}
        
    @staticmethod
    def parse_config_file(config_file_path: str) -> dict:
        """
        Parse the radar configuration file and extract key parameters.
        
        Args:
            config_file_path (str): Path to the configuration file
            
        Returns:
            dict: Configuration parameters including doppler bins, range bins, etc.
        """
        config_parameters = {}
        
        # Read the configuration file
        with open(config_file_path, 'r') as f:
            config = [line.rstrip('\r\n') for line in f]
        
        # Hard coded antenna configuration
        num_rx_ant = 4
        num_tx_ant = 2
        
        for line in config:
            split_words = line.split(" ")
            
            # Profile configuration
            if "profileCfg" in split_words[0]:
                start_freq = int(float(split_words[2]))
                idle_time = int(split_words[3])
                ramp_end_time = float(split_words[5])
                freq_slope_const = float(split_words[8])
                num_adc_samples = int(split_words[10])
                dig_out_sample_rate = int(split_words[11])
                
                # Round to power of 2
                num_adc_samples_round_to2 = 1
                while num_adc_samples > num_adc_samples_round_to2:
                    num_adc_samples_round_to2 *= 2
                    
            # Frame configuration
            elif "frameCfg" in split_words[0]:
                chirp_start_idx = int(split_words[1])
                chirp_end_idx = int(split_words[2])
                num_loops = int(split_words[3])
                num_frames = int(split_words[4])
                frame_periodicity = int(split_words[5])
        
        # Calculate derived parameters
        num_chirps_per_frame = (chirp_end_idx - chirp_start_idx + 1) * num_loops
        config_parameters["numDopplerBins"] = num_chirps_per_frame / num_tx_ant
        config_parameters["numRangeBins"] = num_adc_samples_round_to2
        config_parameters["rangeResolutionMeters"] = (3e8 * dig_out_sample_rate * 1e3) / (
                2 * freq_slope_const * 1e12 * num_adc_samples)
        config_parameters["rangeIdxToMeters"] = (3e8 * dig_out_sample_rate * 1e3) / (
                2 * freq_slope_const * 1e12 * config_parameters["numRangeBins"])
        config_parameters["dopplerResolutionMps"] = 3e8 / (
                2 * start_freq * 1e9 * (idle_time + ramp_end_time) * 1e-6 * 
                config_parameters["numDopplerBins"] * num_tx_ant)
        config_parameters["maxRange"] = (300 * 0.9 * dig_out_sample_rate) / (
                2 * freq_slope_const * 1e3)
        config_parameters["maxVelocity"] = 3e8 / (
                4 * start_freq * 1e9 * (idle_time + ramp_end_time) * 1e-6 * num_tx_ant)
        
        return config_parameters

    def process_frame(self, data_port):
        """Process a single frame of radar data."""
        global byteBuffer, byteBufferLength, out, count_entered, count_exit
        flag = 0
        # Constants
        OBJ_STRUCT_SIZE_BYTES = 12;
        BYTE_VEC_ACC_MAX_SIZE = 2 ** 15;
        MMWDEMO_UART_MSG_POINT_CLOUD_2D = 6;
        MMWDEMO_UART_MSG_TARGET_LIST_2D = 7;
        MMWDEMO_UART_MSG_TARGET_INDEX_2D = 8;
        maxBufferSize = 2 ** 15;
        tlvHeaderLengthInBytes = 8;
        pointLengthInBytes = 16;
        targetLengthInBytes = 68;
        magicWord = [2, 1, 4, 3, 6, 5, 8, 7]

        # Initialize variables
        magicOK = 0  # Checks if magic number has been read
        dataOK = 0  # Checks if the data has been read correctly
        targetDetected = 0  # Checks if a person has been detected
        frameNumber = 0
        targetObj = {}
        pointObj = {}
        out = pd.DataFrame()

        readBuffer = data_port.read(data_port.in_waiting)
        byteVec = np.frombuffer(readBuffer, dtype='uint8')
        byteCount = len(byteVec)

        # Check that the buffer is not full, and then add the data to the buffer
        if (byteBufferLength + byteCount) < maxBufferSize:
            byteBuffer[byteBufferLength:byteBufferLength + byteCount] = byteVec[:byteCount]
            byteBufferLength = byteBufferLength + byteCount

        # Check that the buffer has some data
        if byteBufferLength > 16:

            # Check for all possible locations of the magic word
            possibleLocs = np.where(byteBuffer == magicWord[0])[0]

            # Confirm that is the beginning of the magic word and store the index in startIdx
            startIdx = []
            for loc in possibleLocs:
                check = byteBuffer[loc:loc + 8]
                if np.all(check == magicWord):
                    startIdx.append(loc)

            # Check that startIdx is not empty
            if startIdx:

                # Remove the data before the first start index
                if startIdx[0] > 0 and startIdx[0] < byteBufferLength:
                    byteBuffer[:byteBufferLength - startIdx[0]] = byteBuffer[startIdx[0]:byteBufferLength]
                    byteBuffer[byteBufferLength - startIdx[0]:] = np.zeros(len(byteBuffer[byteBufferLength - startIdx[0]:]),
                                                                        dtype='uint8')
                    byteBufferLength = byteBufferLength - startIdx[0]

                # Check that there have no errors with the byte buffer length
                if byteBufferLength < 0:
                    byteBufferLength = 0

                # word array to convert 4 bytes to a 32 bit number
                word = [1, 2 ** 8, 2 ** 16, 2 ** 24]

                # Read the total packet length
                totalPacketLen = np.matmul(byteBuffer[20:20 + 4], word)

                # Check that all the packet has been read
                if (byteBufferLength >= totalPacketLen) and (byteBufferLength != 0):
                    magicOK = 1

        # If magicOK is equal to 1 then process the message
        if magicOK:
            # word array to convert 4 bytes to a 32 bit number
            word = [1, 2 ** 8, 2 ** 16, 2 ** 24]

            # Initialize the pointer index
            idX = 0

            # Read the header
            # Read the header
            magicNumber = byteBuffer[idX:idX + 8]
            idX += 8
            version = format(np.matmul(byteBuffer[idX:idX + 4], word), 'x')
            idX += 4
            platform = format(np.matmul(byteBuffer[idX:idX + 4], word), 'x')
            idX += 4
            timeStamp = np.matmul(byteBuffer[idX:idX + 4], word)
            idX += 4
            totalPacketLen = np.matmul(byteBuffer[idX:idX + 4], word)
            idX += 4
            frameNumber = np.matmul(byteBuffer[idX:idX + 4], word)
            idX += 4
            subFrameNumber = np.matmul(byteBuffer[idX:idX + 4], word)
            idX += 4
            chirpMargin = np.matmul(byteBuffer[idX:idX + 4], word)
            idX += 4
            frameMargin = np.matmul(byteBuffer[idX:idX + 4], word)
            idX += 4
            uartSentTime = np.matmul(byteBuffer[idX:idX + 4], word)
            idX += 4
            trackProcessTime = np.matmul(byteBuffer[idX:idX + 4], word)
            idX += 4

            word = [1, 2 ** 8]

            numTLVs = np.matmul(byteBuffer[idX:idX + 2], word)
            idX += 2
            checksum = np.matmul(byteBuffer[idX:idX + 2], word)
            idX += 2

            # Read the TLV messages
            for tlvIdx in range(numTLVs):
                # word array to convert 4 bytes to a 32 bit number
                word = [1, 2 ** 8, 2 ** 16, 2 ** 24]

                # Initialize the tlv type
                tlv_type = 0

                try:
                    # Check the header of the TLV message
                    tlv_type = np.matmul(byteBuffer[idX:idX + 4], word)
                    idX += 4
                    tlv_length = np.matmul(byteBuffer[idX:idX + 4], word)
                    idX += 4
                except:
                    pass

                # Read the data depending on the TLV message
                if tlv_type == MMWDEMO_UART_MSG_POINT_CLOUD_2D:
                    # word array to convert 4 bytes to a 16 bit number
                    word = [1, 2 ** 8, 2 ** 16, 2 ** 24]

                    # Calculate the number of detected points
                    numInputPoints = (tlv_length - tlvHeaderLengthInBytes) // pointLengthInBytes

                    # Initialize the arrays
                    rangeVal = np.zeros(numInputPoints, dtype=np.float32)
                    azimuth = np.zeros(numInputPoints, dtype=np.float32)
                    dopplerVal = np.zeros(numInputPoints, dtype=np.float32)
                    snr = np.zeros(numInputPoints, dtype=np.float32)

                    for objectNum in range(numInputPoints):
                        # Read the data for each object
                        rangeVal[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        azimuth[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        dopplerVal[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        snr[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4

                        # Store the data in the detObj dictionary
                    pointObj = {"numObj": numInputPoints, "range": rangeVal, "azimuth": azimuth,
                                "doppler": dopplerVal, "snr": snr}

                    dataOK = 1

                elif tlv_type == MMWDEMO_UART_MSG_TARGET_LIST_2D:

                    # word array to convert 4 bytes to a 16 bit number
                    word = [1, 2 ** 8, 2 ** 16, 2 ** 24]

                    # Calculate the number of target points
                    numTargetPoints = (tlv_length - tlvHeaderLengthInBytes) // targetLengthInBytes
                    # print(numTargetPoints)
                    # Initialize the arrays
                    targetId = np.zeros(numTargetPoints, dtype=np.uint32)
                    posX = np.zeros(numTargetPoints, dtype=np.float32)
                    posY = np.zeros(numTargetPoints, dtype=np.float32)
                    velX = np.zeros(numTargetPoints, dtype=np.float32)
                    velY = np.zeros(numTargetPoints, dtype=np.float32)

                    accX = np.zeros(numTargetPoints, dtype=np.float32)
                    accY = np.zeros(numTargetPoints, dtype=np.float32)
                    EC = np.zeros((3, 3, numTargetPoints), dtype=np.float32)  # Error covariance matrix
                    G = np.zeros(numTargetPoints, dtype=np.float32)  # Gain

                    for objectNum in range(numTargetPoints):
                        # Read the data for each object
                        targetId[objectNum] = np.matmul(byteBuffer[idX:idX + 4], word)
                        idX += 4
                        posX[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        posY[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        velX[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        velY[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        accX[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        accY[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        EC[0, 0, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        EC[0, 1, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        EC[0, 2, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        EC[1, 0, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        EC[1, 1, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        EC[1, 2, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        EC[2, 0, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        EC[2, 1, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        EC[2, 2, objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                        G[objectNum] = byteBuffer[idX:idX + 4].view(dtype=np.float32)
                        idX += 4
                    # timestamp data
                    dateTimeObj = time.time()
                    ID = 'IWR-A'
                    for i in range(0, len(targetId)):
                        if -0.5 <= posX[i] <= 0.5:
                            temp[targetId[i]].append(posX[i])
                            if frameNumber % 20 == 0:
                                zero_crossing_site = np.where(np.diff(np.sign(temp[targetId[i]]) >= 0))[0]
                                if zero_crossing_site.size != 0:
                                    if temp[targetId[i]][zero_crossing_site[0]] < 0:
                                        count_entered = count_entered + 1  # person has entered
                                        temp[targetId[i]] = []
                                        break
                                    else:
                                        count_exit = count_exit + 1  # person has exited
                                        temp[targetId[i]] = []
                                        break
                    # print(count_entered)
                    # print(count_exit)
                    # Store the data in the detObj dictionary
                    targetObj = {'frameNumber': frameNumber, 'TimeStamp': dateTimeObj, 'device_ID': ID,
                                'target_ID': targetId, 'posX': posX, 'posY': posY, 'velX': velX, 'velY': velY,
                                'Num_Targets': len(targetId), 'Ppl_entered': count_entered, 'Ppl_exited': count_exit}
                    out = pd.DataFrame(targetObj)
                    targetDetected = 1

                elif tlv_type == MMWDEMO_UART_MSG_TARGET_INDEX_2D:
                    # Calculate the length of the index message
                    numIndices = tlv_length - tlvHeaderLengthInBytes
                    indices = byteBuffer[idX:idX + numIndices]
                    idX += numIndices

            # Remove already processed data
            if idX > 0:
                shiftSize = totalPacketLen
                byteBuffer[:byteBufferLength - shiftSize] = byteBuffer[shiftSize:byteBufferLength]
                byteBuffer[byteBufferLength - shiftSize:] = np.zeros(len(byteBuffer[byteBufferLength - shiftSize:]),
                                                                    dtype='uint8')
                byteBufferLength = byteBufferLength - shiftSize

                # Check that there are no errors with the buffer length
                if byteBufferLength < 0:
                    byteBufferLength = 0

        return dataOK, targetDetected, frameNumber, out, targetObj, pointObj
