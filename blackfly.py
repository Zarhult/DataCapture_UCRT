"""Configure the Blackfly camera
Harshith Mohan Kumar
1/19/25
"""

import PySpin
import logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

NUM_BUFFERS=1


def set_hardware_sync_mode(cam, nodemap):
    '''
    Sets up the camera to use hardware trigger.
    '''
    result = True
    logging.info('*** CONFIGURING TRIGGER ***\n')

    try:
        # Ensure trigger mode off
        # The trigger must be disabled in order to configure whether the source
        # is software or hardware.
        # Get node for trigger
        node_trigger_mode = PySpin.CEnumerationPtr(nodemap.GetNode('TriggerMode'))
        if not PySpin.IsReadable(node_trigger_mode) or not PySpin.IsWritable(node_trigger_mode):
            logging.error('Unable to disable trigger mode (node retrieval). Aborting...')
            return False

        node_trigger_mode_off = node_trigger_mode.GetEntryByName('Off')
        if not PySpin.IsReadable(node_trigger_mode_off):
            logging.error('Unable to disable trigger mode (enum entry retrieval). Aborting...')
            return False

        node_trigger_mode.SetIntValue(node_trigger_mode_off.GetValue())

        logging.info('Trigger mode disabled...')

        # Set TriggerSelector to FrameStart
        # For this example, the trigger selector should be set to frame start.
        # This is the default for most cameras.
        node_trigger_selector= PySpin.CEnumerationPtr(nodemap.GetNode('TriggerSelector'))
        if not PySpin.IsReadable(node_trigger_selector) or not PySpin.IsWritable(node_trigger_selector):
            logging.error('Unable to get trigger selector (node retrieval). Aborting...')
            return False

        node_trigger_selector_framestart = node_trigger_selector.GetEntryByName('FrameStart')
        if not PySpin.IsReadable(node_trigger_selector_framestart):
            logging.error('Unable to set trigger selector (enum entry retrieval). Aborting...')
            return False
        node_trigger_selector.SetIntValue(node_trigger_selector_framestart.GetValue())

        logging.info('Trigger selector set to frame start...')

        # Select trigger source
        # The trigger source must be set to hardware or software while trigger
        # mode is off.
        node_trigger_source = PySpin.CEnumerationPtr(nodemap.GetNode('TriggerSource'))
        if not PySpin.IsReadable(node_trigger_source) or not PySpin.IsWritable(node_trigger_source):
            logging.error('Unable to get trigger source (node retrieval). Aborting...')
            return False

        # Try to set the trigger source to Line3
        node_trigger_source_line3 = node_trigger_source.GetEntryByName('Line3')
        if PySpin.IsReadable(node_trigger_source_line3):
            node_trigger_source.SetIntValue(node_trigger_source_line3.GetValue())
            logging.info('Trigger source set to Line3...')
        else:
            logging.error('Unable to set trigger source to Line3. Aborting...')
            return False

        # Set trigger activation to LevelHigh
        node_trigger_activation = PySpin.CEnumerationPtr(nodemap.GetNode('TriggerActivation'))
        if not PySpin.IsAvailable(node_trigger_activation) or not PySpin.IsWritable(node_trigger_activation):
            logging.error('Unable to get trigger activation (node retrieval). Aborting...')
            return False
        
        node_trigger_activation_level_high = node_trigger_activation.GetEntryByName('RisingEdge')
        if not PySpin.IsReadable(node_trigger_activation_level_high):
            logging.error('Unable to set trigger activation (enum entry retrieval). Aborting...')
            return False

        node_trigger_activation.SetIntValue(node_trigger_activation_level_high.GetValue())
        logging.info('Trigger activation set to RisingEdge...')

        # Set TriggerOverlap to ReadOut
        node_trigger_overlap = PySpin.CEnumerationPtr(nodemap.GetNode('TriggerOverlap'))
        if not PySpin.IsAvailable(node_trigger_overlap) or not PySpin.IsWritable(node_trigger_overlap):
            logging.error('Unable to get trigger overlap (node retrieval). Aborting...')
            return False
        
        node_trigger_overlap_read_out = node_trigger_overlap.GetEntryByName('ReadOut')
        if not PySpin.IsReadable(node_trigger_overlap_read_out):
            logging.error('Unable to set trigger overlap (enum entry retrieval). Aborting...')
            return False

        node_trigger_overlap.SetIntValue(node_trigger_overlap_read_out.GetValue())
        logging.info('Trigger overlap set to ReadOut...')

        # Turn trigger mode on
        # Once the appropriate trigger source has been set, turn trigger mode
        # on in order to retrieve images using the trigger.
        node_trigger_mode_on = node_trigger_mode.GetEntryByName('On')
        if not PySpin.IsReadable(node_trigger_mode_on):
            logging.error('Unable to enable trigger mode (enum entry retrieval). Aborting...')
            return False

        node_trigger_mode.SetIntValue(node_trigger_mode_on.GetValue())
        logging.info('Trigger mode turned back on...')

    except PySpin.SpinnakerException as ex:
        logging.error('Error: %s' % ex)
        return False

    return result

def get_resulting_frame_rate(cam):
    try:
        resulting_frame_rate = cam.AcquisitionResultingFrameRate.GetValue()
        return resulting_frame_rate
    except AttributeError:
        logging.warning("AcquisitionResultingFrameRate not available for this camera model.")
        return None

#def configure_blackfly(cam,exposure,gain,width_to_set,height_to_set,fps):
#    nodemap = cam.GetNodeMap()
#    set_hardware_sync_mode(cam,nodemap)
#
#    # Set up other parameters
#    cam.AcquisitionFrameRateEnable.SetValue(True)
#    cam.AcquisitionFrameRate.SetValue(fps)
#    cam.ExposureAuto.SetValue(PySpin.ExposureAuto_Off)
#    cam.ExposureTime.SetValue(exposure)
#    cam.GainAuto.SetValue(PySpin.GainAuto_Off)
#    cam.Gain.SetValue(gain)
#    # cam.PixelFormat.SetValue(PySpin.PixelFormat_BayerRG8)
#    cam.PixelFormat.SetValue(PySpin.PixelFormat_BayerRG16)
#    processor = PySpin.ImageProcessor()
#    # processor.SetColorProcessing(PySpin.SPINNAKER_COLOR_PROCESSING_ALGORITHM_HQ_LINEAR)
#
#    # # Set width (adjust max value based on sensor)
#    #node_width = PySpin.CIntegerPtr(nodemap.GetNode("Width"))
#    #node_width.SetValue(width_to_set)
#
#    ## # Set height
#    #node_height = PySpin.CIntegerPtr(nodemap.GetNode("Height"))
#    #node_height.SetValue(height_to_set)
#
#    cam_transfer_layer_stream = cam.GetTLStreamNodeMap()
#    buffer_count = PySpin.CIntegerPtr(cam_transfer_layer_stream.GetNode('StreamBufferCountManual'))
#    buffer_count.SetValue(NUM_BUFFERS)
#
#    logging.info("BlackFly Max FPS:", get_resulting_frame_rate(cam))

def configure_blackfly(cam, exposure, gain, width_to_set, height_to_set, fps_unused):
    nodemap = cam.GetNodeMap()

    # Set trigger source to line3 for hardware sync
    cam.TriggerMode.SetValue(PySpin.TriggerMode_Off)
    # Make sure line3 is an input line first
    cam.LineSelector.SetValue(PySpin.LineSelector_Line3)
    cam.LineMode.SetValue(PySpin.LineMode_Input)
    cam.TriggerSelector.SetValue(PySpin.TriggerSelector_FrameStart)
    cam.TriggerSource.SetValue(PySpin.TriggerSource_Line3)
    cam.TriggerActivation.SetValue(PySpin.TriggerActivation_RisingEdge)

    # Free framerate
    cam.AcquisitionMode.SetValue(PySpin.AcquisitionMode_Continuous)
    if cam.AcquisitionFrameRateEnable.GetAccessMode() == PySpin.RW:
        cam.AcquisitionFrameRateEnable.SetValue(False)
    cam.TriggerMode.SetValue(PySpin.TriggerMode_On)

    # Set exposure, gain
    cam.ExposureAuto.SetValue(PySpin.ExposureAuto_Off)
    cam.ExposureTime.SetValue(exposure) # in µs
    cam.GainAuto.SetValue(PySpin.GainAuto_Off)
    cam.Gain.SetValue(gain)
    # 8 bit format for low latency
    cam.PixelFormat.SetValue(PySpin.PixelFormat_BayerRG8)

    cam_transfer_layer_stream = cam.GetTLStreamNodeMap()
    buffer_count = PySpin.CIntegerPtr(cam_transfer_layer_stream.GetNode('StreamBufferCountManual'))
    buffer_count.SetValue(NUM_BUFFERS)

