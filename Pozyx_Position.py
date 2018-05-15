#!/usr/bin/env python
"""
This is for Discover lab Iris+ Drone localization
"""
from time import sleep
from time import time as pi_time
from pypozyx import (POZYX_POS_ALG_UWB_ONLY,POZYX_POS_ALG_TRACKING, POZYX_3D, Coordinates,PositionError,DeviceRange, POZYX_SUCCESS, POZYX_ANCHOR_SEL_AUTO,
                     DeviceCoordinates, PozyxSerial, Data,get_first_pozyx_serial_port, SingleRegister, DeviceList)
from pythonosc.udp_client import SimpleUDPClient
import struct
import socket
import os
import traceback

Vector3 = struct.Struct( '<fff' )
ByteValue = struct.Struct('<b')

mocap_loca = [0,0,0]

class ReadyToLocalize(object):
    """Continuously calls the Pozyx positioning function and prints its position."""
    def __init__(self, pozyx, position_publisher,osc_udp_client, anchors, algorithm=POZYX_POS_ALG_UWB_ONLY, dimension=POZYX_3D, height=1000, remote_id=None):
        self.pozyx = pozyx
        self.osc_udp_client = osc_udp_client
        self.anchors = anchors
        self.algorithm = algorithm
        self.dimension = dimension
        self.height = height
        self.remote_id = remote_id
        self.position_publisher = position_publisher
    def setup(self):
        """Sets up the Pozyx for positioning by calibrating its anchor list."""
        self.pozyx.clearDevices(self.remote_id)
        self.setAnchorsManual()
        self.printPublishConfigurationResult()
        sleep(5)
        self.loop()

    def loop(self):
        dataThread = Thread( target = self.__dataThreadFunction, args = (, ))
        dataThread.start()

    def __dataThreadFunction(self):
            """Get position of tag."""
        while True:
            position = Coordinates()
            pos_err = PositionError()

            status_pos = self.pozyx.doPositioning(
                position, self.dimension, self.height, self.algorithm, remote_id=self.remote_id)

            if status_pos == POZYX_SUCCESS:
                status_err = self.pozyx.getPositionError(pos_err,remote_id=self.remote_id)
                if status_err == POZYX_SUCCESS:
                    self.printPublishPosition(position) #display position on remote computer
                    #print(position)
                    if self.position_publisher is not None:
                        self.position_publisher(position)
            else:
                self.printPublishErrorCode("positioning")

            sleep(0.1)

        """                     split line                   """
    def printPublishPosition(self, position):
        """Prints the Pozyx's position and possibly sends it as a OSC packet"""
        network_id = self.remote_id
        if network_id is None:
            network_id = 0
        print("POS ID {}, x(mm): {pos.x} y(mm): {pos.y} z(mm): {pos.z}".format(
            "0x%0.4x" % network_id, pos=position))
        if self.osc_udp_client is not None:
            self.osc_udp_client.send_message(
                "/position", [network_id, int(position.x), int(position.y), int(position.z)])

    def printPublishErrorCode(self, operation):
        """Prints the Pozyx's error and possibly sends it as a OSC packet"""
        error_code = SingleRegister()
        network_id = self.remote_id
        if network_id is None:
            self.pozyx.getErrorCode(error_code)
            print("LOCAL ERROR %s, %s" % (operation, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message("/error", [operation, 0, error_code[0]])
            return
        status = self.pozyx.getErrorCode(error_code, self.remote_id)
        if status == POZYX_SUCCESS:
            print("ERROR %s on ID %s, %s" %
                  (operation, "0x%0.4x" % network_id, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/error", [operation, network_id, error_code[0]])
        else:
            self.pozyx.getErrorCode(error_code)
            print("ERROR %s, couldn't retrieve remote error code, LOCAL ERROR %s" %
                  (operation, self.pozyx.getErrorMessage(error_code)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message("/error", [operation, 0, -1])
            # should only happen when not being able to communicate with a remote Pozyx.

    def setAnchorsManual(self):
        """Adds the manually measured anchors to the Pozyx's device list one for one."""
        status = self.pozyx.clearDevices(self.remote_id)
        for anchor in self.anchors:
            status &= self.pozyx.addDevice(anchor, self.remote_id)
        if len(self.anchors) > 4:
            status &= self.pozyx.setSelectionOfAnchors(POZYX_ANCHOR_SEL_AUTO, len(self.anchors))
        return status

    def printPublishConfigurationResult(self):
        """Prints and potentially publishes the anchor configuration result in a human-readable way."""
        list_size = SingleRegister()

        self.pozyx.getDeviceListSize(list_size, self.remote_id)
        print("List size: {0}".format(list_size[0]))
        if list_size[0] != len(self.anchors):
            self.printPublishErrorCode("configuration")
            return
        device_list = DeviceList(list_size=list_size[0])
        self.pozyx.getDeviceIds(device_list, self.remote_id)
        print("Calibration result:")
        print("Anchors found: {0}".format(list_size[0]))
        print("Anchor IDs: ", device_list)

        for i in range(list_size[0]):
            anchor_coordinates = Coordinates()
            self.pozyx.getDeviceCoordinates(device_list[i], anchor_coordinates, self.remote_id)
            print("ANCHOR, 0x%0.4x, %s" % (device_list[i], str(anchor_coordinates)))
            if self.osc_udp_client is not None:
                self.osc_udp_client.send_message(
                    "/anchor", [device_list[i], int(anchor_coordinates.x), int(anchor_coordinates.y), int(anchor_coordinates.z)])
                sleep(0.025)

def __commThreadFunction(server_socket):
    global mocap_loca
    connection, client_address = server_socket.accept()
    try:
        print('connection from', client_address)
        while True:
            data = connection.recv(1)
            status = ByteValue.unpack(data[0:1])[0]
            #print(status)
            if status==POSITION_REQUEST :
                print("POS: X: {pos[0]} Y: {pos[1]} Z: {pos[2]}".format(pos=mocap_loca))
                send_data = Vector3.pack(mocap_loca[0],mocap_loca[1],mocap_loca[2])
                connection.sendall(send_data)
    except Exception:
        traceback.format_exc()

def receiveRigidBodyFrame(position):
    global mocap_loca
    mocap_loca[0] = position[0]
    mocap_loca[1] = position[1]
    mocap_loca[2] = position[2]

if __name__ == "__main__":
    # shortcut to not have to find out the port yourself
    serial_port = get_first_pozyx_serial_port()
    if serial_port is None:
        print("No Pozyx connected. Check your USB cable or your driver!")
        quit()

    remote_id = 0x6977                 # remote device network ID
    remote = False                   # whether to use a remote device
    if not remote:
        remote_id = None

    use_processing = True             # enable to send position data through OSC
    ip = "127.0.0.1"                   # IP for the OSC UDP
    network_port = 8888                # network port for the OSC UDP
    osc_udp_client = None

    if use_processing:
        osc_udp_client = SimpleUDPClient(ip, network_port)
    # necessary data for calibration, change the IDs and coordinates yourself
    anchors = [DeviceCoordinates(0x6e07, 1, Coordinates(0, 0, -1200)),
               DeviceCoordinates(0x6e51, 1, Coordinates(4780, 0, -1200)),
               DeviceCoordinates(0x6e5e, 1, Coordinates(0, 4070, -1200)),
               DeviceCoordinates(0x6e5f, 1, Coordinates(4780, 4070, -1200))]

    algorithm = POZYX_POS_ALG_TRACKING  # positioning algorithm to use
    dimension = POZYX_3D                # positioning dimension
    height = 0                          # height of device, required in 2.5D positioning

    pozyx = PozyxSerial(serial_port)

    r = ReadyToLocalize(pozyx,receiveRigidBodyFrame, osc_udp_client, anchors, algorithm, dimension, height, remote_id)
    r.setup()

    #broacast the position information
    try:
        os.unlink(server_address)
    except OSError:
        if os.path.exists(server_address):
            raise
    server_socket = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    print('starting up on %s' % server_address)
    server_socket.bind(server_address)
    server_socket.listen(1)

    communicationThread = Thread(target = __commThreadFunction, args = (server_socket, ))
    communicationThread.start()
