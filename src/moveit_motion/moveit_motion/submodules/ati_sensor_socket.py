import socket
import struct
import numpy as np
import threading
import time
import errno

class NetFTSensor:
    def __init__(self, ip, port=49152):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setblocking(0)  # Set non-blocking mode
        self.latest_data = None  # To store the latest data
        self.running = False  # Flag to check if data stream is running
        self.data_thread = None  # Thread to continously keep the data being updates parallely

    def send_command(self, command, sample_count):  # NOTE(dhanush) : Function for sending any commands | Refer to : https://www.ati-ia.com/app_content/documents/9620-05-NET%20FT.pdf
        # Page 78 for more commands
        command_packet = struct.pack('!HHI', 0x1234, command, sample_count)
        self.sock.sendto(command_packet, (self.ip, self.port))

    def start_streaming(self, mode=0x0002, sample_count=0):  # NOTE(dhanush) : Command to  streaming the data, 2 modes possible, default mode is the preferred mode
        if not self.running:
            print("Starting streaming...")
            self.running = True
            self.send_command(mode, sample_count)
            self.data_thread = threading.Thread(target=self._update_data)
            self.data_thread.start()
        else:
            print("Streaming is already running.")

    def stop_streaming(self):  # NOTE(dhanush) : Command to stop streaming the data
        if self.running:
            print("Stopping streaming...")
            self.running = False
            self.send_command(0x0000, 0)
            if self.data_thread:
                self.data_thread.join()
            self.sock.close()
        else:
            print("Streaming is not active.")

    def _update_data(self):
        """ Background thread function to update sensor data continuously. """
        while self.running:
            try:
                data, _ = self.sock.recvfrom(36)
                # print(data)
                if data:
                    self.latest_data = NetFTRDTPacket(data)
            except socket.error as e:
                # Handle specific non-blocking socket read errors
                if e.errno not in (errno.EWOULDBLOCK, errno.EAGAIN):
                    print(f"Socket error: {e}")

    def get_most_recent_data(self):
        """ Return the most recently received data packet, or a status message if no data. """
        return self.latest_data if self.latest_data else "No data received yet."

    def close(self):
        """ Close the sensor socket and stop any running processes. """
        self.running = False
        self.sock.close()
        print("Socket closed.")

class NetFTRDTPacket:
    def __init__(self, data):
        unpacked_data = struct.unpack('!IIIIIIIII', data)
        self.rdt_sequence = unpacked_data[0]
        self.ft_sequence = unpacked_data[1]
        self.status = unpacked_data[2]
        self.force_torque_data = np.array(unpacked_data[3:])

    def __str__(self):
        return (f"RDT Seq: {self.rdt_sequence}, FT Seq: {self.ft_sequence}, Status: {self.status}, "
                f"Data: {self.force_torque_data}")

    def get_force_torque_array(self):
        """ Convert force and torque data to a numpy array for easy handling. """
        return self.force_torque_data
    
    def get_rdt_seq(self):  # NOTE(dhanush) : 
        return self.rdt_sequence
    
    def get_ft_seq(self):
        return self.ft_sequence

    def get_status(self):
        return self.status
    

# Example usage
if __name__ == "__main__":
    sensor = NetFTSensor("192.168.10.100")  # IP Address of our sensor
    sensor.start_streaming()
    try:
        while True:
            time.sleep(.002)  # NOTE(dhanush) : This polling delay resulted in matching the publishing rate of the sensor, found empirically - 500Hz
            # NOTE(dhanush) : 500 Hz is set in the webpage hosted by the FT sensor at its IP Address
            packet = sensor.get_most_recent_data()
            if isinstance(packet, NetFTRDTPacket):
                np_array = packet.get_force_torque_array() # Shape is (6,)
                print("RDT SEQ Number", packet.get_rdt_seq())
                print("Numpy Array:", np_array)
                import pdb; pdb.set_trace()
            else:
                print(packet)
    except KeyboardInterrupt:
        print("\nStreaming interrupted by user.")
    finally:
        sensor.stop_streaming()
        sensor.close()

