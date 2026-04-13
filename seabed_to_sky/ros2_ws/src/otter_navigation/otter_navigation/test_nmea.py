#!/usr/bin/env python3
"""
Standalone Testing Script: NMEA TCP Connection Tester

This script connects to the autopilot via TCP and listens for incoming
NMEA sentences. Use this to verify the odometry_to_nmea converter is working.

It can also act as a TCP server (if you want to test sending data from the script).

Usage:
  # Listen for incoming NMEA data from the converter
  python3 test_nmea_connection.py --mode listen --ip 192.168.1.100 --port 5000
  
  # Act as a server and wait for connections
  python3 test_nmea_connection.py --mode server --port 5000
  
  # Send test NMEA sentences to the autopilot
  python3 test_nmea_connection.py --mode send --ip 192.168.1.100 --port 5000
"""

import socket
import argparse
import time
import math
from datetime import datetime


class NMEATester:
    """Test NMEA connectivity and data."""
    
    def __init__(self, ip="127.0.0.1", port=5000):
        self.ip = ip
        self.port = port
        self.sock = None
    
    def listen(self):
        """Connect to autopilot and listen for incoming NMEA data."""
        print(f"\n=== NMEA Connection Listener ===")
        print(f"Connecting to {self.ip}:{self.port}...")
        
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.ip, self.port))
            print(f"✓ Connected successfully!")
            print(f"Listening for NMEA sentences...\n")
            
            buffer = ""
            count = 0
            
            while True:
                try:
                    data = self.sock.recv(1024).decode('utf-8')
                    if not data:
                        print("Connection closed by remote host")
                        break
                    
                    buffer += data
                    
                    # Process complete NMEA sentences (end with \r\n)
                    while '\r\n' in buffer:
                        sentence, buffer = buffer.split('\r\n', 1)
                        sentence = sentence.strip()
                        
                        if sentence:
                            count += 1
                            self._parse_nmea(sentence, count)
                
                except KeyboardInterrupt:
                    print("\n\n✓ Listener stopped by user")
                    break
                except Exception as e:
                    print(f"Error receiving data: {e}")
                    break
        
        except socket.error as e:
            print(f"✗ Connection failed: {e}")
        
        finally:
            if self.sock:
                self.sock.close()
                print("Connection closed")
    
    def server(self):
        """Act as a TCP server and listen for incoming connections."""
        print(f"\n=== NMEA TCP Server ===")
        print(f"Listening on {self.ip}:{self.port}...")
        
        try:
            server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server_sock.bind((self.ip, self.port))
            server_sock.listen(1)
            print(f"✓ Server started, waiting for connections...\n")
            
            while True:
                try:
                    client_sock, client_addr = server_sock.accept()
                    print(f"✓ Client connected: {client_addr[0]}:{client_addr[1]}")
                    
                    buffer = ""
                    count = 0
                    
                    while True:
                        data = client_sock.recv(1024).decode('utf-8')
                        if not data:
                            break
                        
                        buffer += data
                        
                        while '\r\n' in buffer:
                            sentence, buffer = buffer.split('\r\n', 1)
                            sentence = sentence.strip()
                            
                            if sentence:
                                count += 1
                                self._parse_nmea(sentence, count)
                    
                    print(f"Client disconnected\n")
                    client_sock.close()
                
                except KeyboardInterrupt:
                    print("\n\n✓ Server stopped by user")
                    break
        
        except socket.error as e:
            print(f"✗ Server error: {e}")
        
        finally:
            server_sock.close()
    
    def send(self):
        """Send test NMEA sentences to the autopilot."""
        print(f"\n=== NMEA Test Sender ===")
        print(f"Connecting to {self.ip}:{self.port}...")
        
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.ip, self.port))
            print(f"✓ Connected successfully!")
            print(f"Sending test NMEA sentences...\n")
            
            # Generate test positions (in a circle around origin)
            origin_lat = 55.7637
            origin_lon = 12.5675
            radius = 100  # meters
            duration = 20  # seconds
            
            for i in range(duration * 10):  # 10 Hz
                angle = (i / (duration * 10)) * 2 * math.pi
                x = radius * math.cos(angle)
                y = radius * math.sin(angle)
                heading = angle
                
                lat, lon = self._xy_to_latlon(x, y, origin_lat, origin_lon)
                gga = self._create_gga(lat, lon)
                rmc = self._create_rmc(lat, lon, heading)
                
                print(f"[{i+1}] Sending: {gga}")
                print(f"         {rmc}\n")
                
                self.sock.sendall((gga + '\r\n').encode('utf-8'))
                self.sock.sendall((rmc + '\r\n').encode('utf-8'))
                
                time.sleep(0.1)  # 100ms for 10 Hz
            
            print(f"✓ Test completed")
        
        except socket.error as e:
            print(f"✗ Connection failed: {e}")
        
        finally:
            if self.sock:
                self.sock.close()
    
    def _parse_nmea(self, sentence, count):
        """Parse and display NMEA sentence."""
        parts = sentence.split(',')
        
        if sentence.startswith('$GPGGA'):
            try:
                time_str = parts[1]
                lat = self._dms_to_decimal(parts[2], parts[3])
                lon = self._dms_to_decimal(parts[4], parts[5])
                quality = parts[6]
                sats = parts[7]
                hdop = parts[8]
                
                quality_str = {
                    '0': 'No fix',
                    '1': 'GPS fix',
                    '2': 'DGPS fix'
                }.get(quality, 'Unknown')
                
                print(f"[{count}] GGA: {time_str} | Lat: {lat:.6f}° | Lon: {lon:.6f}° | "
                      f"Quality: {quality_str} ({sats} sats, HDOP {hdop})")
            except:
                print(f"[{count}] GGA (parsing error): {sentence}")
        
        elif sentence.startswith('$GPRMC'):
            try:
                time_str = parts[1]
                status = parts[2]
                lat = self._dms_to_decimal(parts[3], parts[4])
                lon = self._dms_to_decimal(parts[5], parts[6])
                sog = parts[7]
                cog = parts[8]
                
                status_str = 'Active' if status == 'A' else 'Invalid'
                
                print(f"[{count}] RMC: {time_str} | {status_str} | Lat: {lat:.6f}° | Lon: {lon:.6f}° | "
                      f"COG: {cog}° | SOG: {sog} knots")
            except:
                print(f"[{count}] RMC (parsing error): {sentence}")
        
        else:
            print(f"[{count}] Unknown: {sentence}")
    
    @staticmethod
    def _dms_to_decimal(dms_str, direction):
        """Convert DMS (degrees, minutes, seconds) to decimal."""
        try:
            if direction in ['N', 'S']:
                deg = int(dms_str[:2])
                min_sec = float(dms_str[2:])
            else:  # E, W
                deg = int(dms_str[:3])
                min_sec = float(dms_str[3:])
            
            decimal = deg + min_sec / 60
            
            if direction in ['S', 'W']:
                decimal = -decimal
            
            return decimal
        except:
            return 0.0
    
    @staticmethod
    def _xy_to_latlon(x, y, origin_lat, origin_lon):
        """Convert ENU to lat/lon."""
        m_per_deg_lat = 111320.0
        m_per_deg_lon = 111320.0 * math.cos(math.radians(origin_lat))
        
        delta_lat = x / m_per_deg_lat
        delta_lon = y / m_per_deg_lon
        
        return origin_lat + delta_lat, origin_lon + delta_lon
    
    @staticmethod
    def _create_gga(lat, lon):
        """Create test GGA sentence."""
        now = datetime.utcnow()
        utc_time = now.strftime('%H%M%S.%f')[:-3]
        
        lat_deg = int(lat)
        lat_min = (lat - lat_deg) * 60
        lat_str = f"{lat_deg:02d}{lat_min:07.4f}"
        lat_dir = 'N' if lat >= 0 else 'S'
        
        lon_deg = int(lon)
        lon_min = (lon - lon_deg) * 60
        lon_str = f"{lon_deg:03d}{lon_min:07.4f}"
        lon_dir = 'E' if lon >= 0 else 'W'
        
        sentence = f"GPGGA,{utc_time},{lat_str},{lat_dir},{lon_str},{lon_dir},1,12,0.9,545.4,M,46.9,M,,"
        
        checksum = 0
        for char in sentence:
            checksum ^= ord(char)
        
        return f"${sentence}*{checksum:02X}"
    
    @staticmethod
    def _create_rmc(lat, lon, heading):
        """Create test RMC sentence."""
        now = datetime.utcnow()
        utc_time = now.strftime('%H%M%S.%f')[:-3]
        date_str = now.strftime('%d%m%y')
        
        lat_deg = int(lat)
        lat_min = (lat - lat_deg) * 60
        lat_str = f"{lat_deg:02d}{lat_min:07.4f}"
        lat_dir = 'N' if lat >= 0 else 'S'
        
        lon_deg = int(lon)
        lon_min = (lon - lon_deg) * 60
        lon_str = f"{lon_deg:03d}{lon_min:07.4f}"
        lon_dir = 'E' if lon >= 0 else 'W'
        
        cog = math.degrees(heading) % 360
        cog_str = f"{cog:06.1f}"
        
        sentence = f"GPRMC,{utc_time},A,{lat_str},{lat_dir},{lon_str},{lon_dir},0.0,{cog_str},{date_str},0.0,W"
        
        checksum = 0
        for char in sentence:
            checksum ^= ord(char)
        
        return f"${sentence}*{checksum:02X}"


def main():
    parser = argparse.ArgumentParser(
        description='NMEA TCP Connection Tester',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Listen for incoming data
  python3 test_nmea_connection.py --mode listen --ip 192.168.1.100 --port 5000
  
  # Act as TCP server
  python3 test_nmea_connection.py --mode server --port 5000
  
  # Send test data
  python3 test_nmea_connection.py --mode send --ip 192.168.1.100 --port 5000
        """
    )
    
    parser.add_argument('--mode', choices=['listen', 'server', 'send'],
                       default='listen',
                       help='Mode: listen (connect and listen), server (open port), or send (send test data)')
    parser.add_argument('--ip', default='127.0.0.1',
                       help='IP address (default: 127.0.0.1)')
    parser.add_argument('--port', type=int, default=5000,
                       help='TCP port (default: 5000)')
    
    args = parser.parse_args()
    
    tester = NMEATester(ip=args.ip, port=args.port)
    
    if args.mode == 'listen':
        tester.listen()
    elif args.mode == 'server':
        tester.server()
    elif args.mode == 'send':
        tester.send()


if __name__ == '__main__':
    main()