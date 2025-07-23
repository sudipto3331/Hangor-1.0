"""
Network communication manager for AUV system
Handles communication between MacBook, Jetson Nano, and Raspberry Pi
"""

import asyncio
import socket
import json
import logging
from datetime import datetime
from config.settings import AUV_CONFIG

class NetworkManager:
    def __init__(self):
        self.logger = logging.getLogger("NetworkManager")
        self.server_socket = None
        self.client_connections = []
        self.telemetry_socket = None
        self.is_running = False
        
    async def initialize(self):
        """Initialize network communication"""
        try:
            # Setup command server socket
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind(('0.0.0.0', AUV_CONFIG["NETWORK"]["COMMUNICATION_PORT"]))
            self.server_socket.listen(5)
            self.server_socket.setblocking(False)
            
            # Setup telemetry socket
            self.telemetry_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            
            self.is_running = True
            self.logger.info("Network manager initialized successfully")
            
            # Start listening for connections
            asyncio.create_task(self._accept_connections())
            
        except Exception as e:
            self.logger.error(f"Network initialization failed: {e}")
            raise
    
    async def _accept_connections(self):
        """Accept incoming connections from ground station"""
        while self.is_running:
            try:
                # Use asyncio to handle socket operations
                loop = asyncio.get_event_loop()
                client_socket, address = await loop.sock_accept(self.server_socket)
                
                self.client_connections.append(client_socket)
                self.logger.info(f"New connection from {address}")
                
                # Handle client communication
                asyncio.create_task(self._handle_client(client_socket))
                
            except asyncio.CancelledError:
                break
            except Exception as e:
                if self.is_running:
                    self.logger.error(f"Error accepting connections: {e}")
                await asyncio.sleep(0.1)
    
    async def _handle_client(self, client_socket):
        """Handle communication with a connected client"""
        try:
            while self.is_running:
                loop = asyncio.get_event_loop()
                data = await loop.sock_recv(client_socket, 1024)
                
                if not data:
                    break
                    
                # Process received command
                command = json.loads(data.decode())
                response = await self._process_command(command)
                
                # Send response
                await loop.sock_sendall(client_socket, json.dumps(response).encode())
                
        except Exception as e:
            self.logger.error(f"Client communication error: {e}")
        finally:
            client_socket.close()
            if client_socket in self.client_connections:
                self.client_connections.remove(client_socket)
    
    async def _process_command(self, command):
        """Process received commands"""
        cmd_type = command.get("type")
        
        if cmd_type == "start_mission":
            return {"status": "mission_started", "timestamp": datetime.now().isoformat()}
        elif cmd_type == "stop_mission":
            return {"status": "mission_stopped", "timestamp": datetime.now().isoformat()}
        elif cmd_type == "get_status":
            return {"status": "operational", "timestamp": datetime.now().isoformat()}
        else:
            return {"status": "unknown_command", "timestamp": datetime.now().isoformat()}
    
    async def send_telemetry(self, telemetry_data):
        """Send telemetry data to ground station"""
        try:
            data = json.dumps(telemetry_data).encode()
            self.telemetry_socket.sendto(
                data, 
                (AUV_CONFIG["NETWORK"]["GROUND_STATION_IP"], 
                 AUV_CONFIG["NETWORK"]["TELEMETRY_PORT"])
            )
        except Exception as e:
            self.logger.error(f"Telemetry send failed: {e}")
    
    async def shutdown(self):
        """Shutdown network manager"""
        self.is_running = False
        
        if self.server_socket:
            self.server_socket.close()
            
        if self.telemetry_socket:
            self.telemetry_socket.close()
            
        for client in self.client_connections:
            client.close()
            
        self.logger.info("Network manager shutdown complete")