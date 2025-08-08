#!/usr/bin/env python3
"""
Start the complete working hospital medicine management system
"""

import subprocess
import sys
import time
import signal
import os
from pathlib import Path

class WorkingSystemLauncher:
    def __init__(self):
        self.processes = []
        self.base_dir = Path(__file__).parent
        
    def check_requirements(self):
        """Check if required packages are installed"""
        required_packages = ['fastapi', 'uvicorn', 'sqlalchemy', 'pydantic', 'requests', 'pyyaml']
        missing = []
        
        for package in required_packages:
            try:
                __import__(package)
            except ImportError:
                missing.append(package)
        
        if missing:
            print(f"Missing packages: {', '.join(missing)}")
            print("Install with: pip3 install " + " ".join(missing))
            return False
        
        return True
    
    def start_main_server(self):
        """Start the main hospital system server"""
        print("Starting main hospital system server...")
        
        try:
            process = subprocess.Popen([
                sys.executable, str(self.base_dir / 'simple_working_system.py')
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            
            self.processes.append(process)
            time.sleep(3)  # Wait for startup
            
            if process.poll() is None:
                print("Main server started successfully (port 8001)")
                return True
            else:
                print("Failed to start main server")
                return False
                
        except Exception as e:
            print(f"Error starting main server: {e}")
            return False
    
    def start_ros2_adapter(self):
        """Start the ROS2 adapter"""
        print("Starting ROS2 adapter...")
        
        try:
            process = subprocess.Popen([
                sys.executable, str(self.base_dir / 'ros2_node_adapter.py')
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            
            self.processes.append(process)
            time.sleep(2)  # Wait for startup
            
            if process.poll() is None:
                print("ROS2 adapter started successfully (port 8002)")
                return True
            else:
                print("Failed to start ROS2 adapter")
                return False
                
        except Exception as e:
            print(f"Error starting ROS2 adapter: {e}")
            return False
    
    def print_system_info(self):
        """Print system information"""
        print("\n" + "=" * 80)
        print("Hospital Medicine Management System - RUNNING")
        print("=" * 80)
        
        print("\nWEB INTERFACES:")
        print("- Medicine Management: http://localhost:8001/integrated_medicine_management.html")
        print("- Doctor Interface: http://localhost:8001/doctor.html")
        print("- Prescription Management: http://localhost:8001/Prescription.html")
        print("- API Documentation: http://localhost:8001/docs")
        
        print("\nROS2 INTEGRATION:")
        print("- Adapter API: http://localhost:8002")
        print("- Order Pull: http://localhost:8002/api/order/next")
        print("- Status Report: http://localhost:8002/api/order/status")
        
        print("\nYOUR ROS2 NODE ENVIRONMENT:")
        print("export ORDER_BASE_URL='http://127.0.0.1:8002'")
        print("export ORDER_PULL_URL='http://127.0.0.1:8002/api/order/next'")
        print("export ORDER_PULL_INTERVAL='3'")
        print("export ORDER_PROGRESS_PATH='/api/order/progress'")
        print("export ORDER_COMPLETE_PATH='/api/order/complete'")
        
        print("\nYAML ORDER FORMAT:")
        print("order_id: \"000001\"")
        print("prescription_id: 1")
        print("patient_name: \"John Doe\"")
        print("medicine:")
        print("  - name: Aspirin")
        print("    amount: 10")
        print("    locate: [1, 2]")
        print("    prompt: tablet")
        
        print("\nSYSTEM STATUS: STABLE AND READY")
        print("Press Ctrl+C to stop the system")
        print("=" * 80)
    
    def shutdown(self):
        """Shutdown all processes"""
        print("\nShutting down system...")
        
        for i, process in enumerate(self.processes):
            try:
                process.terminate()
                process.wait(timeout=5)
                print(f"Process {i+1} stopped")
            except subprocess.TimeoutExpired:
                process.kill()
                print(f"Process {i+1} force stopped")
            except Exception as e:
                print(f"Error stopping process {i+1}: {e}")
        
        print("System shutdown complete")
    
    def run(self):
        """Run the complete system"""
        # Set up signal handlers
        signal.signal(signal.SIGINT, lambda s, f: self.shutdown())
        signal.signal(signal.SIGTERM, lambda s, f: self.shutdown())
        
        print("Hospital Medicine Management System Launcher")
        print("=" * 50)
        
        # Check requirements
        if not self.check_requirements():
            print("Please install missing packages first")
            return False
        
        # Start main server
        if not self.start_main_server():
            print("Failed to start main server")
            return False
        
        # Start ROS2 adapter
        if not self.start_ros2_adapter():
            print("Failed to start ROS2 adapter")
            self.shutdown()
            return False
        
        # Print system info
        self.print_system_info()
        
        # Keep running
        try:
            while True:
                time.sleep(1)
                # Check if processes are still running
                for process in self.processes:
                    if process.poll() is not None:
                        print("A process has stopped unexpectedly")
                        self.shutdown()
                        return False
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()
        
        return True

def main():
    launcher = WorkingSystemLauncher()
    success = launcher.run()
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()