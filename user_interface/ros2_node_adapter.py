#!/usr/bin/env python3
"""
Simple ROS2 Node Adapter for Hospital Medicine System
Provides HTTP API for your ROS2 node to pull orders and report status
"""

from fastapi import FastAPI, HTTPException
from fastapi.responses import JSONResponse
import requests
import json
import yaml
import uvicorn
import logging
from typing import Dict, Any
import asyncio

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("ros2_adapter")

# Create adapter API
adapter_app = FastAPI(title="ROS2 Node Adapter", version="1.0.0")

# Configuration
MAIN_SERVER_URL = "http://localhost:8001"
current_orders = {}  # Track current orders

@adapter_app.get("/api/order/next")
async def get_next_order():
    """Pull next order for ROS2 node"""
    try:
        # Check for pending prescriptions
        response = requests.get(f"{MAIN_SERVER_URL}/api/prescription/", timeout=10)
        prescriptions = response.json()
        
        # Find oldest pending prescription
        pending = [p for p in prescriptions if p.get('status') == 'pending']
        if not pending:
            return JSONResponse(status_code=204, content={})  # No content
        
        # Get oldest one
        oldest = min(pending, key=lambda x: x.get('id', 0))
        
        # Convert to order format
        order = convert_prescription_to_order(oldest)
        
        # Mark as processing
        requests.put(
            f"{MAIN_SERVER_URL}/api/prescription/{oldest['id']}/status",
            json={"status": "processing"},
            timeout=10
        )
        
        # Track the order
        current_orders[order['order_id']] = {
            'prescription_id': oldest['id'],
            'status': 'processing'
        }
        
        # Return YAML format
        yaml_content = yaml.safe_dump(order, allow_unicode=True)
        return {
            "order": order,
            "yaml": yaml_content
        }
        
    except Exception as e:
        logger.error(f"Error getting order: {e}")
        raise HTTPException(status_code=500, detail=str(e))

@adapter_app.post("/api/order/progress")
async def report_progress(payload: Dict[str, Any]):
    """Receive progress reports from ROS2 node"""
    order_id = payload.get('order_id')
    stage = payload.get('stage')
    message = payload.get('message')
    
    logger.info(f"Order {order_id} progress: [{stage}] {message}")
    
    if order_id in current_orders:
        current_orders[order_id]['last_update'] = stage
        current_orders[order_id]['message'] = message
    
    return {"status": "received"}

@adapter_app.post("/api/order/complete")
async def report_complete(payload: Dict[str, Any]):
    """Receive completion reports from ROS2 node"""
    order_id = payload.get('order_id')
    status = payload.get('status')  # success / failed
    details = payload.get('details', '')
    
    logger.info(f"Order {order_id} completed: {status} - {details}")
    
    if order_id in current_orders:
        order_info = current_orders[order_id]
        prescription_id = order_info['prescription_id']
        
        # Update prescription status
        final_status = 'completed' if status == 'success' else 'failed'
        
        try:
            response = requests.put(
                f"{MAIN_SERVER_URL}/api/prescription/{prescription_id}/status",
                json={"status": final_status},
                timeout=10
            )
            if response.status_code == 200:
                logger.info(f"Prescription {prescription_id} status updated to: {final_status}")
            else:
                logger.error(f"Failed to update prescription status: {response.status_code}")
        except Exception as e:
            logger.error(f"Error updating prescription status: {e}")
        
        # Remove from tracking
        del current_orders[order_id]
    
    return {"status": "completed"}

@adapter_app.get("/api/order/status")
async def get_status():
    """Get current order status"""
    return {
        "current_orders": current_orders,
        "total_processing": len(current_orders)
    }

def convert_prescription_to_order(prescription: Dict[str, Any]) -> Dict[str, Any]:
    """Convert prescription to order format"""
    order = {
        "order_id": f"{prescription['id']:06d}",
        "prescription_id": prescription['id'],
        "patient_name": prescription.get('patient_name', 'Unknown'),
        "medicine": []
    }
    
    medicines = prescription.get('medicines', [])
    for i, med in enumerate(medicines):
        medicine_item = {
            "name": med.get('name', 'Unknown'),
            "amount": med.get('amount', 1),
            "locate": generate_location(i),
            "prompt": determine_prompt(med.get('name', ''))
        }
        order["medicine"].append(medicine_item)
    
    return order

def generate_location(index: int) -> list:
    """Generate medicine location [row, col]"""
    row = (index // 5) + 1  # Max 5 per row
    col = (index % 5) + 1
    return [row, col]

def determine_prompt(medicine_name: str) -> str:
    """Determine medicine type based on name"""
    name_lower = medicine_name.lower()
    
    if any(word in name_lower for word in ['capsule', 'cap']):
        return 'capsule'
    elif any(word in name_lower for word in ['box', 'package']):
        return 'white_circle_box'
    else:
        return 'tablet'  # Default

@adapter_app.get("/")
async def root():
    return {
        "message": "ROS2 Node Adapter",
        "status": "running",
        "endpoints": {
            "get_order": "/api/order/next",
            "report_progress": "/api/order/progress",
            "report_complete": "/api/order/complete",
            "get_status": "/api/order/status"
        }
    }

if __name__ == "__main__":
    print("Starting ROS2 Node Adapter...")
    print("Adapter API: http://localhost:8002")
    print("Main Server: http://localhost:8001")
    print("\nYour ROS2 node should:")
    print("- Pull orders from: http://localhost:8002/api/order/next")
    print("- Report progress to: http://localhost:8002/api/order/progress")
    print("- Report completion to: http://localhost:8002/api/order/complete")
    
    uvicorn.run(adapter_app, host="0.0.0.0", port=8002)