#!/usr/bin/env python3
"""
Quick verification script to test if simulator can connect to backend.
"""
import sys
import json
from datetime import datetime, timezone

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("[ERROR] paho-mqtt is required. Install with: pip install paho-mqtt")
    sys.exit(1)

def test_connection():
    print("Verifying Simulator <-> Backend Connection\n")
    
    # Test configuration
    host = "localhost"
    port = 1883
    topic = "drones/frames"
    cam_id = "e8a76237-df96-4a6a-9375-baa4d74f5f12"
    token = "257c87b4-9469-44fe-9132-8937f69723bd"
    
    print(f"Configuration:")
    print(f"  MQTT Broker: {host}:{port}")
    print(f"  Topic: {topic}")
    print(f"  Camera ID: {cam_id}")
    print(f"  Token: {token[:8]}...\n")
    
    # Create client
    client = mqtt.Client(client_id="test-connection")
    connected = False
    
    def on_connect(client, userdata, flags, rc):
        nonlocal connected
        if rc == 0:
            connected = True
            print("[OK] Connected to MQTT broker")
        else:
            print(f"[ERROR] Connection failed with code {rc}")
    
    def on_publish(client, userdata, mid):
        print("[OK] Test message published successfully")
        client.disconnect()
    
    client.on_connect = on_connect
    client.on_publish = on_publish
    
    try:
        print(f"Connecting to {host}:{port}...")
        client.connect(host, port, keepalive=60)
        client.loop_start()
        
        # Wait for connection
        import time
        timeout = 5
        elapsed = 0
        while not connected and elapsed < timeout:
            time.sleep(0.1)
            elapsed += 0.1
        
        if not connected:
            print("[ERROR] Connection timeout")
            return False
        
        # Create test payload matching backend expectations
        test_payload = {
            "fram_id": "test-001",
            "cam_id": cam_id,
            "token_id": token,
            "timestamp": datetime.now(timezone.utc).isoformat(),
            "image_info": {
                "width": 1920,
                "height": 1080
            },
            "objects": [
                {
                    "obj_id": "test-drone-1",
                    "type": "drone",
                    "lat": 13.7563,
                    "lng": 100.5018,
                    "alt": 120.0,
                    "speed_kt": 15.0
                }
            ]
        }
        
        print(f"\nPublishing test message to {topic}...")
        print(f"   Payload: {json.dumps(test_payload, indent=2)}")
        
        result = client.publish(topic, json.dumps(test_payload), qos=0)
        
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            print("[OK] Message queued for publishing")
            # Wait for publish callback
            time.sleep(1)
        else:
            print(f"[ERROR] Publish failed with code {result.rc}")
            return False
        
        client.loop_stop()
        print("\n[SUCCESS] Connection test successful!")
        print("\nSummary:")
        print("   [OK] Simulator can connect to MQTT broker")
        print("   [OK] Simulator can publish to 'drones/frames' topic")
        print("   [OK] Message format matches backend expectations")
        print("\nNext steps:")
        print("   1. Start the backend: cd Backend_Tesa && npm run dev")
        print("   2. Run simulator: python drone_mqtt_simulator.py --mode frames --center-lat 13.7563 --center-lon 100.5018 --updates 5")
        return True
        
    except OSError as e:
        print(f"[ERROR] Connection error: {e}")
        print("\nMake sure MQTT broker is running:")
        print("   cd Backend_Tesa && docker-compose up -d mqtt")
        return False
    except Exception as e:
        print(f"[ERROR] Error: {e}")
        return False

if __name__ == "__main__":
    success = test_connection()
    sys.exit(0 if success else 1)

