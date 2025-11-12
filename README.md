# Drone MQTT Simulator

Python simulator for generating drone detection data and publishing to MQTT using the new frame structure.

## New Structure Format

The simulator now publishes data in the following format:

```json
{
  "fram_id": "string",
  "cam_id": "string",
  "token_id": {
    "camera_info": {
      "name": "string",
      "sort": "string",
      "location": "string",
      "institute": "string"
    }
  },
  "timestamp": "ISO 8601 string",
  "image_info": {
    "width": 1920,
    "height": 1080
  },
  "objects": [
    {
      "obj_id": "string",
      "type": "string",
      "lat": 0.0,
      "lng": 0.0,
      "alt": 0.0,
      "speed_kt": 0.0
    }
  ]
}
```

## Installation

1. Install Python dependencies:
```bash
pip install paho-mqtt
```

## Usage

### Basic Example

```bash
python drone_mqtt_simulator.py \
    --mode frames \
    --host localhost \
    --topic drones/frames \
    --center-lat 13.7563 \
    --center-lon 100.5018 \
    --num-drones 2 \
    --interval-s 0.5 \
    --radius-m 120 \
    --cam-id camera-1 \
    --camera-name "Test Camera" \
    --camera-sort outdoor \
    --camera-location Bangkok \
    --camera-institute TESA
```

### Parameters

**Connection:**
- `--host`: MQTT broker hostname (default: localhost)
- `--port`: MQTT broker port (default: 1883)
- `--topic`: MQTT topic (default: drones/frames)
- `--qos`: MQTT QoS level 0-2 (default: 0)

**Scene:**
- `--center-lat`: Latitude of scene center (required)
- `--center-lon`: Longitude of scene center (required)
- `--interval-s`: Seconds between frames (default: 0.5)
- `--radius-m`: Orbit radius in meters (default: 120.0)
- `--altitude-m`: Base altitude in meters (default: 120.0)

**Frames Mode:**
- `--num-drones`: Number of objects per frame (default: 1)
- `--speed-range-kt`: Speed range in knots [MIN MAX] (default: 6.0 24.0)
- `--cam-id`: Camera identifier (default: camera-1)
- `--camera-name`: Camera name (default: Test Camera)
- `--camera-sort`: Camera sort/type (default: outdoor)
- `--camera-location`: Camera location (default: Bangkok)
- `--camera-institute`: Camera institute (default: TESA)
- `--noise-level-m`: GPS jitter in meters (default: 3.0)
- `--miss-rate`: Probability to miss detection (default: 0.10)
- `--false-positive-rate`: Probability for false positives (default: 0.03)

## Speed Units

- The simulator uses **knots (kt)** for speed in the published data
- Internal calculations use m/s for physics simulation
- Conversion: 1 knot = 0.514444 m/s

## Example Output

Each frame will contain:
- `fram_id`: Sequential frame identifier (as string)
- `cam_id`: Camera identifier
- `token_id.camera_info`: Camera metadata
- `timestamp`: ISO 8601 timestamp
- `image_info`: Image dimensions
- `objects`: Array of detected objects with:
  - `obj_id`: Object identifier
  - `type`: Object type (optional)
  - `lat`, `lng`: Position coordinates
  - `alt`: Altitude in meters
  - `speed_kt`: Speed in knots

## Integration with Backend

This simulator is designed to work with the Backend_Tesa system:
1. Start the backend server
2. Start MQTT broker (via docker-compose)
3. Run this simulator
4. Data will be ingested and broadcast via WebSocket

## License

ISC
