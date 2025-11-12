# Simulator ↔ Backend Compatibility

## ✅ Verified: Simulator CAN work with Backend

The `drone_mqtt_simulator.py` is fully compatible with the `Backend_Tesa` backend.

## Configuration Match

| Component | Simulator | Backend | Status |
|-----------|-----------|---------|--------|
| **MQTT Broker** | `localhost:1883` | `localhost:1883` | ✅ Match |
| **Topic** | `drones/frames` (default) | `drones/frames` (default) | ✅ Match |
| **Message Format** | Frame structure | Frame schema | ✅ Match |
| **Token Handling** | Sends `token_id` as string | Extracts token from payload | ✅ Match |
| **Camera Info** | Not included (fetched by backend) | Fetched from API | ✅ Match |

## Message Format

The simulator sends messages in this format:
```json
{
  "fram_id": "string",
  "cam_id": "e8a76237-df96-4a6a-9375-baa4d74f5f12",
  "token_id": "257c87b4-9469-44fe-9132-8937f69723bd",
  "timestamp": "ISO 8601 string",
  "image_info": {
    "width": 1920,
    "height": 1080
  },
  "objects": [
    {
      "obj_id": "string",
      "type": "string",
      "lat": 13.7563,
      "lng": 100.5018,
      "alt": 120.0,
      "speed_kt": 15.0
    }
  ]
}
```

The backend:
1. Receives the message via MQTT
2. Extracts `cam_id` and `token_id` from payload
3. Calls API: `GET https://tesa-api.crma.dev/api/object-detection/info/{cam_id}`
4. Maps API response to `token_id.camera_info` structure
5. Validates with `frameSchema`
6. Saves to database

## How to Test

### 1. Start Backend Services
```bash
cd Backend_Tesa
docker-compose up -d db mqtt
npm run dev
```

### 2. Run Simulator (Quick Test - 5 frames)
```bash
cd SimulationData_Tesa
python drone_mqtt_simulator.py \
  --mode frames \
  --host localhost \
  --topic drones/frames \
  --center-lat 13.7563 \
  --center-lon 100.5018 \
  --num-drones 1 \
  --interval-s 1.0 \
  --radius-m 120 \
  --updates 5 \
  --cam-id e8a76237-df96-4a6a-9375-baa4d74f5f12 \
  --token 257c87b4-9469-44fe-9132-8937f69723bd
```

### 3. Run Simulator (Continuous)
```bash
cd SimulationData_Tesa
python drone_mqtt_simulator.py \
  --mode frames \
  --host localhost \
  --topic drones/frames \
  --center-lat 13.7563 \
  --center-lon 100.5018 \
  --num-drones 1 \
  --interval-s 0.5 \
  --radius-m 120
```

### 4. Verify Connection
```bash
cd SimulationData_Tesa
python verify-connection.py
```

## What to Check

When running the simulator, check the backend logs for:
- ✅ `📡 MQTT connected`
- ✅ `✅ MQTT subscribed to drones/frames`
- ✅ `📡 Fetching camera info from API...`
- ✅ `✅ API Response received`
- ✅ `✅ Mapped camera_info`
- ✅ `🖼️ Saved frame`

## Troubleshooting

### Simulator can't connect
- Check MQTT broker is running: `docker-compose ps` in Backend_Tesa
- Start broker: `docker-compose up -d mqtt`

### Backend not receiving messages
- Check backend is running: `npm run dev`
- Check MQTT subscription: Look for `✅ MQTT subscribed to drones/frames`
- Verify topic matches: Both should use `drones/frames`

### API call fails
- Check token is correct in simulator payload
- Verify API endpoint is accessible
- Check backend logs for API error details

## Features

✅ **Automatic Camera Info Fetching**: Backend fetches camera info from API using token  
✅ **Error Handling**: Falls back gracefully if API fails  
✅ **Logging**: Detailed logs for debugging  
✅ **Real-time Processing**: Frames processed as they arrive  
✅ **Database Storage**: Frames saved with camera info  

