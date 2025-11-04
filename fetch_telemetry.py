import time
import websocket

def on_message(ws, message):
    print("Received:", message)

def on_error(ws, error):
    print("Error:", error)

def on_close(ws, close_status_code, close_msg):
    print("Closed:", close_status_code, close_msg)

def on_open(ws):
    print("Connection opened")

# Replace 'ws://backend-url' with the actual WebSocket URL
url = "ws://backend-url"

ws = websocket.WebSocketApp(
    url,
    on_message=on_message,
    on_error=on_error,
    on_close=on_close
)

ws.on_open = on_open

# Run the WebSocket for 10 seconds
try:
    ws.run_forever()
    time.sleep(10)
finally:
    ws.close()