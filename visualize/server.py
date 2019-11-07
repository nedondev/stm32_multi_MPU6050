import asyncio
import socketio
import uvicorn
import json
import settings
import sys

import python_serial as ps
import time

static_file = {
            '/': 'index.html',
            '/static': './static',
            '/css': './css'
        }
sio = socketio.AsyncServer( async_mode='asgi')
app = socketio.ASGIApp(sio, static_files=static_file)

@sio.event
def my_event(sid, data):
    pass

@sio.on('message')
async def another_event(sid, data):
    # print('message_event from: ', sid)
    # print('data: ', data)
    if data['get'] == 'data':
        try:
            data = ps.read_all_filtered_out()
            # print(data)
            await send_information(sid,json.dumps(data))
        except :
            print("Unexpected error:", sys.exc_info())
            pass
    pass

@sio.on('ping_client')
def another_event(sid, data):
    print('ping_client: ',data)
    pass

@sio.event
async def connect(sid, environ):
    print('connect ', sid)
    await send_information(sid,json.dumps({"var1":1,"var2":9.043}))
    pass

@sio.event
async def disconnect(sid):
    print('disconnect ', sid)
    pass

async def send_information(sid, data):
    await sio.emit('message',data, room=sid)
    pass

if __name__ == '__main__':
    uvicorn.run(app, host=settings.IP_ADDRESS, port=settings.PORT)
    pass

