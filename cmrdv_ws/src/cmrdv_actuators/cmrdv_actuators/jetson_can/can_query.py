import asyncio
import time
import os
import cantools
import can_utils

db = cantools.database.load_file(os.path.join(os.getcwd(), "CMR_19e.dbc"))

lookup_dict = {}

MAX_TIMEOUT : int = 20000

for message in db.messages:
    lookup_dict[message.name] = (message.frame_id, message.cycle_time if message.cycle_time else MAX_TIMEOUT // 10)
    for signal in message.signals:
        lookup_dict[signal.name] = (message.frame_id, message.cycle_time)


async def query_message(message: str):
    if message not in lookup_dict:
        raise Exception(f"No {message} found in {lookup_dict.keys()}")
    can_id, cycle_time = lookup_dict[message]
    #print(can_id, cycle_time)
    # can_id = 1
    timeout: int = min(cycle_time * 10, 20000)
    # timeout = 2000000
    can_data = await can_utils.get_data(can_id=can_id, timeout=timeout)
    #print(can_data.data)
    return db.decode_message(can_id, can_data.data, decode_containers=True)

def send_message(message : str, data : dict):
    if message not in lookup_dict:
        raise Exception(f"No {message} found in {lookup_dict.keys()}")
    can_id, _ = lookup_dict[message]
    can_data = db.encode_message(can_id, data, strict=True)
    print(can_data)
    can_utils.send_data(can_id=can_id, data=can_data)

async def test_query():
    while True:
        for data in asyncio.as_completed([query_message(message.name) for message in db.messages]):
            data = await data
            print(data)

if __name__ == '__main__':
    # asyncio.run(test_query())
    # while True:
        # send_message("CDC_WheelSpeeds", {"CDC_wheelFrontLeft" : 100, 
                                        # "CDC_wheelFrontRight" : 100,
                                        # "CDC_wheelBackLeft" : 100,
                                        # "CDC_wheelBackRight" : 100,
                                        # }
            # )
    message = "FSM_Data"
    #message = "FSM_PedalsADC"
    
    time_fired = time.time()
    while True:
        data = asyncio.run(query_message(message))
        if (True):
        	print(data)
        	time_fired = time.time()
