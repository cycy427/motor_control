import time

from channel import ChannelSubscriber, ChannelFactoryInitialize
from user_data import *


if __name__ == "__main__":
    ChannelFactoryInitialize()
    # Create a subscriber to subscribe the data defined in UserData class
    sub = ChannelSubscriber("topic_1", UserData)
    sub.Init()

    while True:
        msg = sub.Read()
        if msg is not None:
            print("Subscribe success. msg:", msg)
        else:
            print("No data subscribed.")
            break
    sub.Close()
