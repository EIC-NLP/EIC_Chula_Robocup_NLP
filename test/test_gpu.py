import torch 
import tensorflow as tf
import os

os.system('clear')

print(torch.cuda.is_available())

from tensorflow.python.client import device_lib
print(device_lib.list_local_devices())
print("Num GPUs Available: ", len(tf.config.list_physical_devices('GPU')))