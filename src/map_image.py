from PIL import Image as Image
import ctypes
import numpy as np
from lab_utils.plan_utils import *
import multiprocessing as mp
import time
from lab_utils.global_values import *
from lab_utils.image_utils import *

class MAP_IMG:   
   _robot_position = mp.Array("d", 2)
   _process = None
   _running = mp.Value("i", 1)
   _tag = mp.Array("d", 4)
   _nbTag = mp.Value("i", 1)
   size_robot = (12, 12)
	
   def __init__(self, map_source):
      self.robot = Image.new(mode = "RGB", size = self.size_robot, color = (255, 0, 0))
      self.path_robot = Image.new(mode = "RGB", size = (5, 5), color = (255, 0, 0))
      self.tag = Image.new(mode = "RGB", size = (8, 8), color = (0, 255, 0))

      self.original_map = Image.open(map_source)
      self.original_map = resize_img(self.original_map, MAP_SCALE)
      self.original_map = self.original_map.convert("RGB")
      self.map_size = self.original_map.size
      map_bytes = img_to_bytes(self.original_map)
      
      self._map_avec_robot = mp.RawArray(ctypes.c_wchar_p, len(map_bytes))
      self._map_avec_robot = map_bytes
      self._map_path = mp.RawArray(ctypes.c_wchar_p, len(map_bytes))
      self._map_path = map_bytes
      self._map_sans_robot = mp.RawArray(ctypes.c_wchar_p, len(map_bytes))
      self._map_sans_robot = map_bytes

      self._running.value = 0
      self._nbTag.value = 0
	

   # Send the tags detected to the map
   def send_tag(self, tag, nbTag):
      self._nbTag.value = nbTag
      
      for i in range(nbTag - 1):
         if tag[i] == 0 and tag[i + 1] == 0:
            break
         else:
            self._tag[i * 2]     = self.map_size[0]/2 + ((SIM_TO_REAL[0]/2 + tag[1]) * (MAP_SCALE / MAP_RES)) + 2
            self._tag[i * 2 + 1] = self.map_size[1]/2 - ((SIM_TO_REAL[1]/2 - tag[i + 1]) * (MAP_SCALE / MAP_RES)) - 2

   # Start the map process
   def start_map(self, robot = None):
      if robot is None:
         print("There is no robot connected")
      else:
         timer_start = time.perf_counter()
         max_time = 100
         while time.perf_counter() - timer_start < max_time and self._process is None:
            if robot._process is None:
               time.sleep(1)
            else:
               self._running.value = 1
               self._process = mp.Process(
                  target= self.loop_process,
                  args= (robot.positions, self._robot_position, self._map_avec_robot, self._map_sans_robot, self._running)
               )
               self._process.start()
            time.sleep(1)
   
   # Stop the map process
   def stop_map(self):
      if self._process is not None:
         self._running.value = 0

   # Main loop process to update the map display
   def loop_process(self, 
                    real_pos,
                    map_pos,
                    map_robot, 
                    map_sans, 
                    running):
      start_time = time.perf_counter()
      dt = 2.0
      while running.value == 1:
         if time.perf_counter() - start_time > dt:
            start_time = time.perf_counter()
      
   @property
   def map_avec_robot(self):
      return self._map_avec_robot

   @property
   def map_sans_robot(self):
      return self._map_sans_robot

   @property
   def map_path(self):
      return self._map_path